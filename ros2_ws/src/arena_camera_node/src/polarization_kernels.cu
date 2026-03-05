#ifdef HAS_CUDA

#include "polarization_kernels.h"
#include <cuda_runtime.h>
#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// Fused Stokes + DOLP + AoLP kernel
//
// Input: 4 packed mono8 planes [ch0 | ch1 | ch2 | ch3], each plane_pixels bytes.
// Each plane is the raw output of Arena::ImageFactory::SplitChannels() — one
// byte per pixel, already extracted from the 2×2 polarization superpixel.
// The raw value is a direct luminance sample; no debayering needed for Stokes.
//
// S0 = ch0 + ch2   (0° + 90°  — total intensity)
// S1 = ch0 - ch2   (0° - 90°)
// S2 = ch1 - ch3   (45° - 135°)
//
// DOLP = sqrt(S1²+S2²)/S0  clamped [0,1] → [0,255]
// AoLP = 0.5·atan2(S2,S1) mapped [0,255]: phase/(2π)+0.5, wrap, ×255
// ---------------------------------------------------------------------------

__global__ void polar_kernel(
    const uint8_t* __restrict__ d_in,  // 4 packed mono8 planes
    size_t plane_pixels,
    uint8_t* __restrict__ d_dolp,
    uint8_t* __restrict__ d_aolp)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= (int)plane_pixels) return;

    const float g0   = d_in[idx];                        // 0°
    const float g45  = d_in[plane_pixels     + idx];     // 45°
    const float g90  = d_in[plane_pixels * 2 + idx];     // 90°
    const float g135 = d_in[plane_pixels * 3 + idx];     // 135°

    const float S0 = g0  + g90;
    const float S1 = g0  - g90;
    const float S2 = g45 - g135;

    // DOLP
    const float s0_safe = (S0 > 1e-5f) ? S0 : 1e-5f;
    float dolp = __fsqrt_rn(S1 * S1 + S2 * S2) / s0_safe;
    if (S0 <= 1e-5f) dolp = 0.0f;
    dolp = fminf(fmaxf(dolp, 0.0f), 1.0f);
    d_dolp[idx] = (uint8_t)(dolp * 255.0f + 0.5f);

    // AoLP
    constexpr float inv_2pi = 0.15915494309f;
    float phase = atan2f(S2, S1) * inv_2pi + 0.5f;
    if (phase >= 1.0f) phase -= 1.0f;
    if (phase < 0.0f)  phase += 1.0f;
    d_aolp[idx] = (uint8_t)fminf(phase * 255.0f + 0.5f, 255.0f);
}

// ---------------------------------------------------------------------------
// PolarKernelBuffers
// ---------------------------------------------------------------------------

void PolarKernelBuffers::free() {
    if (d_in)   { cudaFree(d_in);   d_in   = nullptr; }
    if (d_dolp) { cudaFree(d_dolp); d_dolp = nullptr; }
    if (d_aolp) { cudaFree(d_aolp); d_aolp = nullptr; }
    plane_bytes = 0;
}

bool PolarKernelBuffers::ensure(size_t w, size_t h) {
    const size_t need = w * h;
    if (need <= plane_bytes) return true;
    free();
    if (cudaMalloc(reinterpret_cast<void**>(&d_in),   4 * need) != cudaSuccess) return false;
    if (cudaMalloc(reinterpret_cast<void**>(&d_dolp), need)     != cudaSuccess) { free(); return false; }
    if (cudaMalloc(reinterpret_cast<void**>(&d_aolp), need)     != cudaSuccess) { free(); return false; }
    plane_bytes = need;
    return true;
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

bool polar_compute_gpu(
    const uint8_t* const ch[4],
    int w, int h,
    uint8_t* dolp_out,
    uint8_t* aolp_out,
    PolarKernelBuffers& bufs)
{
    const size_t npix = static_cast<size_t>(w) * h;
    if (!bufs.ensure(w, h)) return false;

    // Upload all 4 planes in one contiguous transfer.
    // Arena SplitChannels data is contiguous (w*h bytes, no row padding).
    // Pack into a single pinned staging buffer for maximum PCIe/NVLink throughput.
    static uint8_t* h_staging = nullptr;
    static size_t   h_staging_sz = 0;
    const size_t total = 4 * npix;
    if (total > h_staging_sz) {
        if (h_staging) cudaFreeHost(h_staging);
        if (cudaMallocHost(reinterpret_cast<void**>(&h_staging), total) != cudaSuccess)
            return false;
        h_staging_sz = total;
    }
    for (int i = 0; i < 4; ++i) {
        std::memcpy(h_staging + i * npix, ch[i], npix);
    }
    if (cudaMemcpy(bufs.d_in, h_staging, total, cudaMemcpyHostToDevice) != cudaSuccess)
        return false;

    // Launch kernel
    const int threads = 256;
    const int blocks  = static_cast<int>((npix + threads - 1) / threads);
    polar_kernel<<<blocks, threads>>>(bufs.d_in, npix, bufs.d_dolp, bufs.d_aolp);
    if (cudaGetLastError() != cudaSuccess) return false;

    // Download results
    if (cudaMemcpy(dolp_out, bufs.d_dolp, npix, cudaMemcpyDeviceToHost) != cudaSuccess) return false;
    if (cudaMemcpy(aolp_out, bufs.d_aolp, npix, cudaMemcpyDeviceToHost) != cudaSuccess) return false;

    return true;
}

#endif  // HAS_CUDA
