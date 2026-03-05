#pragma once

#ifdef HAS_CUDA

#include <cstdint>
#include <cstddef>

// GPU buffers owned by ArenaCameraNode for the fused Stokes+DOLP+AoLP kernel.
// Allocated once on first use and reused across frames (grow-only).
struct PolarKernelBuffers {
    // Device-side input: 4 mono8 planes, each w*h bytes, packed contiguously.
    uint8_t*  d_in{nullptr};      // [4 * w * h]  raw Bayer8 / mono8 channel data
    // Device-side output: one byte per pixel.
    uint8_t*  d_dolp{nullptr};    // [w * h]
    uint8_t*  d_aolp{nullptr};    // [w * h]
    size_t    plane_bytes{0};     // w * h — size of one input plane

    void free();
    // Ensure buffers are large enough for w*h pixels. No-op if already sufficient.
    bool ensure(size_t w, size_t h);
};

// Launch the fused Stokes+DOLP+AoLP kernel.
//
// Inputs  (host pointers, contiguous mono8 / BayerRG8, w*h bytes each):
//   ch[0..3]  — raw channel data from Arena::ImageFactory::SplitChannels()
//               Each plane is w*h bytes, one byte per pixel, no padding.
//   w, h      — image dimensions in pixels
//
// Outputs (host pointers, pre-allocated by caller, w*h bytes each):
//   dolp_out  — DOLP mapped to [0,255]
//   aolp_out  — AoLP mapped to [0,255]
//
// Returns false on any CUDA error.
bool polar_compute_gpu(
    const uint8_t* const ch[4],  // 4 contiguous mono8 input planes
    int w, int h,
    uint8_t* dolp_out,
    uint8_t* aolp_out,
    PolarKernelBuffers& bufs);

#endif  // HAS_CUDA
