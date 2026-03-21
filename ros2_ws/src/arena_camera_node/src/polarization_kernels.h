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
    // Optional device-side Stokes output (only allocated when publish_stokes_ is true)
    uint8_t*  d_s0{nullptr};      // [w * h]  S0 normalized to [0,255]
    uint8_t*  d_s1{nullptr};      // [w * h]  S1 shifted to [0,255] (128=zero)
    uint8_t*  d_s2{nullptr};      // [w * h]  S2 shifted to [0,255] (128=zero)
    size_t    plane_bytes{0};     // w * h — size of one input plane

    void free();
    // Ensure buffers are large enough for w*h pixels. No-op if already sufficient.
    // Pass want_stokes=true to also allocate d_s0/d_s1/d_s2.
    bool ensure(size_t w, size_t h, bool want_stokes = false);
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
//   s0_out    — S0 normalized: S0/2 → [0,255]. Pass nullptr to skip.
//   s1_out    — S1 shifted:   (S1+255)/2 → [0,255] (128 = zero). Pass nullptr to skip.
//   s2_out    — S2 shifted:   (S2+255)/2 → [0,255] (128 = zero). Pass nullptr to skip.
//
// Returns false on any CUDA error.
bool polar_compute_gpu(
    const uint8_t* const ch[4],  // 4 contiguous mono8 input planes
    int w, int h,
    uint8_t* dolp_out,
    uint8_t* aolp_out,
    PolarKernelBuffers& bufs,
    uint8_t* s0_out = nullptr,
    uint8_t* s1_out = nullptr,
    uint8_t* s2_out = nullptr);

#endif  // HAS_CUDA
