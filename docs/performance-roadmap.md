# Arena Camera ROS2 - Performance Profiling & Optimization Roadmap

## Context

The polarized camera node on NVIDIA Orin drops ~5.5 frames/sec at 10 FPS, meaning only ~4.5 FPS
gets through. The worker thread processing time exceeds the 100ms/frame budget. The single-slot
producer-consumer queue drops the oldest unprocessed frame when a new one arrives before the
worker finishes.

## Goals

- **Minimum:** Sustain 10 FPS with zero or near-zero drops (balanced latency/throughput)
- **Nice to have:** Sustain 15 FPS
- **All topics remain configurable** — user is still experimenting with which to enable
- **Cross-platform:** Must build and run on x86 laptops (no GPU) and NVIDIA Orin (with GPU)

---

## Real Profiling Data

### Baseline (before any optimization)

```
split=6ms  bgr=76ms  max=14ms  dolp_aolp=61ms  TOTAL=158ms  (~6.3 FPS)
dolp_aolp breakdown: stokes_conv=7ms stokes=5ms dolp=28ms aolp=21ms
```

### After Phase 1–3 (profiling + duplicate BGR fix + AoLP vectorization)

```
split=6ms  channels=76ms  max=14ms  dolp_aolp=61ms  TOTAL=158ms
dolp_aolp: stokes_conv=7ms stokes=5ms dolp=28ms aolp=21ms
```
*(profiling instrumentation only — no measurable improvement from Phase 2/3 alone at this point)*

### After Part B: nvJPEG GPU encoding (cuda-cudart-dev-12-6 + libnvjpeg-dev-12-6 + cuda-crt-12-6)

```
split=6ms  channels=51ms  max=8ms  dolp_aolp=55ms  TOTAL=120ms  (~8.3 FPS)
```
channels drop: Arena SDK debayer unchanged, but JPEG encodes now GPU. max improved.

### After CUDA fused Stokes+DOLP+AoLP kernel (BGR input — first attempt)

```
split=6ms  channels=51ms  max=13ms  dolp_aolp=38ms  TOTAL=108ms  (~9.3 FPS)
dolp_aolp: stokes_conv=32ms (BGR→gray upload) stokes=0ms dolp=0ms aolp=0ms
```
Kernel active but dominated by BGR→gray CPU loop + H2D upload.

### After profiling sub-breakdown (ch_debayer vs ch_jpeg separated)

```
split=6ms  ch_debayer=32ms  ch_jpeg=19ms  max=11ms  dolp_aolp=38ms  TOTAL=106ms
dolp_aolp: stokes_conv=32ms stokes=0ms dolp=0ms aolp=0ms
```

### After CUDA kernel: raw mono8 input (skip BGR→gray entirely) ← CURRENT

```
split=10ms  ch_debayer=40ms  ch_jpeg=19ms  max=12ms  dolp_aolp=10ms  TOTAL=92ms  (~10.9 FPS ✓)
dolp_aolp: stokes_conv=3ms stokes=0ms dolp=0ms aolp=0ms
```
**Target met: TOTAL < 100ms, 10 FPS sustained.**

---

## Packages Installed on Orin

```
sudo apt install cuda-cudart-dev-12-6 libnvjpeg-dev-12-6 cuda-crt-12-6 cuda-nvcc-12-6
```

nvcc path: `/usr/local/cuda-12.6/bin/nvcc` (added to `~/.bashrc` PATH).

Build command:
```bash
cd ros2_ws && PATH="/usr/local/cuda-12.6/bin:$PATH" colcon build --packages-select arena_camera_node
```

---

## Optimizations Implemented

### Phase 1: StageAccumulator profiling (DONE)
Per-frame timing at DEBUG level, 30-frame averages at INFO level.
`profile_processing: true` in camera.yaml to enable.

### Phase 2: Eliminate duplicate BGR8 conversion (DONE)
`cached_bgr[4]` array reuses per-channel BGR mats for max-combined image.
Saved ~30ms at the time.

### Phase 3: Vectorize AoLP with cv::phase() (DONE)
ARM NEON SIMD via `cv::phase(S1, S2)`. Replaced scalar `std::atan2` loop.

### Part A: Eliminate 4x duplicate Arena SDK Mono8 conversions (DONE)
`compute_and_publish_dolp_aolp_()` now derives gray via `cv::cvtColor(BGR2GRAY)`
from `cached_bgr` instead of calling `ImageFactory::Convert(→Mono8)` 4 more times.

### Part B: nvJPEG GPU encoding (DONE)
- `src/nvjpeg_encoder.h/.cpp` — `NvJpegEncoder` wrapper class
- `jpeg_encode_()` helper replaces all 7 `cv::imencode` calls
- CMakeLists.txt glob-based path detection for CUDA headers/libs
- Mono8 uses `nvjpegEncodeYUV(NVJPEG_CSS_GRAY)` — `NVJPEG_INPUT_Y` doesn't exist in this version
- Saved ~30ms on channels, ~8ms on max/dolp/aolp JPEG encodes

### Part C: CUDA fused Stokes+DOLP+AoLP kernel (DONE)
- `src/polarization_kernels.h/.cu` — kernel takes 4 raw mono8 planes directly
- Each thread: reads 4 channel bytes → Stokes → DOLP + AoLP → writes 2 output bytes
- Input: `channels[i]->GetData()` from Arena SplitChannels (BayerRG8, one byte per pixel)
- No BGR→gray conversion needed — raw luminance values feed Stokes math directly
- Pinned host staging buffer for fast H2D transfer
- `HAS_POLAR_KERNEL` define gates GPU path; CPU fallback always available
- `use_gpu_polar_` runtime flag; falls back to CPU on any kernel error
- Saved ~48ms on dolp_aolp (38ms → 10ms)

---

## Current Bottlenecks (as of TOTAL ~92ms)

| Stage | Time | Notes |
|-------|------|-------|
| split | 10ms | `ImageFactory::SplitChannels()` — Arena SDK, hard to replace |
| ch_debayer | 40ms | 4× `ImageFactory::Convert(BayerRG8→BGR8)` — CPU, ~10ms each |
| ch_jpeg | 19ms | 4× nvJPEG BGR encode — sequential (single nvJPEG stream) |
| max | 12ms | `cv::max` cascade + 1× nvJPEG BGR encode |
| dolp_aolp | 10ms | GPU kernel (3ms) + 2× nvJPEG mono8 encode (~7ms) |

The `ch_debayer=40ms` is the dominant remaining bottleneck. It's the Arena SDK CPU Bayer→BGR
debayer, required for the per-channel JPEG topic output. Options to attack it:

---

## Remaining Optimization Options

### Option 1: CUDA Bayer debayering (est. -20ms)
Replace `ImageFactory::Convert(BayerRG8→BGR8)` with a CUDA Bayer→BGR kernel.
Each channel's BayerRG8 data is already available as `channels[i]->GetData()`.
Output feeds directly into nvJPEG for GPU-to-GPU encode (avoiding a round-trip to host).
Risk: Bayer demosaicing quality may differ from Arena SDK (bilinear vs higher-order).

### Option 2: Disable per-channel BGR topics (est. -40ms if ch_debayer + ch_jpeg eliminated)
If the 4× per-channel compressed images aren't needed, disabling them eliminates
both `ch_debayer` and `ch_jpeg` entirely. The raw mono8 data feeds the CUDA kernel
already — no debayer needed for DOLP/AoLP.
Set `publish_compressed: false` or add per-topic enable flags.

### Option 3: Async GPU pipeline (est. -10ms)
Overlap ch_debayer (CPU) with dolp_aolp GPU kernel using separate CUDA streams.
Currently sequential: debayer → ch_jpeg → kernel → dolp_jpeg.
With async: debayer CPU work runs while kernel executes on GPU simultaneously.

### Option 4: nvJPEG batch encoding (est. -5ms on ch_jpeg)
Use `nvjpegEncodeBatched()` to encode all 4 BGR channels in one GPU launch instead
of 4 sequential single-image encodes. Reduces CUDA launch overhead.

---

## Expected Impact Estimates (remaining)

| Optimization | Est. Savings | Est. Frame Time |
|-------------|-------------|-----------------|
| Current (after Part C) | — | ~92ms (~10.9 FPS ✓) |
| CUDA Bayer debayer (Option 1) | ~20ms | ~72ms (~13.9 FPS) |
| Disable ch topics (Option 2) | ~40ms | ~52ms (~19.2 FPS) |
| Async GPU pipeline (Option 3) | ~10ms | ~62ms (~16.1 FPS) |
| nvJPEG batch (Option 4) | ~5ms | ~87ms (~11.5 FPS) |

---

## Verification

1. Build: `PATH="/usr/local/cuda-12.6/bin:$PATH" colcon build --packages-select arena_camera_node`
2. Run node with `profile_processing: true`, collect per-stage timing at 10 FPS
3. Startup log should show:
   - `nvJPEG hardware JPEG encoding enabled (GPU)`
   - `CUDA polarization kernel enabled (fused Stokes+DOLP+AoLP)`
4. **x86 laptop:** Builds cleanly without CUDA, CPU path active, `profile_processing` works identically
