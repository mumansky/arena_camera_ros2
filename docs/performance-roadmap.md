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

## What We're Implementing Now

**Phase 1 only:** Profiling instrumentation + the duplicate BGR8 quick fix (since it's an obvious
bug regardless of profiling results). GPU acceleration deferred until real timing data drives the decision.

---

## Phase 1: Profiling Instrumentation (Implement Now)

Add permanent per-stage timing to `process_copied_image_()` and `compute_and_publish_dolp_aolp_()`,
gated by a YAML config toggle.

### New YAML parameter

```yaml
# Processing Profiler
# -------------------
# Log per-frame timing breakdown for each processing stage.
# Useful for identifying bottlenecks. Adds negligible overhead (~microseconds).
profile_processing: false
```

**Files modified:**
- `ArenaCameraNode.h` — add `bool profile_processing_` member
- `ArenaCameraNode.cpp` — parse parameter in `parse_parameters_()`, add timing around each stage

### Stages to instrument

| Stage | Location (ArenaCameraNode.cpp) | Description |
|-------|-------------------------------|-------------|
| `split` | L1032-1033 | `ImageFactory::SplitChannels()` |
| `bgr` | L1053-1055 | 4x `ImageFactory::Convert(BayerRG8→BGR8)` |
| `ch_raw` | L1065-1077 | 4x raw channel memcpy + publish |
| `ch_jpeg` | L1078-1087 | 4x `cv::imencode(".jpg")` |
| `max_bgr` | L1091-1098 | 4x DUPLICATE BGR8 conversion (bug — Phase 2 removes this) |
| `max` | L1109-1112 | `cv::max()` cascade |
| `max_pub` | L1115-1135 | Max raw + JPEG publish |
| `stokes_conv` | L789-818 | 4x Mono8 convert + float32 convert |
| `stokes` | L820-823 | S0, S1, S2 matrix operations |
| `dolp` | L827-869 | DOLP compute + publish (raw + JPEG) |
| `aolp` | L877-915 | AoLP compute + publish (raw + JPEG) |

### Log output format (DEBUG level, one line per frame)

```
[DEBUG] Frame 42 timing (ms): split=8.2 bgr=32.1 ch_jpeg=24.5 max_bgr=31.8 max=3.1 stokes_conv=17.5 stokes=1.2 dolp=24.4 aolp=28.6 TOTAL=171.4
```

### Implementation approach

Use a lightweight helper to avoid cluttering the processing code:

```cpp
// In process_copied_image_() and compute_and_publish_dolp_aolp_():
struct StageTimer {
  std::chrono::steady_clock::time_point start;
  std::vector<std::pair<std::string, double>>& stages;
  bool enabled;
  void mark(const std::string& name) { /* record elapsed since last mark */ }
};
```

---

## Phase 2: Fix Duplicate BGR8 Conversion (Implement Now)

**Bug:** The max-combined image reconverts all 4 channels from Bayer→BGR8 (lines 1091-1098)
even though the per-channel loop already converted them (lines 1054-1055). This wastes ~30ms.

**Fix:** Cache the BGR8 `cv::Mat` data from the per-channel loop in an array, reuse for
max-combined. No new dependencies, no behavior change.

**File:** `ArenaCameraNode.cpp` lines 1036, 1053-1063, 1090-1112

---

## Future Phases (Deferred — Implement After Profiling Data)

### Phase 3: Vectorize AoLP Loop

The AoLP computation (line 885-890) uses a scalar per-pixel `std::atan2` loop. Replace with
`cv::phase(S2, S1, angle)` which uses SIMD (ARM NEON on Orin, SSE on x86). Trivial change,
estimated ~10-20ms savings.

**File:** `ArenaCameraNode.cpp` lines 877-890

### Phase 4: GPU Acceleration (CUDA)

Only pursue if profiling shows CPU optimizations can't hit 10 FPS target.

#### Cross-Platform Strategy

**Compile-time (CMakeLists.txt):** Auto-detect CUDA via `check_language(CUDA)`. No `nvcc`
on x86 → CUDA code not compiled. No manual flags needed.

```cmake
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    add_definitions(-DHAS_CUDA)
  endif()
endif()
```

**Runtime (camera.yaml):**

```yaml
# GPU Acceleration
# ----------------
# Controls whether GPU (CUDA) is used for image processing.
#   "auto"  (default): Use GPU if available, fall back to CPU
#   "gpu":  Force GPU — error if unavailable
#   "cpu":  Force CPU — ignore GPU even if present
gpu_acceleration: "auto"
```

**Runtime dispatch:** `m_use_gpu_` bool gates GPU code paths. CPU path is always the
existing (Phase 2-3 optimized) OpenCV code. `#ifdef HAS_CUDA` guards compilation.

#### 4a: Fused CUDA Kernel for Stokes + DOLP + AoLP

Single kernel: 4x Mono8 input → dolp_u8 + aolp_u8 output. Eliminates ~10 intermediate
cv::Mat allocations and fuses ~8 sequential operations.

**New file:** `src/cuda/polarization_kernels.cu/.h`

#### 4b: nvJPEG Hardware Encoding

Replace `cv::imencode(".jpg")` with `nvjpegEncode()` for all 6+ JPEG compressions per frame.

**New file:** `src/cuda/nvjpeg_encoder.cpp/.h`

#### 4c: GPU Debayering (conditional on profiling)

Only if `ImageFactory::Convert(BayerRG8→BGR8)` is a major bottleneck (>30ms).

### Phase 5: Pipeline Parallelism

Overlap CPU publishing with async GPU computation. Only relevant after Phase 4.

---

## Expected Impact Estimates

| Optimization | Est. Savings | Est. Frame Time |
|-------------|-------------|-----------------|
| Baseline (current) | — | ~170ms (~5.9 FPS) |
| Fix duplicate BGR8 (Phase 2) | ~30ms | ~140ms (~7.1 FPS) |
| Vectorize AoLP (Phase 3) | ~15ms | ~125ms (~8.0 FPS) |
| CUDA Stokes+DOLP+AoLP (Phase 4a) | ~50ms | ~75ms (~13.3 FPS) |
| nvJPEG (Phase 4b) | ~20ms | ~55ms (~18.2 FPS) |
| Pipeline overlap (Phase 5) | ~15ms | ~40ms (~25 FPS) |

*Estimates to be validated by Phase 1 profiling data.*

---

## Verification

1. Build and pass all tests: `./build_and_test.sh`
2. Run node with `profile_processing: true`, collect per-stage timing at 10 FPS
3. Confirm backpressure drop count decreases after Phase 2 fix
4. Check diagnostics for `Last Processing Time (ms)` and `Avg Processing Time (ms)`
5. **x86 laptop:** Builds cleanly without CUDA, `profile_processing` works identically
