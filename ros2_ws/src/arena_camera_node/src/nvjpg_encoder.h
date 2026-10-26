#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "nvjpg/nvjpg_plugin_api.h"

// JPEG encoder backed by the Orin's dedicated NVJPG engine.
//
// Same interface as NvJpegEncoder (CUDA nvJPEG), but frees the GPU and CPU:
// the encode runs on the fixed-function JPEG block. The NVIDIA library is
// loaded from libarena_nvjpg.so with dlopen(RTLD_LOCAL | RTLD_DEEPBIND) so its
// unversioned libjpeg symbols never collide with OpenCV's libjpeg.so.8 — see
// nvjpg/nvjpg_plugin_api.h.
//
// Mono8 input is encoded as YUV 4:2:0 with neutral chroma (NVJPG has no
// grayscale mode), so decoders see 3 components; it displays identically.
// Width and height must be even, otherwise encode() returns false and the
// caller falls back.
//
// Usage:
//   NvjpgEncoder enc(80, NvjpgEncoder::default_plugin_path());
//   if (enc.is_valid()) enc.encode(bgr_or_mono, buf);
//
// Not copyable. Single-threaded; call only from the worker thread.
class NvjpgEncoder {
public:
  NvjpgEncoder(int quality, const std::string& plugin_path);
  ~NvjpgEncoder();
  NvjpgEncoder(const NvjpgEncoder&) = delete;
  NvjpgEncoder& operator=(const NvjpgEncoder&) = delete;

  // libarena_nvjpg.so next to the running executable (install/lib/<pkg>/).
  static std::string default_plugin_path();

  bool is_valid() const { return enc_ != nullptr; }

  // Why construction or the last encode() failed ("" if it didn't).
  const std::string& error() const { return error_; }

  // Encode img (CV_8UC3 BGR or CV_8UC1 mono8) into JPEG bytes.
  // Returns false on error — caller should fall back to another encoder.
  bool encode(const cv::Mat& img, std::vector<uint8_t>& out);

private:
  using CreateFn = arena_nvjpg* (*)(int);
  using DestroyFn = void (*)(arena_nvjpg*);
  using EncodeFn = int (*)(arena_nvjpg*, int, int, const uint8_t*, int, const uint8_t*,
                           const uint8_t*, int, const uint8_t**, size_t*);
  using LastErrorFn = const char* (*)(const arena_nvjpg*);

  void* lib_{nullptr};
  arena_nvjpg* enc_{nullptr};
  DestroyFn destroy_{nullptr};
  EncodeFn encode_{nullptr};
  LastErrorFn last_error_{nullptr};
  // Reused BGR -> full-range YCbCr 4:2:0 conversion buffers.
  cv::Mat ycrcb_;
  cv::Mat planes_[3];
  cv::Mat cb_half_;
  cv::Mat cr_half_;
  std::string error_;
};
