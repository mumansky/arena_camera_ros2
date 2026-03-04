#pragma once

#ifdef HAS_CUDA

#include <nvjpeg.h>
#include <cuda_runtime.h>
#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>

// Hardware JPEG encoder using NVIDIA nvJPEG.
//
// Supports CV_8UC3 (BGR interleaved) and CV_8UC1 (mono8 / grayscale) inputs.
// Encode quality is set at construction time (matches jpeg_quality_ YAML param).
//
// Usage:
//   NvJpegEncoder enc(80);       // quality 80
//   if (enc.is_valid()) {
//     std::vector<uint8_t> buf;
//     enc.encode(bgr_mat, buf);  // true on success
//   }
//
// Not copyable; owns CUDA resources (handle, stream, GPU scratch buffer).
// All methods are single-threaded; call only from the worker thread.
class NvJpegEncoder {
public:
    explicit NvJpegEncoder(int quality = 80);
    ~NvJpegEncoder();
    NvJpegEncoder(const NvJpegEncoder&) = delete;
    NvJpegEncoder& operator=(const NvJpegEncoder&) = delete;

    // Returns true if initialization succeeded and encoding is available.
    bool is_valid() const { return valid_; }

    // Encode img (CV_8UC3 BGR or CV_8UC1 mono8) into JPEG bytes.
    // Returns true and fills `out` on success.
    // Returns false on error — caller should fall back to cv::imencode.
    bool encode(const cv::Mat& img, std::vector<uint8_t>& out);

private:
    bool ensure_buffer(size_t bytes);
    void cleanup();

    nvjpegHandle_t        handle_{};
    nvjpegEncoderState_t  state_{};
    nvjpegEncoderParams_t params_{};
    cudaStream_t          stream_{nullptr};
    unsigned char*        d_buf_{nullptr};
    size_t                d_buf_size_{0};
    int                   quality_{80};
    bool                  valid_{false};
};

#endif  // HAS_CUDA
