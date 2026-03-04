#ifdef HAS_CUDA

#include "nvjpeg_encoder.h"
#include <cstring>

NvJpegEncoder::NvJpegEncoder(int quality) : quality_(quality) {
    if (nvjpegCreateSimple(&handle_) != NVJPEG_STATUS_SUCCESS) {
        return;
    }
    if (cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking) != cudaSuccess) {
        cleanup(); return;
    }
    if (nvjpegEncoderStateCreate(handle_, &state_, stream_) != NVJPEG_STATUS_SUCCESS) {
        cleanup(); return;
    }
    if (nvjpegEncoderParamsCreate(handle_, &params_, stream_) != NVJPEG_STATUS_SUCCESS) {
        cleanup(); return;
    }
    if (nvjpegEncoderParamsSetQuality(params_, quality_, stream_) != NVJPEG_STATUS_SUCCESS) {
        cleanup(); return;
    }
    // Optimized Huffman tables give slightly smaller files at negligible extra cost.
    nvjpegEncoderParamsSetOptimizedHuffman(params_, 1, stream_);

    valid_ = true;
}

NvJpegEncoder::~NvJpegEncoder() {
    cleanup();
}

void NvJpegEncoder::cleanup() {
    if (d_buf_) { cudaFree(d_buf_); d_buf_ = nullptr; d_buf_size_ = 0; }
    if (params_) { nvjpegEncoderParamsDestroy(params_); params_ = {}; }
    if (state_)  { nvjpegEncoderStateDestroy(state_);   state_  = {}; }
    if (stream_) { cudaStreamDestroy(stream_);           stream_ = nullptr; }
    if (handle_) { nvjpegDestroy(handle_);               handle_ = {}; }
    valid_ = false;
}

bool NvJpegEncoder::ensure_buffer(size_t bytes) {
    if (bytes <= d_buf_size_) return true;
    if (d_buf_) { cudaFree(d_buf_); d_buf_ = nullptr; d_buf_size_ = 0; }
    if (cudaMalloc(reinterpret_cast<void**>(&d_buf_), bytes) != cudaSuccess) return false;
    d_buf_size_ = bytes;
    return true;
}

bool NvJpegEncoder::encode(const cv::Mat& img, std::vector<uint8_t>& out) {
    if (!valid_ || img.empty()) return false;

    const int w  = img.cols;
    const int h  = img.rows;
    const int ch = img.channels();
    if (ch != 1 && ch != 3) return false;

    // Upload image to GPU (cudaMemcpy2D handles non-contiguous cv::Mat step).
    const size_t row_bytes = static_cast<size_t>(w) * ch;
    if (!ensure_buffer(row_bytes * h)) return false;
    if (cudaMemcpy2D(d_buf_, row_bytes,
                     img.data, img.step,
                     row_bytes, h,
                     cudaMemcpyHostToDevice) != cudaSuccess) {
        return false;
    }

    nvjpegImage_t nv_img;
    std::memset(&nv_img, 0, sizeof(nv_img));
    nv_img.channel[0] = d_buf_;
    nv_img.pitch[0]   = static_cast<unsigned int>(row_bytes);

    // Encode: mono8 uses nvjpegEncodeYUV (Y-plane only, NVJPEG_CSS_GRAY);
    // BGR uses nvjpegEncodeImage with NVJPEG_INPUT_BGRI (interleaved, 4:2:0).
    // NVJPEG_INPUT_Y does not exist in this nvJPEG version — grayscale must
    // go through nvjpegEncodeYUV with chroma subsampling = NVJPEG_CSS_GRAY.
    if (ch == 1) {
        if (nvjpegEncoderParamsSetSamplingFactors(params_, NVJPEG_CSS_GRAY, stream_) != NVJPEG_STATUS_SUCCESS) {
            return false;
        }
        if (nvjpegEncodeYUV(handle_, state_, params_, &nv_img, NVJPEG_CSS_GRAY, w, h, stream_) != NVJPEG_STATUS_SUCCESS) {
            return false;
        }
    } else {
        if (nvjpegEncoderParamsSetSamplingFactors(params_, NVJPEG_CSS_420, stream_) != NVJPEG_STATUS_SUCCESS) {
            return false;
        }
        if (nvjpegEncodeImage(handle_, state_, params_, &nv_img, NVJPEG_INPUT_BGRI, w, h, stream_) != NVJPEG_STATUS_SUCCESS) {
            return false;
        }
    }

    // First call: get compressed size (data == nullptr).
    size_t length = 0;
    if (nvjpegEncodeRetrieveBitstream(handle_, state_, nullptr, &length, stream_) != NVJPEG_STATUS_SUCCESS) {
        return false;
    }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) return false;

    // Second call: download compressed bitstream to host.
    out.resize(length);
    if (nvjpegEncodeRetrieveBitstream(handle_, state_, out.data(), &length, stream_) != NVJPEG_STATUS_SUCCESS) {
        return false;
    }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) return false;

    return true;
}

#endif  // HAS_CUDA
