#include "nvjpg_encoder.h"

#include <dlfcn.h>
#include <limits.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

NvjpgEncoder::NvjpgEncoder(int quality, const std::string& plugin_path) {
  // RTLD_DEEPBIND: the plugin's jpeg_* calls resolve to NVIDIA's libnvjpeg
  // (its own dependency) before the libjpeg.so.8 OpenCV already loaded.
  // RTLD_LOCAL: NVIDIA's symbols stay out of the global scope, so OpenCV's
  // cv::imencode/imdecode keep using libjpeg.so.8.
  lib_ = dlopen(plugin_path.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
  if (!lib_) {
    const char* e = dlerror();
    error_ = std::string("dlopen failed: ") + (e ? e : plugin_path);
    return;
  }
  auto create = reinterpret_cast<CreateFn>(dlsym(lib_, "arena_nvjpg_create"));
  destroy_ = reinterpret_cast<DestroyFn>(dlsym(lib_, "arena_nvjpg_destroy"));
  encode_ = reinterpret_cast<EncodeFn>(dlsym(lib_, "arena_nvjpg_encode_i420"));
  last_error_ = reinterpret_cast<LastErrorFn>(dlsym(lib_, "arena_nvjpg_last_error"));
  if (!create || !destroy_ || !encode_ || !last_error_) {
    error_ = "plugin is missing arena_nvjpg_* symbols";
    dlclose(lib_);
    lib_ = nullptr;
    return;
  }
  enc_ = create(quality);
  if (!enc_) {
    error_ = "NVJPG encoder creation failed (no JPEG engine?)";
    dlclose(lib_);
    lib_ = nullptr;
  }
}

NvjpgEncoder::~NvjpgEncoder() {
  if (enc_) destroy_(enc_);
  if (lib_) dlclose(lib_);
}

std::string NvjpgEncoder::default_plugin_path() {
  char exe[PATH_MAX];
  const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (n <= 0) return "libarena_nvjpg.so";
  std::string path(exe, static_cast<size_t>(n));
  return path.substr(0, path.find_last_of('/') + 1) + "libarena_nvjpg.so";
}

bool NvjpgEncoder::encode(const cv::Mat& img, std::vector<uint8_t>& out) {
  if (!enc_ || img.empty() || img.depth() != CV_8U) return false;
  const int w = img.cols;
  const int h = img.rows;
  if ((w & 1) || (h & 1)) return false;

  const uint8_t* y = nullptr;
  const uint8_t* u = nullptr;
  const uint8_t* v = nullptr;
  int y_step = 0;
  int uv_step = 0;
  if (img.channels() == 1) {
    y = img.data;
    y_step = static_cast<int>(img.step);
  } else if (img.channels() == 3) {
    // JPEG (JFIF) needs full-range BT.601 YCbCr. COLOR_BGR2YCrCb is exactly
    // that; COLOR_BGR2YUV_I420 is limited range (16-235) and washes colours
    // out by ~6%. Chroma is then box-filtered 2x2 to 4:2:0, as libjpeg does.
    cv::cvtColor(img, ycrcb_, cv::COLOR_BGR2YCrCb);
    cv::split(ycrcb_, planes_);  // Y, Cr, Cb
    cv::resize(planes_[1], cr_half_, cv::Size(w / 2, h / 2), 0, 0, cv::INTER_AREA);
    cv::resize(planes_[2], cb_half_, cv::Size(w / 2, h / 2), 0, 0, cv::INTER_AREA);
    y = planes_[0].data;
    y_step = static_cast<int>(planes_[0].step);
    u = cb_half_.data;
    v = cr_half_.data;
    uv_step = static_cast<int>(cb_half_.step);
  } else {
    return false;
  }

  const uint8_t* jpeg = nullptr;
  size_t len = 0;
  if (encode_(enc_, w, h, y, y_step, u, v, uv_step, &jpeg, &len) != 0) {
    error_ = last_error_(enc_);
    return false;
  }
  out.assign(jpeg, jpeg + len);
  return true;
}
