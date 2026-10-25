// NVJPG hardware JPEG encoder plugin (Jetson only).
//
// Built as libarena_nvjpg.so and loaded at runtime by NvjpgEncoder — see
// nvjpg_plugin_api.h for why it lives in its own isolated library.
//
// Frames are copied into a pitch-linear YUV420 NvBufSurface and encoded with
// NvJPEGEncoder::encodeFromFd, which reads the DMA buffer directly on the
// NVJPG engine (no GPU, no CPU entropy coding). Surfaces are cached per
// (size, grayscale) so frames of different shapes on alternating topics don't
// force reallocation, and grayscale surfaces get their neutral chroma once.

#include "nvjpg_plugin_api.h"

#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "NvJpegEncoder.h"
#include "nvbufsurface.h"

#define ARENA_NVJPG_EXPORT __attribute__((visibility("default")))

namespace {

constexpr size_t kMaxCachedSurfaces = 4;

struct Surface {
  int width = 0;
  int height = 0;
  bool gray = false;
  NvBufSurface* surf = nullptr;

  ~Surface() {
    if (surf) {
      NvBufSurfaceUnMap(surf, 0, -1);
      NvBufSurfaceDestroy(surf);
    }
  }
};

}  // namespace

struct arena_nvjpg {
  int quality = 80;
  std::unique_ptr<NvJPEGEncoder> encoder;
  std::vector<std::unique_ptr<Surface>> surfaces;  // most recently used last
  std::vector<unsigned char> out_buf;
  std::string error;

  Surface* get_surface(int w, int h, bool gray);
};

Surface* arena_nvjpg::get_surface(int w, int h, bool gray) {
  for (size_t i = 0; i < surfaces.size(); ++i) {
    if (surfaces[i]->width == w && surfaces[i]->height == h && surfaces[i]->gray == gray) {
      if (i + 1 != surfaces.size()) {
        auto s = std::move(surfaces[i]);
        surfaces.erase(surfaces.begin() + static_cast<long>(i));
        surfaces.push_back(std::move(s));
      }
      return surfaces.back().get();
    }
  }

  auto s = std::make_unique<Surface>();
  s->width = w;
  s->height = h;
  s->gray = gray;

  NvBufSurfaceAllocateParams params{};
  params.params.width = static_cast<uint32_t>(w);
  params.params.height = static_cast<uint32_t>(h);
  params.params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
  params.params.layout = NVBUF_LAYOUT_PITCH;
  params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
  params.memtag = NvBufSurfaceTag_JPEG;
  if (NvBufSurfaceAllocate(&s->surf, 1, &params) != 0) {
    s->surf = nullptr;
    error = "NvBufSurfaceAllocate failed";
    return nullptr;
  }
  s->surf->numFilled = 1;
  if (NvBufSurfaceMap(s->surf, 0, -1, NVBUF_MAP_READ_WRITE) != 0) {
    error = "NvBufSurfaceMap failed";
    return nullptr;
  }

  if (gray) {
    const NvBufSurfacePlaneParams& pp = s->surf->surfaceList[0].planeParams;
    for (int pl = 1; pl < 3; ++pl) {
      std::memset(s->surf->surfaceList[0].mappedAddr.addr[pl], 128,
                  static_cast<size_t>(pp.pitch[pl]) * pp.height[pl]);
    }
    NvBufSurfaceSyncForDevice(s->surf, 0, 1);
    NvBufSurfaceSyncForDevice(s->surf, 0, 2);
  }

  if (surfaces.size() >= kMaxCachedSurfaces) surfaces.erase(surfaces.begin());
  surfaces.push_back(std::move(s));
  return surfaces.back().get();
}

static void copy_plane(uint8_t* dst, uint32_t dst_pitch, const uint8_t* src, int src_step,
                       int row_bytes, int rows) {
  for (int r = 0; r < rows; ++r) {
    std::memcpy(dst + static_cast<size_t>(r) * dst_pitch,
                src + static_cast<size_t>(r) * src_step, static_cast<size_t>(row_bytes));
  }
}

extern "C" {

ARENA_NVJPG_EXPORT arena_nvjpg* arena_nvjpg_create(int quality) {
  auto enc = std::make_unique<arena_nvjpg>();
  enc->quality = quality;
  enc->encoder.reset(NvJPEGEncoder::createJPEGEncoder("arena_nvjpg"));
  if (!enc->encoder) return nullptr;
  return enc.release();
}

ARENA_NVJPG_EXPORT void arena_nvjpg_destroy(arena_nvjpg* enc) {
  delete enc;
}

ARENA_NVJPG_EXPORT int arena_nvjpg_encode_i420(arena_nvjpg* enc, int width, int height,
                                               const uint8_t* y, int y_step,
                                               const uint8_t* u, const uint8_t* v, int uv_step,
                                               const uint8_t** out, size_t* out_len) {
  if (!enc || !y || !out || !out_len) return -1;
  enc->error.clear();
  if (width <= 0 || height <= 0 || (width & 1) || (height & 1)) {
    enc->error = "width and height must be positive and even";
    return -1;
  }
  const bool gray = (u == nullptr || v == nullptr);

  Surface* s = enc->get_surface(width, height, gray);
  if (!s) return -1;

  NvBufSurfaceParams& sp = s->surf->surfaceList[0];
  copy_plane(static_cast<uint8_t*>(sp.mappedAddr.addr[0]), sp.planeParams.pitch[0],
             y, y_step, width, height);
  NvBufSurfaceSyncForDevice(s->surf, 0, 0);
  if (!gray) {
    copy_plane(static_cast<uint8_t*>(sp.mappedAddr.addr[1]), sp.planeParams.pitch[1],
               u, uv_step, width / 2, height / 2);
    copy_plane(static_cast<uint8_t*>(sp.mappedAddr.addr[2]), sp.planeParams.pitch[2],
               v, uv_step, width / 2, height / 2);
    NvBufSurfaceSyncForDevice(s->surf, 0, 1);
    NvBufSurfaceSyncForDevice(s->surf, 0, 2);
  }

  // Worst case for a JPEG of this frame is well under the raw I420 size.
  const size_t cap = static_cast<size_t>(width) * height * 3 / 2;
  if (enc->out_buf.size() < cap) enc->out_buf.resize(cap);
  unsigned char* buf = enc->out_buf.data();
  unsigned long len = static_cast<unsigned long>(enc->out_buf.size());

  if (enc->encoder->encodeFromFd(static_cast<int>(sp.bufferDesc), JCS_YCbCr, &buf, len,
                                 enc->quality) < 0) {
    enc->error = "encodeFromFd failed";
    return -1;
  }
  // libjpeg's memory destination mallocs a new buffer if ours was too small.
  if (buf != enc->out_buf.data()) {
    enc->out_buf.assign(buf, buf + len);
    std::free(buf);
  }
  *out = enc->out_buf.data();
  *out_len = static_cast<size_t>(len);
  return 0;
}

ARENA_NVJPG_EXPORT const char* arena_nvjpg_last_error(const arena_nvjpg* enc) {
  return enc ? enc->error.c_str() : "null encoder";
}

}  // extern "C"
