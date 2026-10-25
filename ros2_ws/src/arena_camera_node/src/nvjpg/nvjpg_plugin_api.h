#pragma once

// C ABI between the node and libarena_nvjpg.so (the NVJPG hardware JPEG plugin).
//
// The plugin links NVIDIA's L4T libnvjpeg.so, which exports the whole libjpeg
// API (jpeg_*) without symbol versions. The node already has the system
// libjpeg.so.8 loaded through OpenCV, so the two would interpose on each other
// if they shared a symbol scope. NvjpgEncoder therefore dlopen()s the plugin
// with RTLD_LOCAL | RTLD_DEEPBIND and talks to it only through these
// functions — no C++ types or libjpeg structs cross the boundary.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct arena_nvjpg arena_nvjpg;

// Returns nullptr on failure (no NVJPG engine, allocation failure, ...).
arena_nvjpg* arena_nvjpg_create(int quality);
void arena_nvjpg_destroy(arena_nvjpg* enc);

// Encode one I420 (YUV 4:2:0 planar) frame. width and height must be even.
// Pass u = v = nullptr for a grayscale image: the plugin uses neutral chroma.
// On success returns 0 and points *out at the JPEG bytes, which stay valid
// until the next call on the same encoder. Returns non-zero on failure.
int arena_nvjpg_encode_i420(arena_nvjpg* enc, int width, int height,
                            const uint8_t* y, int y_step,
                            const uint8_t* u, const uint8_t* v, int uv_step,
                            const uint8_t** out, size_t* out_len);

// Last error message for this encoder ("" if none).
const char* arena_nvjpg_last_error(const arena_nvjpg* enc);

#ifdef __cplusplus
}
#endif
