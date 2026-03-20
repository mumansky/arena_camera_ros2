/**
 * @file pixel_format_helpers.h
 * @brief Pixel format constants and detection helpers
 *
 * PFNC (PixelFormat Naming Convention) constants used by Arena SDK,
 * plus lightweight utility functions for format detection and naming.
 * Header-only, no Arena SDK dependency — safe to use in tests.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

// ============================================================================
// Pixel Format Constants
// ============================================================================

namespace PixelFormat {
  // PolarizedAngles_0d_45d_90d_135d_BayerRG8 format
  // Used by polarized cameras (e.g., PHX050S1-QC) that capture 4 polarization
  // angles (0°, 45°, 90°, 135°) in a single Bayer pattern image.
  constexpr uint64_t PFNC_POLARIZED_BAYER_RG8 = 0x8220020F;

  // BGR8 (PFNC value 0x02180015): 3-channel Blue-Green-Red, 8 bits per channel.
  // Standard output format for OpenCV and compressed image publishing.
  constexpr uint64_t BGR8 = 0x02180015;

  // Mono8 (PFNC value 0x01080001): single-channel 8-bit grayscale.
  constexpr uint64_t MONO8 = 0x01080001;
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Check if the given pixel format is a polarized format
 * @param format PFNC pixel format value from Arena SDK
 * @return true if the format is a polarized format
 */
inline bool is_polarized_format(uint64_t format) {
  return format == PixelFormat::PFNC_POLARIZED_BAYER_RG8;
}

/**
 * @brief Get a human-readable name for a pixel format
 * @param format PFNC pixel format value from Arena SDK
 * @return String describing the pixel format
 */
inline std::string get_pixel_format_name(uint64_t format) {
  if (format == PixelFormat::PFNC_POLARIZED_BAYER_RG8) {
    return "PolarizedAngles_0d_45d_90d_135d_BayerRG8";
  } else if (format == PixelFormat::BGR8) {
    return "BGR8";
  } else {
    char buf[32];
    snprintf(buf, sizeof(buf), "0x%08lX", static_cast<unsigned long>(format));
    return std::string("Unknown (") + buf + ")";
  }
}
