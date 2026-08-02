/**
 * @file arena_image_raii.h
 * @brief RAII wrappers for Arena SDK image objects to ensure exception safety
 *
 * This header provides RAII wrappers for Arena::IImage pointers to prevent
 * memory leaks when exceptions occur during image processing. The wrappers
 * use custom deleters that call Arena::ImageFactory::Destroy appropriately.
 */

#pragma once

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>
#include "ArenaApi.h"

namespace arena_camera {

/**
 * @brief Custom deleter for Arena::IImage objects created by ImageFactory::Convert
 * 
 * Arena SDK requires that images created by ImageFactory::Convert or
 * ImageFactory::SplitChannels must be destroyed using ImageFactory::Destroy.
 * This deleter ensures proper cleanup even when exceptions occur.
 */
struct ArenaImageDeleter {
  void operator()(Arena::IImage* image) const noexcept {
    if (image) {
      try {
        Arena::ImageFactory::Destroy(image);
      } catch (...) {
        // Suppress exceptions in destructor to maintain noexcept guarantee
      }
    }
  }
};

/**
 * @brief RAII wrapper for a single Arena::IImage pointer
 *
 * Use this wrapper when working with images returned by
 * Arena::ImageFactory::Convert or similar factory methods.
 *
 * Example usage:
 * @code
 *   ArenaImagePtr bgr_image(Arena::ImageFactory::Convert(src, 0x02180015));
 *   // Use bgr_image.get() to access the raw pointer
 *   // Image is automatically destroyed when bgr_image goes out of scope
 * @endcode
 */
using ArenaImagePtr = std::unique_ptr<Arena::IImage, ArenaImageDeleter>;

/**
 * @brief Create an ArenaImagePtr from a raw Arena::IImage pointer
 * @param image Raw pointer to an Arena::IImage (takes ownership)
 * @return RAII-wrapped image pointer
 */
inline ArenaImagePtr make_arena_image_ptr(Arena::IImage* image) {
  return ArenaImagePtr(image, ArenaImageDeleter{});
}

/**
 * @brief Take ownership of a vector of raw images (e.g. from SplitChannels)
 *
 * Example usage:
 * @code
 *   auto channels = own_arena_images(Arena::ImageFactory::SplitChannels(pImage));
 *   // channels[0]->GetData(), channels.size(), ...
 *   // All channels destroyed when 'channels' goes out of scope
 * @endcode
 */
inline std::vector<ArenaImagePtr> own_arena_images(std::vector<Arena::IImage*> images) {
  std::vector<ArenaImagePtr> owned;
  owned.reserve(images.size());
  for (auto* img : images) owned.push_back(make_arena_image_ptr(img));
  return owned;
}

}  // namespace arena_camera
