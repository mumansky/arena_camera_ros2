/**
 * @file arena_image_raii.h
 * @brief RAII wrappers for Arena SDK image objects to ensure exception safety
 *
 * This header provides RAII wrappers for Arena::IImage pointers to prevent
 * memory leaks when exceptions occur during image processing. The wrappers
 * use custom deleters that call Arena::ImageFactory::Destroy appropriately.
 */

#pragma once

#include <memory>
#include <vector>
#include <utility>
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
 * @brief RAII wrapper for a vector of Arena::IImage pointers
 *
 * This class manages a vector of Arena::IImage pointers, ensuring all
 * images are properly destroyed when the container goes out of scope.
 * This is particularly useful for managing the output of
 * Arena::ImageFactory::SplitChannels which returns a vector of images.
 *
 * Example usage:
 * @code
 *   std::vector<Arena::IImage*> raw_channels = Arena::ImageFactory::SplitChannels(pImage);
 *   ArenaImageVector channels(std::move(raw_channels));
 *   // Access channels with channels[0], channels[1], etc.
 *   // All channels are automatically destroyed when 'channels' goes out of scope
 * @endcode
 */
class ArenaImageVector {
 public:
  /**
   * @brief Construct an empty ArenaImageVector
   */
  ArenaImageVector() = default;

  /**
   * @brief Construct from a vector of raw image pointers (takes ownership)
   * @param images Vector of raw Arena::IImage pointers
   */
  explicit ArenaImageVector(std::vector<Arena::IImage*>&& images)
      : images_(std::move(images)) {}

  /**
   * @brief Destructor - destroys all owned images
   */
  ~ArenaImageVector() noexcept {
    clear();
  }

  // Non-copyable
  ArenaImageVector(const ArenaImageVector&) = delete;
  ArenaImageVector& operator=(const ArenaImageVector&) = delete;

  // Moveable
  ArenaImageVector(ArenaImageVector&& other) noexcept
      : images_(std::move(other.images_)) {
    other.images_.clear();
  }

  ArenaImageVector& operator=(ArenaImageVector&& other) noexcept {
    if (this != &other) {
      clear();
      images_ = std::move(other.images_);
      other.images_.clear();
    }
    return *this;
  }

  /**
   * @brief Add an image to the vector (takes ownership)
   * @param image Raw pointer to an Arena::IImage
   */
  void push_back(Arena::IImage* image) {
    images_.push_back(image);
  }

  /**
   * @brief Access an image by index
   * @param index Index of the image
   * @return Pointer to the image (ownership retained by this container)
   */
  Arena::IImage* operator[](size_t index) const {
    return images_[index];
  }

  /**
   * @brief Get the number of images in the vector
   * @return Number of images
   */
  size_t size() const noexcept {
    return images_.size();
  }

  /**
   * @brief Check if the vector is empty
   * @return true if empty, false otherwise
   */
  bool empty() const noexcept {
    return images_.empty();
  }

  /**
   * @brief Clear all images and destroy them
   */
  void clear() noexcept {
    for (auto* img : images_) {
      if (img) {
        try {
          Arena::ImageFactory::Destroy(img);
        } catch (...) {
          // Suppress exceptions during cleanup
        }
      }
    }
    images_.clear();
  }

  /**
   * @brief Get iterator to the beginning
   * @return Iterator to the first element
   */
  auto begin() noexcept { return images_.begin(); }
  auto begin() const noexcept { return images_.begin(); }

  /**
   * @brief Get iterator to the end
   * @return Iterator past the last element
   */
  auto end() noexcept { return images_.end(); }
  auto end() const noexcept { return images_.end(); }

 private:
  std::vector<Arena::IImage*> images_;
};

}  // namespace arena_camera
