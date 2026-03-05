/**
 * @file test_arena_image_raii.cpp
 * @brief Unit tests for Arena image RAII wrapper classes
 * 
 * These tests verify the RAII behavior of ArenaImagePtr and ArenaImageVector.
 * Since we can't create real Arena::IImage objects without the SDK runtime,
 * we test the wrapper logic using nullptr and verify the RAII pattern works
 * correctly for edge cases.
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>

// Include the RAII wrapper header
#include "arena_image_raii.h"

/**
 * @brief Test that ArenaImageDeleter handles nullptr correctly
 */
TEST(ArenaImageRaiiTest, DeleterHandlesNullptr)
{
  // Test that the deleter does not crash when given nullptr
  arena_camera::ArenaImageDeleter deleter;
  
  // This should not crash or throw - nullptr should be handled gracefully
  EXPECT_NO_THROW(deleter(nullptr));
}

/**
 * @brief Test ArenaImagePtr with nullptr
 */
TEST(ArenaImageRaiiTest, ArenaImagePtrWithNullptr)
{
  // Create an ArenaImagePtr with nullptr
  arena_camera::ArenaImagePtr ptr(nullptr);
  
  // Verify it's null
  EXPECT_EQ(ptr.get(), nullptr);
  
  // Destruction should not crash (implicit test when ptr goes out of scope)
}

/**
 * @brief Test ArenaImagePtr move semantics
 */
TEST(ArenaImageRaiiTest, ArenaImagePtrMoveSemantics)
{
  // Create an ArenaImagePtr with nullptr
  arena_camera::ArenaImagePtr ptr1(nullptr);
  
  // Move to another instance
  arena_camera::ArenaImagePtr ptr2 = std::move(ptr1);
  
  // Verify the move worked
  EXPECT_EQ(ptr2.get(), nullptr);
  
  // ptr1 should now be null after move (unique_ptr guarantee)
  EXPECT_EQ(ptr1.get(), nullptr);
}

/**
 * @brief Test make_arena_image_ptr helper function
 */
TEST(ArenaImageRaiiTest, MakeArenaImagePtrHelper)
{
  // Test the helper function with nullptr
  auto ptr = arena_camera::make_arena_image_ptr(nullptr);
  
  EXPECT_EQ(ptr.get(), nullptr);
}

/**
 * @brief Test ArenaImageVector construction with empty vector
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorEmptyConstruction)
{
  arena_camera::ArenaImageVector vec;
  
  EXPECT_TRUE(vec.empty());
  EXPECT_EQ(vec.size(), 0u);
}

/**
 * @brief Test ArenaImageVector construction from empty std::vector
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorFromEmptyStdVector)
{
  std::vector<Arena::IImage*> empty_vec;
  arena_camera::ArenaImageVector vec(std::move(empty_vec));
  
  EXPECT_TRUE(vec.empty());
  EXPECT_EQ(vec.size(), 0u);
}

/**
 * @brief Test ArenaImageVector size() method
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorSize)
{
  arena_camera::ArenaImageVector vec;
  
  EXPECT_EQ(vec.size(), 0u);
  
  // Add nullptr (safe to do, the deleter handles nullptr)
  vec.push_back(nullptr);
  EXPECT_EQ(vec.size(), 1u);
  
  vec.push_back(nullptr);
  EXPECT_EQ(vec.size(), 2u);
}

/**
 * @brief Test ArenaImageVector empty() method
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorEmpty)
{
  arena_camera::ArenaImageVector vec;
  
  EXPECT_TRUE(vec.empty());
  
  vec.push_back(nullptr);
  EXPECT_FALSE(vec.empty());
}

/**
 * @brief Test ArenaImageVector clear() method
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorClear)
{
  arena_camera::ArenaImageVector vec;
  vec.push_back(nullptr);
  vec.push_back(nullptr);
  
  EXPECT_EQ(vec.size(), 2u);
  
  vec.clear();
  
  EXPECT_TRUE(vec.empty());
  EXPECT_EQ(vec.size(), 0u);
}

/**
 * @brief Test ArenaImageVector operator[] access
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorIndexAccess)
{
  arena_camera::ArenaImageVector vec;
  vec.push_back(nullptr);
  
  EXPECT_EQ(vec[0], nullptr);
}

/**
 * @brief Test that move constructor properly transfers ownership
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorMoveConstructor)
{
  arena_camera::ArenaImageVector vec1;
  vec1.push_back(nullptr);
  vec1.push_back(nullptr);
  
  EXPECT_EQ(vec1.size(), 2u);
  
  // Move construct
  arena_camera::ArenaImageVector vec2(std::move(vec1));
  
  // vec2 should have the elements
  EXPECT_EQ(vec2.size(), 2u);
  
  // vec1 should be empty after move
  EXPECT_TRUE(vec1.empty());
}

/**
 * @brief Test that move assignment properly transfers ownership
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorMoveAssignment)
{
  arena_camera::ArenaImageVector vec1;
  vec1.push_back(nullptr);
  vec1.push_back(nullptr);
  
  arena_camera::ArenaImageVector vec2;
  vec2.push_back(nullptr);
  
  EXPECT_EQ(vec1.size(), 2u);
  EXPECT_EQ(vec2.size(), 1u);
  
  // Move assign
  vec2 = std::move(vec1);
  
  // vec2 should now have 2 elements
  EXPECT_EQ(vec2.size(), 2u);
  
  // vec1 should be empty after move
  EXPECT_TRUE(vec1.empty());
}

/**
 * @brief Test ArenaImageVector iterator access
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorIterator)
{
  arena_camera::ArenaImageVector vec;
  vec.push_back(nullptr);
  vec.push_back(nullptr);
  
  int count = 0;
  for (auto* img : vec) {
    EXPECT_EQ(img, nullptr);
    count++;
  }
  
  EXPECT_EQ(count, 2);
}

/**
 * @brief Test that a freshly default-constructed vector is in a valid state
 *
 * Note: self-move-assignment (vec = std::move(vec)) is undefined behavior for
 * std::vector per the C++ standard [lib.types.movedfrom] and was removed.
 * The move-assignment path is already covered by ArenaImageVectorMoveAssignment.
 */
TEST(ArenaImageRaiiTest, ArenaImageVectorDefaultStateIsValid)
{
  arena_camera::ArenaImageVector vec;
  EXPECT_TRUE(vec.empty());
  EXPECT_EQ(vec.size(), 0u);
  EXPECT_NO_THROW(vec.clear());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
