// Tests for NvjpgEncoder (NVJPG hardware JPEG plugin).
// Skipped when the plugin isn't built (non-Jetson hosts) or no NVJPG engine.

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "nvjpg_encoder.h"

namespace {

#ifdef NVJPG_PLUGIN_PATH
const char* kPluginPath = NVJPG_PLUGIN_PATH;
#else
const char* kPluginPath = nullptr;
#endif

cv::Mat gradient_mono(int w, int h) {
  cv::Mat m(h, w, CV_8UC1);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) m.at<uint8_t>(y, x) = static_cast<uint8_t>((x + 2 * y) & 0xFF);
  return m;
}

cv::Mat gradient_bgr(int w, int h) {
  cv::Mat mono = gradient_mono(w, h);
  cv::Mat bgr;
  cv::merge(std::vector<cv::Mat>{mono, 255 - mono, mono / 2}, bgr);
  return bgr;
}

class NvjpgEncoderTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!kPluginPath) GTEST_SKIP() << "NVJPG plugin not built on this host";
    enc_ = std::make_unique<NvjpgEncoder>(80, kPluginPath);
    if (!enc_->is_valid()) GTEST_SKIP() << "NVJPG unavailable: " << enc_->error();
  }
  std::unique_ptr<NvjpgEncoder> enc_;
};

}  // namespace

TEST(NvjpgEncoderLoad, MissingPluginIsInvalidNotFatal) {
  NvjpgEncoder enc(80, "/nonexistent/libarena_nvjpg.so");
  EXPECT_FALSE(enc.is_valid());
  EXPECT_FALSE(enc.error().empty());
  std::vector<uint8_t> out;
  EXPECT_FALSE(enc.encode(gradient_mono(64, 64), out));
}

TEST_F(NvjpgEncoderTest, EncodesMonoAsDecodableJpeg) {
  std::vector<uint8_t> out;
  ASSERT_TRUE(enc_->encode(gradient_mono(1224, 1024), out)) << enc_->error();
  ASSERT_GT(out.size(), 4u);
  EXPECT_EQ(out[0], 0xFF);
  EXPECT_EQ(out[1], 0xD8);
  EXPECT_EQ(out[out.size() - 2], 0xFF);
  EXPECT_EQ(out[out.size() - 1], 0xD9);
  cv::Mat dec = cv::imdecode(out, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(dec.empty());
  EXPECT_EQ(dec.cols, 1224);
  EXPECT_EQ(dec.rows, 1024);
}

TEST_F(NvjpgEncoderTest, EncodesBgrAndPreservesContent) {
  const cv::Mat src = gradient_bgr(640, 480);
  std::vector<uint8_t> out;
  ASSERT_TRUE(enc_->encode(src, out)) << enc_->error();
  cv::Mat dec = cv::imdecode(out, cv::IMREAD_COLOR);
  ASSERT_EQ(dec.size(), src.size());

  // A colour-range or matrix mismatch (e.g. limited vs full-range YCbCr)
  // shows up as a per-channel bias, independent of compression loss.
  const cv::Scalar src_mean = cv::mean(src);
  const cv::Scalar dec_mean = cv::mean(dec);
  for (int c = 0; c < 3; ++c) {
    EXPECT_NEAR(dec_mean[c], src_mean[c], 2.0) << "channel " << c;
  }

  // Compression loss should be comparable to libjpeg at the same quality.
  std::vector<uint8_t> ref;
  ASSERT_TRUE(cv::imencode(".jpg", src, ref, {cv::IMWRITE_JPEG_QUALITY, 80}));
  const cv::Mat ref_dec = cv::imdecode(ref, cv::IMREAD_COLOR);
  const double denom = static_cast<double>(src.total() * 3);
  const double nvjpg_err = cv::norm(dec, src, cv::NORM_L1) / denom;
  const double ref_err = cv::norm(ref_dec, src, cv::NORM_L1) / denom;
  EXPECT_LT(nvjpg_err, ref_err * 1.5) << "nvjpg " << nvjpg_err << " vs libjpeg " << ref_err;
}

TEST_F(NvjpgEncoderTest, HandlesAlternatingShapes) {
  std::vector<uint8_t> out;
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(enc_->encode(gradient_mono(1224, 1024), out)) << enc_->error();
    ASSERT_TRUE(enc_->encode(gradient_bgr(1224, 1024), out)) << enc_->error();
    ASSERT_TRUE(enc_->encode(gradient_mono(320, 240), out)) << enc_->error();
  }
}

TEST_F(NvjpgEncoderTest, RejectsOddDimensions) {
  std::vector<uint8_t> out;
  EXPECT_FALSE(enc_->encode(gradient_mono(63, 64), out));
}

TEST_F(NvjpgEncoderTest, OpenCvJpegStillWorksAfterPluginLoad) {
  // If L4T's libnvjpeg leaked into the global symbol scope, OpenCV's
  // libjpeg.so.8 calls could bind to it and fail with a struct mismatch.
  std::vector<uint8_t> cv_out;
  ASSERT_TRUE(cv::imencode(".jpg", gradient_bgr(320, 240), cv_out));
  cv::Mat dec = cv::imdecode(cv_out, cv::IMREAD_COLOR);
  ASSERT_FALSE(dec.empty());
  EXPECT_EQ(dec.cols, 320);
}
