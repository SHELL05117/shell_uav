#include <gtest/gtest.h>
#include "yolo_tensor_layout.h"
#include "latest_result_cache.h"
#include "detection_freshness.h"

using navigation_yolo::Head;

TEST(DetectionFreshness, RejectsMissingFutureAndOldHeaders)
{
  using navigation_yolo::freshDetectionSource;
  EXPECT_FALSE(freshDetectionSource(0.0,100.0,0.0,0.0,1.0));
  EXPECT_FALSE(freshDetectionSource(101.0,100.0,-1.0,0.0,1.0));
  EXPECT_FALSE(freshDetectionSource(98.9,100.0,1.1,0.0,1.0));
  EXPECT_TRUE(freshDetectionSource(99.5,100.0,.5,0.0,1.0));
}

TEST(DetectionFreshness, CompletionDoesNotRenewOldSource)
{
  using navigation_yolo::freshDetectionSource;
  EXPECT_TRUE(freshDetectionSource(99.25,100.0,.75,0.0,1.0));
  // Actual inference may be 0.66 s; use the result limit (normally 3 s),
  // rather than the image callback limit (normally 1 s).
  EXPECT_TRUE(freshDetectionSource(99.25,100.66,.75,.66,3.0));
  EXPECT_FALSE(freshDetectionSource(99.25,100.66,.75,.66,1.1));
}

TEST(DetectionFreshness, MonotonicElapsedSurvivesRosClockPauseAndRejectsInvalidTime)
{
  using navigation_yolo::freshDetectionSource;
  EXPECT_FALSE(freshDetectionSource(99.9,100.0,.1,3.0,3.0));
  EXPECT_FALSE(freshDetectionSource(99.9,99.8,.1,.5,3.0));
  EXPECT_TRUE(freshDetectionSource(99.5,100.5,.5,.5,1.0));
  EXPECT_FALSE(freshDetectionSource(99.5,100.5,.5,-.1,1.0));
  EXPECT_FALSE(freshDetectionSource(99.5,100.5,.5,.5,std::numeric_limits<double>::quiet_NaN()));
}

TEST(YoloLayout, FiveDimensionalHeadsPreserveAnchorCellAndField)
{
  const int height = 2, width = 3, fields = 9;
  std::vector<float> raw(3 * height * width * fields);
  for (int a = 0; a < 3; ++a)
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        for (int k = 0; k < fields; ++k)
          raw[((a * height + y) * width + x) * fields + k] =
              static_cast<float>(1000 * a + 100 * k + 10 * y + x);
  Head head;
  std::string error;
  ASSERT_TRUE(navigation_yolo::normalizeHead({1,3,2,3,9}, 27, raw.data(), raw.size(), head, error)) << error;
  EXPECT_EQ(27, head.channels);
  EXPECT_EQ(2, head.height);
  EXPECT_EQ(3, head.width);
  for (int a = 0; a < 3; ++a)
    for (int k = 0; k < fields; ++k)
      for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
          EXPECT_FLOAT_EQ(static_cast<float>(1000 * a + 100 * k + 10 * y + x),
                          head.data[((a * fields + k) * height + y) * width + x]);
}

TEST(YoloLayout, LegacyRknnDimensionOrderMatchesLogicalOnnxHead)
{
  const uint32_t dims[] = {9,52,52,3,1};
  const auto logical = navigation_yolo::reversedDimensions(dims, 5);
  EXPECT_EQ((std::vector<std::size_t>{1,3,52,52,9}), logical);
  Head head;
  std::string error;
  ASSERT_TRUE(navigation_yolo::describeHead(logical, 27, 73008, head, error)) << error;
  EXPECT_EQ(52, head.width);
  EXPECT_EQ(52, head.height);
}

TEST(YoloLayout, FourDimensionalChwHeadsAreUnchanged)
{
  std::vector<float> raw(27*2*2);
  for (std::size_t i = 0; i < raw.size(); ++i) raw[i] = static_cast<float>(i);
  Head head;
  std::string error;
  ASSERT_TRUE(navigation_yolo::normalizeHead({1,27,2,2}, 27, raw.data(), raw.size(), head, error));
  EXPECT_EQ(raw, head.data);
}

TEST(YoloLayout, BadBatchClassesOrBufferCountAreRejected)
{
  Head head;
  std::string error;
  EXPECT_FALSE(navigation_yolo::describeHead({2,3,52,52,9},27,146016,head,error));
  EXPECT_FALSE(navigation_yolo::describeHead({1,3,52,52,11},27,89232,head,error));
  EXPECT_FALSE(navigation_yolo::describeHead({1,3,52,52,9},27,73007,head,error));
  EXPECT_FALSE(navigation_yolo::describeHead({1,27,0,52},27,0,head,error));
}

TEST(YoloLayout, CorruptOrMissingOutputIsRejected)
{
  std::vector<float> raw(27, 0.0f);
  Head head;
  std::string error;
  EXPECT_FALSE(navigation_yolo::normalizeHead({1,27,1,1},27,nullptr,27,head,error));
  raw[4] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(navigation_yolo::normalizeHead({1,27,1,1},27,raw.data(),raw.size(),head,error));
}

TEST(YoloDecode, UsesHalfCellOffsetAndPixelAnchors)
{
  const auto box = navigation_yolo::decodeV5Box(0,0,0,0,2,3,8,10,13);
  EXPECT_FLOAT_EQ(20.0f, box.center_x);
  EXPECT_FLOAT_EQ(28.0f, box.center_y);
  EXPECT_FLOAT_EQ(10.0f, box.width);
  EXPECT_FLOAT_EQ(13.0f, box.height);
}

TEST(YoloDecode, StableSigmoidAndNonzeroBoxLogits)
{
  const float value = std::log(3.0f);
  const auto box = navigation_yolo::decodeV5Box(value,0,0,value,2,3,8,10,13);
  EXPECT_NEAR(24.0f, box.center_x, 1e-5f);
  EXPECT_NEAR(29.25f, box.height, 1e-5f);
  EXPECT_FLOAT_EQ(0.0f, navigation_yolo::sigmoid(-1000.0f));
  EXPECT_FLOAT_EQ(1.0f, navigation_yolo::sigmoid(1000.0f));
}

TEST(YoloAsyncResults, EmptyDetectionIsConsumedOncePerSequence)
{
  navigation_yolo::LatestResultCache<std::vector<int> > cache;
  std::vector<int> result;
  uint64_t sequence = 0;
  EXPECT_FALSE(cache.poll(result, sequence));
  cache.publish({});
  ASSERT_TRUE(cache.poll(result, sequence));
  EXPECT_EQ(1u, sequence);
  EXPECT_TRUE(result.empty());
  EXPECT_FALSE(cache.poll(result, sequence));
  cache.publish({3});
  ASSERT_TRUE(cache.poll(result, sequence));
  EXPECT_EQ(2u, sequence);
  EXPECT_EQ((std::vector<int>{3}), result);
}

TEST(YoloAsyncResults, ConsumersHaveIndependentSequenceCursors)
{
  navigation_yolo::LatestResultCache<std::vector<int> > cache;
  std::vector<int> a, b;
  uint64_t seq_a = 0, seq_b = 0;
  cache.publish({1});
  EXPECT_TRUE(cache.poll(a, seq_a));
  EXPECT_TRUE(cache.poll(b, seq_b));
  EXPECT_FALSE(cache.poll(a, seq_a));
  EXPECT_EQ(a, b);
}

TEST(YoloAsyncResults, RestartClearsCacheAndKeepsSequenceMonotonic)
{
  navigation_yolo::LatestResultCache<std::vector<int> > cache;
  std::vector<int> result;
  uint64_t sequence = 0;
  cache.publish({1});
  ASSERT_TRUE(cache.poll(result, sequence));
  cache.clear();
  EXPECT_FALSE(cache.poll(result, sequence));
  cache.publish({2});
  EXPECT_TRUE(cache.poll(result, sequence));
  EXPECT_EQ(2u, sequence);
}
