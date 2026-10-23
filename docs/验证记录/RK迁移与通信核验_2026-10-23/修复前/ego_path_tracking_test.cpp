#include <ego_path_tracking.h>

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <utility>

namespace
{
nav_msgs::Path makePath(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::Path path;
  for (const auto &point : points)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    path.poses.push_back(pose);
  }
  return path;
}
}

TEST(EgoPathTracking, LooksAheadAlongStraightPath)
{
  geometry_msgs::Point target;
  double remaining = 0.0;
  ASSERT_TRUE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}),
      0.5, 0.0, 0.4, 0.1, target, remaining));
  EXPECT_NEAR(target.x, 0.9, 1e-6);
  EXPECT_NEAR(target.y, 0.0, 1e-6);
  EXPECT_NEAR(remaining, 1.5, 1e-6);
}

TEST(EgoPathTracking, FollowsCurvedPathByArcLength)
{
  geometry_msgs::Point target;
  double remaining = 0.0;
  ASSERT_TRUE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}),
      0.8, 0.0, 0.5, 0.1, target, remaining));
  EXPECT_NEAR(target.x, 1.0, 1e-6);
  EXPECT_NEAR(target.y, 0.3, 1e-6);
}

TEST(EgoPathTracking, OvershootTargetsTheRemainingEndpoint)
{
  geometry_msgs::Point target;
  double remaining = 0.0;
  ASSERT_TRUE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}),
      2.2, 0.0, 0.4, 0.1, target, remaining));
  EXPECT_NEAR(target.x, 2.0, 1e-6);
  EXPECT_NEAR(remaining, 0.2, 1e-6);
}

TEST(EgoPathTracking, TracksObstacleDetourInsteadOfCuttingCorner)
{
  geometry_msgs::Point target;
  double remaining = 0.0;
  ASSERT_TRUE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}, {1.0, 0.5}}),
      0.4, 0.0, 0.3, 0.1, target, remaining));
  EXPECT_NEAR(target.x, 0.5, 1e-6);
  EXPECT_NEAR(target.y, 0.2, 1e-6);
}

TEST(EgoPathTracking, EmptyPathAndReachedGoalDoNotCommandMotion)
{
  geometry_msgs::Point target;
  double remaining = 0.0;
  EXPECT_FALSE(ego_path_tracking::selectLookaheadPoint(
      makePath({}), 0.0, 0.0, 0.4, 0.1, target, remaining));
  EXPECT_FALSE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}}),
      0.95, 0.0, 0.4, 0.1, target, remaining));
}

TEST(EgoPathTracking, CrossTrackIsPerpendicularOffsetNotDistanceToTarget)
{
  // The out-parameter added on 2026-09-25 exists so a sway can be measured from a
  // log. It must report how far the vehicle is OFF the path, not how far it is
  // from the pursuit target; the two differ by the lookahead and only the first
  // one turns into steer angle, atan(cross_track / lookahead).
  geometry_msgs::Point target;
  double remaining = 0.0;
  double cross_track = -1.0;
  ASSERT_TRUE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}),
      0.5, 0.05, 0.45, 0.1, target, remaining, &cross_track));
  EXPECT_NEAR(cross_track, 0.05, 1e-9);
  // Same lag, a quarter of the steering: this is the whole reason the lookahead
  // was raised from 0.20 m to 0.45 m.
  EXPECT_NEAR(std::atan2(cross_track, 0.45) * 180.0 / 3.14159265358979323846,
              6.34, 0.01);
  EXPECT_NEAR(std::atan2(cross_track, 0.20) * 180.0 / 3.14159265358979323846,
              14.04, 0.01);
}

TEST(EgoPathTracking, CrossTrackIsNanWhenTheCommandIsRejected)
{
  // Rejected inputs must not leave a stale value behind for the diagnostic line
  // to print as if it were a measurement.
  geometry_msgs::Point target;
  double remaining = 0.0;
  double cross_track = 42.0;
  EXPECT_FALSE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}}),
      0.95, 0.0, 0.4, 0.1, target, remaining, &cross_track));
  EXPECT_TRUE(std::isnan(cross_track));
}

TEST(EgoPathTracking, RejectsNonFinitePoseAndPathPoints)
{
  geometry_msgs::Point target;
  double remaining = 0.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {1.0, 0.0}}),
      nan, 0.0, 0.4, 0.1, target, remaining));
  EXPECT_FALSE(ego_path_tracking::selectLookaheadPoint(
      makePath({{0.0, 0.0}, {nan, 0.0}}),
      0.0, 0.0, 0.4, 0.1, target, remaining));
}
