#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "dwa_control_math.h"
#include "dwa_mission_policy.h"

TEST(DwaControlMath, BodyForwardRotatesWithActualYaw)
{
  dwa_control::Vec2 local;
  ASSERT_TRUE(dwa_control::bodyVelocityToLocal({0.3, 0.0}, M_PI/2.0, 0.35, local));
  EXPECT_NEAR(local.x, 0.0, 1e-12);
  EXPECT_NEAR(local.y, 0.3, 1e-12);
}

TEST(DwaControlMath, DiagonalSpeedUsesVectorBound)
{
  dwa_control::Vec2 local;
  ASSERT_TRUE(dwa_control::bodyVelocityToLocal({0.35, 0.35}, 0.0, 0.35, local));
  EXPECT_NEAR(std::hypot(local.x,local.y), 0.35, 1e-12);
  EXPECT_NEAR(local.x,local.y,1e-12);
}

TEST(DwaControlMath, InvalidVelocityProducesNoCommand)
{
  dwa_control::Vec2 local(1.0,2.0);
  EXPECT_FALSE(dwa_control::bodyVelocityToLocal(
      {std::numeric_limits<double>::quiet_NaN(),0.0},0.0,0.35,local));
  EXPECT_DOUBLE_EQ(local.x,0.0);
  EXPECT_DOUBLE_EQ(local.y,0.0);
  EXPECT_FALSE(dwa_control::bodyVelocityToLocal({0.1,0.1},0.0,-1.0,local));
  const double huge = std::numeric_limits<double>::max();
  EXPECT_FALSE(dwa_control::bodyVelocityToLocal({huge,huge},0.0,0.35,local));
}

TEST(DwaControlMath, MapPointUsesRotationAndOriginTranslation)
{
  dwa_control::Vec2 local;
  ASSERT_TRUE(dwa_control::mapPointToLocal({3.0,3.0},{2.0,3.0,0.0},
                                          {10.0,20.0,M_PI/2.0},local));
  EXPECT_NEAR(local.x,10.0,1e-12);
  EXPECT_NEAR(local.y,21.0,1e-12);
}

TEST(DwaControlMath, InvalidMapObservationIsRejected)
{
  dwa_control::Vec2 local;
  EXPECT_FALSE(dwa_control::mapPointToLocal({1.0,2.0},
      {0.0,0.0,std::numeric_limits<double>::infinity()},{},local));
}

TEST(DwaControlMath, QuaternionMustBeValidAndCanBeNormalized)
{
  double yaw;
  EXPECT_FALSE(dwa_control::yawOf(0.0,0.0,0.0,0.0,yaw));
  ASSERT_TRUE(dwa_control::yawOf(0.0,0.0,2.0,2.0,yaw));
  EXPECT_NEAR(yaw,M_PI/2.0,1e-12);
}

TEST(DwaControlMath, SamplesCannotBeMissingStaleOrFarFuture)
{
  EXPECT_TRUE(dwa_control::fresh(10.0,9.8,0.5));
  EXPECT_FALSE(dwa_control::fresh(10.0,9.0,0.5));
  EXPECT_FALSE(dwa_control::fresh(10.0,0.0,0.5));
  EXPECT_FALSE(dwa_control::fresh(10.0,11.0,0.5));
}

TEST(DwaMissionPolicy, FireDropRequiresTheConfirmedRequestedHazard)
{
  using dwa_mission::DropKind;
  EXPECT_EQ(DropKind::Fire,dwa_mission::confirmedDrop(true,"flammable","flammable",true,true));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(true,"flammable","toxic",true,true));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(true,"flammable","person",true,true));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(true,"flammable","",true,true));
}

TEST(DwaMissionPolicy, MissingTargetCannotAuthorizeButExplicitAnyStillNeedsAHazard)
{
  using dwa_mission::DropKind;
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(true,"","toxic",false,true));
  EXPECT_EQ(DropKind::Fire,dwa_mission::confirmedDrop(true,"","toxic",false,false));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(true,"","person",false,false));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(true,"","",false,false));
}

TEST(DwaMissionPolicy, RescueDropRequiresPersonAndEnabledRescueStage)
{
  using dwa_mission::DropKind;
  EXPECT_EQ(DropKind::Rescue,dwa_mission::confirmedDrop(false,"","person",true,true));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(false,"","person",false,true));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(false,"","explosive",true,true));
  EXPECT_EQ(DropKind::None,dwa_mission::confirmedDrop(false,"","",true,true));
}

TEST(DwaMissionPolicy, FlightAuthorityRequiresArmedOffboard)
{
  EXPECT_TRUE(dwa_mission::offboardAuthority(true,"OFFBOARD"));
  EXPECT_FALSE(dwa_mission::offboardAuthority(false,"OFFBOARD"));
  EXPECT_FALSE(dwa_mission::offboardAuthority(true,"POSCTL"));
  EXPECT_FALSE(dwa_mission::offboardAuthority(true,"AUTO.LAND"));
}

TEST(DwaMissionPolicy, TakeoffTimeoutCannotAuthorizeHorizontalMission)
{
  using dwa_mission::TakeoffProgress;
  EXPECT_EQ(TakeoffProgress::Waiting,dwa_mission::takeoffProgress(1.0,true,20.0));
  EXPECT_EQ(TakeoffProgress::Reached,dwa_mission::takeoffProgress(2.1,true,20.0));
  EXPECT_EQ(TakeoffProgress::Waiting,dwa_mission::takeoffProgress(19.9,false,20.0));
  EXPECT_EQ(TakeoffProgress::TimedOut,dwa_mission::takeoffProgress(20.0,false,20.0));
  EXPECT_EQ(TakeoffProgress::TimedOut,dwa_mission::takeoffProgress(1.0,false,0.8));
}

TEST(DwaMissionPolicy, LandingOnlyAllowsRequestedTransitionsAndFinalDisarm)
{
  EXPECT_TRUE(dwa_mission::landingAuthority(true,"OFFBOARD",0));
  EXPECT_FALSE(dwa_mission::landingAuthority(true,"POSCTL",0));
  EXPECT_FALSE(dwa_mission::landingAuthority(false,"OFFBOARD",0));
  EXPECT_TRUE(dwa_mission::landingAuthority(true,"OFFBOARD",1));
  EXPECT_TRUE(dwa_mission::landingAuthority(true,"POSCTL",1));
  EXPECT_FALSE(dwa_mission::landingAuthority(true,"AUTO.LAND",1));
  EXPECT_TRUE(dwa_mission::landingAuthority(true,"POSCTL",2));
  EXPECT_TRUE(dwa_mission::landingAuthority(true,"AUTO.LAND",2));
  EXPECT_TRUE(dwa_mission::landingAuthority(false,"AUTO.LAND",2));
  EXPECT_FALSE(dwa_mission::landingAuthority(true,"OFFBOARD",2));
  for (int stage = 0; stage < 3; ++stage)
  {
    EXPECT_FALSE(dwa_mission::landingAuthority(true,"MANUAL",stage));
    EXPECT_FALSE(dwa_mission::landingAuthority(false,"MANUAL",stage));
  }
}
