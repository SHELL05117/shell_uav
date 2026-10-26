#include <gtest/gtest.h>
#include <localization_guard.h>
#include <limits>

using namespace location_guard;

TEST(Freshness, RejectsMissingOldFutureAndNan)
{
  EXPECT_TRUE(fresh(10.0,10.1,.2));
  EXPECT_FALSE(fresh(0,10.1,.2));
  EXPECT_FALSE(fresh(10,10.3,.2));
  EXPECT_FALSE(fresh(11,10.1,.2));
  EXPECT_FALSE(fresh(std::numeric_limits<double>::quiet_NaN(),10.1,.2));
}
TEST(Range, RejectsTinyNanAndOutOfRangeWithoutRefreshing)
{
  RangeGate gate;
  EXPECT_TRUE(gate.observe(.59,10));
  EXPECT_FALSE(gate.observe(.002,10.1));
  EXPECT_FALSE(gate.observe(std::numeric_limits<double>::quiet_NaN(),10.2));
  EXPECT_FALSE(gate.observe(7,10.3));
  EXPECT_EQ(10,gate.stamp());
  EXPECT_DOUBLE_EQ(.59,gate.value());
  EXPECT_FALSE(fresh(gate.stamp(),10.5,.3));
}
TEST(Range, RepeatedCachedSamplesDoNotConfirmJump)
{
  RangeGate gate; gate.confirmations_required=3;
  ASSERT_TRUE(gate.observe(.59,10));
  ASSERT_FALSE(gate.observe(2,10.1));
  for (int i=0;i<50;++i) EXPECT_FALSE(gate.observe(2,10.1));
  EXPECT_DOUBLE_EQ(.59,gate.value());
  EXPECT_FALSE(gate.observe(2.01,10.2));
  EXPECT_TRUE(gate.observe(2.02,10.3));
}
TEST(Range, InconsistentJumpSamplesDoNotRebaseline)
{
  RangeGate gate; gate.confirmations_required=3;
  ASSERT_TRUE(gate.observe(.59,10));
  EXPECT_FALSE(gate.observe(2,10.1));
  EXPECT_FALSE(gate.observe(3,10.2));
  EXPECT_FALSE(gate.observe(2,10.3));
  EXPECT_FALSE(gate.observe(3,10.4));
}
TEST(Position, JumpRejectionDoesNotAcceptDuplicateStamp)
{
  PositionGate gate; gate.confirmations_required=3;
  EXPECT_TRUE(gate.observe(0,0,10));
  EXPECT_TRUE(gate.observe(0,0,10));
  EXPECT_FALSE(gate.observe(2,0,10.1));
  for(int i=0;i<50;++i) EXPECT_FALSE(gate.observe(2,0,10.1));
  EXPECT_FALSE(gate.observe(2,0,10.2));
  EXPECT_TRUE(gate.observe(2,0,10.3));
}
TEST(Height, SubtractsMeasuredLevelGroundClearance)
{
  double z=0;
  EXPECT_TRUE(projectHeight(.59,{0,0,0,1},{0,0,-1},{0,0,0},.09,35,-.2,3,z));
  EXPECT_NEAR(.5,z,1e-12);
}
TEST(Height, ProjectsSlantRangeUsingImuPitch)
{
  const double angle=20*3.14159265358979323846/180;
  double z=0;
  EXPECT_TRUE(projectHeight(.59/std::cos(angle),{0,std::sin(angle/2),0,std::cos(angle/2)},
                           {0,0,-1},{0,0,0},.09,35,-.2,3,z));
  EXPECT_NEAR(.5,z,1e-12);
}
TEST(Height, AppliesRotatedSensorOffset)
{
  const double angle=20*3.14159265358979323846/180;
  double z=0;
  EXPECT_TRUE(projectHeight(.59/std::cos(angle),{0,std::sin(angle/2),0,std::cos(angle/2)},
                           {0,0,-1},{.2,0,0},.09,35,-.2,3,z));
  EXPECT_NEAR(.5+.2*std::sin(angle),z,1e-12);
}
TEST(Height, RejectsTiltInvalidQuaternionAndOutsideHeight)
{
  double z=0;
  EXPECT_FALSE(projectHeight(1,{0,.5,0,std::sqrt(.75)},{0,0,-1},{0,0,0},.09,35,-.2,3,z));
  EXPECT_FALSE(projectHeight(1,{0,0,0,0},{0,0,-1},{0,0,0},.09,35,-.2,3,z));
  EXPECT_FALSE(projectHeight(5,{0,0,0,1},{0,0,-1},{0,0,0},.09,35,-.2,3,z));
}
TEST(Height, RejectsInvalidGeometryParameters)
{
  double z=0;
  EXPECT_FALSE(projectHeight(1,{0,0,0,1},{0,0,0},{0,0,0},.09,35,-.2,3,z));
  EXPECT_FALSE(projectHeight(1,{0,0,0,1},{0,0,-1},{0,0,0},.09,
                            std::numeric_limits<double>::quiet_NaN(),-.2,3,z));
}
TEST(UartFrame, ValidatesHeaderChecksumAndLength)
{
  unsigned char frame[9]={0x59,0x59,100,0,20,0,0,0,0};
  unsigned sum=0; for (int i=0;i<8;++i) sum+=frame[i]; frame[8]=sum;
  EXPECT_TRUE(validSerialFrame(frame,9));
  EXPECT_FALSE(validSerialFrame(frame,8));
  frame[2]^=1;
  EXPECT_FALSE(validSerialFrame(frame,9));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
