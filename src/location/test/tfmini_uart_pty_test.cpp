#include <gtest/gtest.h>
#include <TFplus.h>
#include <pty.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

namespace
{
// Protocol fixtures cross a real POSIX tty. They do not call the production
// parser or fabricate its internal buffers.
std::vector<unsigned char> frame(unsigned centimetres)
{
  std::vector<unsigned char> bytes{0x59,0x59,
      static_cast<unsigned char>(centimetres),static_cast<unsigned char>(centimetres>>8),
      30,0,0,8,0};
  unsigned sum=0;
  for (std::size_t i=0;i<8;++i) sum+=bytes[i];
  bytes[8]=static_cast<unsigned char>(sum);
  return bytes;
}

class TfminiUartPty : public testing::Test
{
protected:
  void SetUp() override
  {
    int slave=-1; char path[128]={0};
    ASSERT_EQ(0,openpty(&master_,&slave,path,nullptr,nullptr));
    sensor_.reset(new benewake::TFplus(path,115200));
    close(slave);
  }
  void TearDown() override
  {
    sensor_.reset();
    if (master_>=0) close(master_);
  }
  void send(const std::vector<unsigned char> &bytes)
  {
    std::size_t offset=0;
    while (offset<bytes.size())
    {
      const ssize_t count=write(master_,bytes.data()+offset,bytes.size()-offset);
      if (count<0 && errno==EINTR) continue;
      ASSERT_GT(count,0);
      offset+=static_cast<std::size_t>(count);
    }
  }
  std::thread later(std::vector<unsigned char> bytes)
  {
    return std::thread([this,bytes]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      send(bytes);
    });
  }
  int master_=-1;
  std::unique_ptr<benewake::TFplus> sensor_;
};

TEST_F(TfminiUartPty, PreCallBacklogWithoutANewSampleIsNotFresh)
{
  send(frame(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto started=std::chrono::steady_clock::now();
  EXPECT_LT(sensor_->getDist(),0.0f);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count(),0.65);
}

TEST_F(TfminiUartPty, NewFrameAfterBoundaryReplacesOldQueuedDistance)
{
  send(frame(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto writer=later(frame(321));
  const float distance=sensor_->getDist();
  writer.join();
  EXPECT_NEAR(3.21f,distance,1e-5f);
}

TEST_F(TfminiUartPty, DrainsABurstLargerThanOneReadToItsLatestCompleteFrame)
{
  std::vector<unsigned char> burst;
  for (unsigned value=101;value<=400;value+=1)
  {
    const auto bytes=frame(value);
    burst.insert(burst.end(),bytes.begin(),bytes.end());
  }
  // A partial newer frame must not invalidate the latest complete one.
  burst.push_back(0x59); burst.push_back(0x59); burst.push_back(1);
  auto writer=later(burst);
  const float distance=sensor_->getDist();
  writer.join();
  EXPECT_FLOAT_EQ(4.0f,distance);
}

TEST_F(TfminiUartPty, ResynchronizesNoiseCorruptChecksumAndFragmentedFrame)
{
  auto broken=frame(999); broken[8]^=1;
  const auto valid=frame(123);
  std::thread writer([this,broken,valid]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    send({0,0xff,0x59,1});
    send(broken);
    send(std::vector<unsigned char>(valid.begin(),valid.begin()+4));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    send(std::vector<unsigned char>(valid.begin()+4,valid.end()));
  });
  const float distance=sensor_->getDist();
  writer.join();
  EXPECT_NEAR(1.23f,distance,1e-5f);
}

TEST_F(TfminiUartPty, AConsumedFrameCannotRefreshAgainAfterTheStreamStops)
{
  auto writer=later(frame(250));
  const float first=sensor_->getDist();
  writer.join();
  EXPECT_FLOAT_EQ(2.5f,first);
  EXPECT_LT(sensor_->getDist(),0.0f);
  EXPECT_LT(sensor_->getDist(),0.0f);
}
}  // namespace

int main(int argc,char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
