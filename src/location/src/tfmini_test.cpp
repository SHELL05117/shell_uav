/***************************************************************************************************************************
 * TFmini 台架测试（不启动 ROS 节点）。
 *
 *   tfmini_test i2c [bus] [address]        默认 bus=6, address=0x10
 *   tfmini_test uart [port] [baud]         默认 /dev/ttyUSB0, 115200
 *
 * 换平台后原来的 /dev/ttyTHS0（Jetson UART）和写死的 i2c 地址 0 都不对，这里一并修掉。
 ***************************************************************************************************************************/
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include <TFplus.h>
#include <tf_mini_i2c.h>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "usage: " << argv[0] << " i2c [bus] [address] | uart [port] [baud]" << std::endl;
    return 1;
  }

  benewake::TFplus *tf_uart = nullptr;
  TFMiniPro *tf_i2c = nullptr;

  if (std::strcmp(argv[1], "i2c") == 0)
  {
    const int bus = argc > 2 ? std::atoi(argv[2]) : 6;
    const int address = argc > 3 ? std::atoi(argv[3]) : 0x10;
    try
    {
      tf_i2c = new TFMiniPro(static_cast<uint8>(address), static_cast<uint8>(bus));
    }
    catch (const std::exception &error)
    {
      std::cout << "cannot open TFmini on i2c-" << bus << " address 0x" << std::hex << address
                << ": " << error.what() << std::endl;
      return 1;
    }
    std::cout << "TFmini on /dev/i2c-" << std::dec << bus << ", address 0x" << std::hex << address
              << std::dec << std::endl;
  }
  else if (std::strcmp(argv[1], "uart") == 0)
  {
    const std::string port = argc > 2 ? argv[2] : "/dev/ttyUSB0";
    const int baud = argc > 3 ? std::atoi(argv[3]) : 115200;
    tf_uart = new benewake::TFplus(port, baud);
    if (!tf_uart->isOpen())
    {
      std::cout << "cannot open TFmini on " << port << " @ " << baud << std::endl;
      return 1;
    }
    std::cout << "TFmini on " << port << " @ " << baud << std::endl;
  }
  else
  {
    std::cout << "unknown interface '" << argv[1] << "'" << std::endl;
    return 1;
  }

  while (true)
  {
    std::cout << "dist: " << (tf_i2c != nullptr ? tf_i2c->ReadData() : tf_uart->getDist()) << " m"
              << std::endl;
    usleep(100000);
  }

  delete tf_i2c;
  delete tf_uart;
  return 0;
}
