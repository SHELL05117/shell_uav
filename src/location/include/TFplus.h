#ifndef LOCATION_TFPLUS_H
#define LOCATION_TFPLUS_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>

namespace benewake
{
  class TFplus
  {
    public:
      TFplus(std::string _name, int _baudRate);
      ~TFplus();
      float getDist();
      void closePort();
      bool isOpen();

      unsigned char dataBuf[7];

    private:
      std::string portName_;
      int baudRate_;
      int serial_ = -1;
      bool is_open_ = false;

  };
}
#endif
