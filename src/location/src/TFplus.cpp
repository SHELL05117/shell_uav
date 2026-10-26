#include <TFplus.h>
#include <localization_guard.h>
#include <chrono>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace benewake
{
TFplus::TFplus(std::string name, int baud) : portName_(name), baudRate_(baud)
{
  serial_ = open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_ < 0) throw std::system_error(errno, std::generic_category(), "TFmini UART open");
  try
  {
    struct termios options;
    if (tcgetattr(serial_, &options) != 0)
      throw std::system_error(errno, std::generic_category(), "TFmini UART termios");
    speed_t speed;
    switch (baud)
    {
      case 921600: speed=B921600; break;
      case 576000: speed=B576000; break;
      case 500000: speed=B500000; break;
      case 460800: speed=B460800; break;
      case 230400: speed=B230400; break;
      case 115200: speed=B115200; break;
      case 57600: speed=B57600; break;
      case 38400: speed=B38400; break;
      case 19200: speed=B19200; break;
      case 9600: speed=B9600; break;
      case 4800: speed=B4800; break;
      default: throw std::invalid_argument("Unsupported TFmini UART baud rate");
    }
    cfmakeraw(&options);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CLOCAL | CREAD | CS8;
    options.c_cc[VMIN]=0; options.c_cc[VTIME]=0;
    cfsetispeed(&options,speed); cfsetospeed(&options,speed);
    if (tcsetattr(serial_, TCSANOW, &options) != 0)
      throw std::system_error(errno, std::generic_category(), "TFmini UART configure");
    tcflush(serial_,TCIFLUSH);
    is_open_=true;
  }
  catch (...) { closePort(); throw; }
}
TFplus::~TFplus() { closePort(); }

float TFplus::getDist()
{
  if (!is_open_) return -1.0f;
  // No cached serial byte can become a newly timed measurement. Discard the
  // entire pre-call backlog, then require a complete frame received after this
  // boundary. Hardware-read timestamps still do not claim a sensor-side clock.
  if (tcflush(serial_,TCIFLUSH)!=0) return -1.0f;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
  std::vector<unsigned char> pending;
  pending.reserve(1024);
  float latest=-1.0f;
  while (std::chrono::steady_clock::now()<deadline)
  {
    const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline-std::chrono::steady_clock::now()).count();
    struct pollfd waiter{serial_,POLLIN,0};
    const int ready=poll(&waiter,1,latest>=0.0f ? 2 : static_cast<int>(std::max<long long>(1,left)));
    if (ready<0 && errno==EINTR) continue;
    if (ready<0 || (waiter.revents & (POLLERR|POLLHUP|POLLNVAL))) return -1.0f;
    if (ready==0) return latest;
    unsigned char chunk[512];
    const ssize_t got=read(serial_,chunk,sizeof(chunk));
    if (got<0 && (errno==EAGAIN || errno==EINTR)) continue;
    if (got<=0) return -1.0f;
    pending.insert(pending.end(),chunk,chunk+got);
    while (pending.size()>=9)
    {
      if (!location_guard::validSerialFrame(pending.data(),9))
      {
        pending.erase(pending.begin());
        continue;
      }
      const unsigned distance=pending[2] | (static_cast<unsigned>(pending[3])<<8);
      if (distance>0)
      {
        latest=static_cast<float>(distance)/100.0f;
        std::copy(pending.begin()+2,pending.begin()+9,dataBuf);
      }
      pending.erase(pending.begin(),pending.begin()+9);
    }
    // Drain all currently available bytes before returning the newest complete
    // frame, including bursts larger than one read(). Partial frames are never
    // retained across calls, and silence on the next call returns failure.
  }
  return latest;
}
void TFplus::closePort()
{
  if (serial_>=0) close(serial_);
  serial_=-1; is_open_=false;
}
bool TFplus::isOpen() { return is_open_; }
}
