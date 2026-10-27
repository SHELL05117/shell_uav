#ifndef UAV_AUDIT_JSON_H
#define UAV_AUDIT_JSON_H
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
namespace uav_audit {
inline std::string quote(const std::string &value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c=='"' || c=='\\') out << '\\' << c;
    else if (c<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else out << c;
  }
  out << '"'; return out.str();
}
inline std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out; out << std::setprecision(16) << value; return out.str();
}
}
#endif
