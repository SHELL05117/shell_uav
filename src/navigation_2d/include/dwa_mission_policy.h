#ifndef NAVIGATION_2D_DWA_MISSION_POLICY_H
#define NAVIGATION_2D_DWA_MISSION_POLICY_H

#include <string>

namespace dwa_mission
{
enum class DropKind { None, Fire, Rescue };
enum class TakeoffProgress { Waiting, Reached, TimedOut };

inline bool offboardAuthority(bool armed, const std::string &mode)
{
  return armed && mode == "OFFBOARD";
}

// Only acknowledge the landing transitions that this node has requested.
// Old mode is allowed while the request is in flight; manual modes never are.
inline bool landingAuthority(bool armed, const std::string &mode, int land_stage)
{
  if (!armed)
    return land_stage >= 2 && (mode == "POSCTL" || mode == "AUTO.LAND");
  if (land_stage == 0)
    return mode == "OFFBOARD";
  if (land_stage == 1)
    return mode == "OFFBOARD" || mode == "POSCTL";
  return mode == "POSCTL" || mode == "AUTO.LAND";
}

inline TakeoffProgress takeoffProgress(double elapsed, bool at_height, double timeout)
{
  if (at_height && elapsed > 2.0)
    return TakeoffProgress::Reached;
  return elapsed >= timeout ? TakeoffProgress::TimedOut : TakeoffProgress::Waiting;
}

inline bool hazardLabel(const std::string &label)
{
  return label == "explosive" || label == "flammable" || label == "toxic";
}

// A timeout or route exhaustion supplies no confirmed label and must never
// authorize a drop. Optional target selection still requires a real hazard.
inline DropKind confirmedDrop(bool hazard_task, const std::string &target,
                              const std::string &confirmed_label,
                              bool rescue_enabled, bool require_target)
{
  if (hazard_task)
  {
    if (!hazardLabel(confirmed_label) ||
        (target.empty() ? require_target : target != confirmed_label))
      return DropKind::None;
    return DropKind::Fire;
  }
  return rescue_enabled && confirmed_label == "person" ? DropKind::Rescue : DropKind::None;
}
}  // namespace dwa_mission

#endif
