// C++ port of the pure ballistic-trajectory functions in
// hopcopter-ballistic-planning/hopping_astar_planner.py (Campana & Laumond
// 2016 BEAM + this robot's energy chain). Straight transcriptions; the Python
// docstrings are the authoritative derivations. The diagnostic `trace`
// machinery is deliberately not ported (the planner always passes None).
#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "ballistic_motion_planner/map2d5.hpp"

namespace ballistic
{

// Small angular slack keeping alpha strictly inside the open feasible interval.
constexpr double kAlphaEps = 1e-6;

using Interval = std::pair<double, double>;

// 3D Coulomb friction cone reduced to an in-plane wedge (gamma, delta).
// nullopt when the plane meets the cone only at its apex (no jump possible).
std::optional<Interval> inplane_friction_cone(const Normal & n, double theta, double mu);

// tan(alpha) interval satisfying v_s^2 <= W. nullopt if W <= 0 or D < 0.
std::optional<Interval> speed_tan_interval(double X, double Z, double W, double g);

// Landing friction cone mapped back through the arc to an alpha_s interval.
// NOTE the +2Z/X sign (a docs transcription has it negative — erratum).
std::optional<Interval> landing_cone_alpha_s(
  double X, double Z, double gamma_g, double delta_g);

// Lower bound on tan(alpha_s) so the CoM drops >= h_min from apex to landing.
// nullopt when degenerate (X ~ 0) or vacuous (h_min + Z < 0).
std::optional<double> min_apex_tan(double X, double Z, double h_min);

// tan(alpha_s) of the cheapest parabola reaching (X, Z); 45 deg on the flat.
double min_energy_tan(double X, double Z);

double xdot(double X, double Z, double alpha_s, double g);
double takeoff_speed(double X, double Z, double alpha_s, double g);
double landing_speed(double v_s, double Z, double g);
double injection_energy(double v_s, double v_s_min, double mass);

// Farthest flat-ground 45-degree hop the current energy state affords.
double max_hop_radius(
  double v_g_in, double eta, double e_inject_max, double mass, double g, double V_max);

// Feasible takeoff-angle interval [alpha_min, alpha_max], or nullopt.
// Gate order matches Python: E1/E2 energy band, E3 min-apex, (4) landing
// speed, (1) takeoff cone, (2) landing cone; interval shrunk by kAlphaEps.
std::optional<Interval> feasible_alpha_interval(
  double X, double Z, double V_max, double g,
  double mu,                       // friction coefficient (always used here)
  const Normal & n_s, const Normal & n_g,
  double theta,
  double v_s_min, double e_inject_max, double mass,
  double min_apex, double V_g_max);

struct ArcSamples
{
  std::vector<double> u;       // 0 .. X inclusive
  double X;
  std::vector<double> field;   // inflated field at each sample (nearest cell)
  std::vector<uint8_t> active; // endpoints forced false
  bool any_active;
};

// March the hop centreline and read the inflated field along it. Step is
// clamped to min(max_step, resolution / 3) — the /3 is a hard requirement
// (the T_req pole at the endpoints; see the Python docstring). nullopt when
// the centreline leaves the map or the hop is degenerate.
std::optional<ArcSamples> arc_samples(
  double x_s, double y_s, double t_s,
  double x_g, double y_g, double t_g,
  const Map2D5 & map, const std::vector<double> & inflated, double max_step);

// Shallowest takeoff angle whose arc clears the (inflated) terrain, in closed
// form. -inf when nothing constrains; nullopt when the centreline leaves the
// map. Caller clamps into [alpha_min, alpha_max].
std::optional<double> clearance_floor_alpha(
  double x_s, double y_s, double t_s,
  double x_g, double y_g, double t_g,
  const Map2D5 & map, const std::vector<double> & inflated,
  double gate, double max_step);

// ---------------------------------------------------------------------------
// The hop the robot flies (stage 3 of docs/follow_planner_plan.md). Not in the
// reference planner; the search uses these only when momentum_aware is set.
// ---------------------------------------------------------------------------

struct Vec3
{
  double x, y, z;
};

// Vertical-push model. The robot pushes level after liftoff, so the leg alone
// supplies the horizontal speed: it launches at alpha_leg with
// v_leg = sqrt(eta) v_g_in, and the push acts as an instant vertical kick. The
// arc, apex, landing angle and injection come out as in feasible_alpha_interval
// (the same parabola, and e = m/2 (v_s^2 - v_leg^2) is the same number), so
// gates E1/E2/E3/(4)/(2) are unchanged. Only the takeoff cone (1) moves from the
// arc angle to alpha_leg = acos(v_h / v_leg), which bounds tan(alpha) in
// closed form:
//   v_h <= v_leg cos(lo)  <=>  tan(alpha) >= Z/X + g X / (2 v_leg^2 cos^2 lo)
// and likewise a ceiling from hi. lo is the higher of the cone's lower edge and
// alpha_leg_min (the controller's margin); hi is the cone's upper edge, capped
// at vertical. v_s_min is v_leg.
std::optional<Interval> feasible_alpha_interval_vertical_push(
  double X, double Z, double V_max, double g,
  double mu,
  const Normal & n_s, const Normal & n_g,
  double theta,
  double v_s_min, double e_inject_max, double mass,
  double min_apex, double V_g_max, double alpha_leg_min);

// The push stage 2 measured (PWM 30000, cut off at the target apex, capped at
// t_max): its injection above the idle coast fits a level push at accel for
// the push time plus t_offset, against g, from the leg's vertical speed.
struct PushModel
{
  double accel = 7.55;       // m/s^2, effective specific thrust (motor model 7.98)
  double g_coast = 9.55;     // m/s^2, effective gravity of the idle coast
  double t_offset = -0.001;  // s, spin-up and spin-down, added to the push time
  double t_max = 0.4;        // s, the controller's cap on one push
};

// Most injection [J/kg] one push adds from leg vertical speed v_z0: it runs to
// the cap or to the apex, whichever comes first.
double push_capacity(double v_z0, const PushModel & push, double g);

// Hopcopter's stance model (JumpLib JumpingModel3D.fcn1 with
// InPlaneJumpingModel(k1, k2)). The stance turns the velocity from -v_in to the
// takeoff direction d_out, an angle theta_v, and the body z axis at touchdown
// sits theta_v / k2 of the way from -v_in toward d_out. k1 sets only the
// liftoff attitude, which the planner does not use.
struct Stance
{
  Vec3 axis;       // body z axis at touchdown, world frame (unit)
  double tilt;     // angle of axis from vertical, rad
  double theta_v;  // redirection angle, rad
};

// v_in: arrival velocity (any length); d_out: takeoff direction (unit).
Stance stance(const Vec3 & v_in, const Vec3 & d_out, double k2);

}  // namespace ballistic
