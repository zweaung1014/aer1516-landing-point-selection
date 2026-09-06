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

}  // namespace ballistic
