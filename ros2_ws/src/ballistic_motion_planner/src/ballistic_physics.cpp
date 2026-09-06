#include "ballistic_motion_planner/ballistic_physics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ballistic
{

std::optional<Interval> inplane_friction_cone(const Normal & n, double theta, double mu)
{
  const double n_xt = n.x * std::cos(theta) + n.y * std::sin(theta);
  const double n_z = n.z;

  const double A = std::hypot(n_xt, n_z);
  if (A < 1e-12) {
    return std::nullopt;  // normal perpendicular to the hop plane
  }
  const double c = std::cos(std::atan(mu)) / A;
  if (c > 1.0) {
    return std::nullopt;  // degenerate: plane meets the cone only at the apex
  }
  return Interval{std::atan2(n_z, n_xt),
    std::acos(std::min(1.0, std::max(-1.0, c)))};
}

std::optional<Interval> speed_tan_interval(double X, double Z, double W, double g)
{
  if (W <= 0.0) {
    return std::nullopt;
  }
  const double D = W * W - g * g * X * X - 2.0 * g * Z * W;
  if (D < 0.0) {
    return std::nullopt;
  }
  const double s = std::sqrt(D);
  return Interval{(W - s) / (g * X), (W + s) / (g * X)};
}

std::optional<Interval> landing_cone_alpha_s(
  double X, double Z, double gamma_g, double delta_g)
{
  const double gamma_r = gamma_g - M_PI;  // reversed cone axis; in (-pi, 0)

  const double lo_g = std::max(gamma_r - delta_g, -0.5 * M_PI + kAlphaEps);
  const double hi_g = std::min(gamma_r + delta_g, std::atan2(Z, X));
  if (lo_g >= hi_g) {
    return std::nullopt;
  }
  const double k = 2.0 * Z / X;  // POSITIVE sign — see header note
  return Interval{std::atan(k - std::tan(hi_g)), std::atan(k - std::tan(lo_g))};
}

std::optional<double> min_apex_tan(double X, double Z, double h_min)
{
  if (X < 1e-9) {
    return std::nullopt;
  }
  const double w = h_min + Z;
  if (w < 0.0) {
    return std::nullopt;  // the terrain drop alone already exceeds h_min
  }
  return 2.0 * (Z + h_min + std::sqrt(h_min * w)) / X;
}

double min_energy_tan(double X, double Z)
{
  return (Z + std::hypot(Z, X)) / X;
}

double xdot(double X, double Z, double alpha_s, double g)
{
  const double denom = 2.0 * (X * std::tan(alpha_s) - Z);
  return std::sqrt(g * X * X / denom);
}

double takeoff_speed(double X, double Z, double alpha_s, double g)
{
  return xdot(X, Z, alpha_s, g) / std::cos(alpha_s);
}

double landing_speed(double v_s, double Z, double g)
{
  return std::sqrt(std::max(0.0, v_s * v_s - 2.0 * g * Z));
}

double injection_energy(double v_s, double v_s_min, double mass)
{
  return 0.5 * mass * std::max(0.0, v_s * v_s - v_s_min * v_s_min);
}

double max_hop_radius(
  double v_g_in, double eta, double e_inject_max, double mass, double g, double V_max)
{
  const double v_s_min = std::sqrt(eta) * v_g_in;
  const double v_s_max = std::sqrt(std::min(
      v_s_min * v_s_min + 2.0 * e_inject_max / mass, V_max * V_max));
  return v_s_max * v_s_max / g;
}

std::optional<Interval> feasible_alpha_interval(
  double X, double Z, double V_max, double g,
  double mu,
  const Normal & n_s, const Normal & n_g,
  double theta,
  double v_s_min, double e_inject_max, double mass,
  double min_apex, double V_g_max)
{
  if (X < 1e-9) {
    return std::nullopt;  // degenerate horizontal displacement
  }

  // (E1) energy floor and (E2) injection ceiling.
  const double W_lo = v_s_min * v_s_min;
  const double W_hi = std::min(W_lo + 2.0 * e_inject_max / mass, V_max * V_max);

  const auto iv_hi = speed_tan_interval(X, Z, W_hi, g);
  if (!iv_hi) {
    return std::nullopt;  // unreachable even at full thrust
  }
  const auto iv_lo = speed_tan_interval(X, Z, W_lo, g);
  // Upper branch of the floor when v_s_min can reach the target; else the
  // binding lower bound is the ceiling interval's own lower root.
  double alpha_lo = std::atan(iv_lo ? iv_lo->second : iv_hi->first);
  double alpha_hi = std::atan(iv_hi->second);

  // (E3) minimum apex-to-landing drop — max against the energy floor.
  const auto T_apex = min_apex_tan(X, Z, min_apex);
  if (T_apex) {
    alpha_lo = std::max(alpha_lo, std::atan(*T_apex));
  }

  // (4) landing speed v_g <= V_g_max.
  const auto tan_iv = speed_tan_interval(X, Z, V_g_max * V_g_max + 2.0 * g * Z, g);
  if (!tan_iv) {
    return std::nullopt;
  }
  alpha_lo = std::max(alpha_lo, std::atan(tan_iv->first));
  alpha_hi = std::min(alpha_hi, std::atan(tan_iv->second));

  // (1) takeoff non-sliding.
  const auto cone_s = inplane_friction_cone(n_s, theta, mu);
  if (!cone_s) {
    return std::nullopt;
  }
  alpha_lo = std::max(alpha_lo, cone_s->first - cone_s->second);
  alpha_hi = std::min(alpha_hi, cone_s->first + cone_s->second);

  // (2) landing non-sliding — mapped back through the parabola.
  const auto cone_g = inplane_friction_cone(n_g, theta, mu);
  if (!cone_g) {
    return std::nullopt;
  }
  const auto land_iv = landing_cone_alpha_s(X, Z, cone_g->first, cone_g->second);
  if (!land_iv) {
    return std::nullopt;
  }
  alpha_lo = std::max(alpha_lo, land_iv->first);
  alpha_hi = std::min(alpha_hi, land_iv->second);

  const double alpha_min = alpha_lo + kAlphaEps;
  const double alpha_max = alpha_hi - kAlphaEps;
  if (alpha_min >= alpha_max) {
    return std::nullopt;
  }
  return Interval{alpha_min, alpha_max};
}

std::optional<ArcSamples> arc_samples(
  double x_s, double y_s, double t_s,
  double x_g, double y_g, double t_g,
  const Map2D5 & map, const std::vector<double> & inflated, double max_step)
{
  const double dx = x_g - x_s;
  const double dy = y_g - y_s;
  const double X = std::hypot(dx, dy);
  if (X < 1e-9) {
    return std::nullopt;
  }

  const double step = std::min(max_step, map.resolution() / 3.0);
  const int n = std::max(3, static_cast<int>(std::ceil(X / step)) + 1);

  ArcSamples out;
  out.X = X;
  out.u.resize(n);
  out.field.resize(n);
  out.active.assign(n, 0);
  out.any_active = false;

  const double t_max = std::max(t_s, t_g);
  const double res = map.resolution();
  for (int i = 0; i < n; ++i) {
    const double u = X * i / (n - 1);  // linspace(0, X, n)
    out.u[i] = u;
    const double cx = x_s + u * (dx / X);
    const double cy = y_s + u * (dy / X);
    if (!map.is_within_bounds(cx, cy)) {
      return std::nullopt;  // centreline leaves the map
    }
    // Nearest-cell lookup: truncate then clamp (matches Python int64 cast).
    int ci = static_cast<int>(std::trunc((cx - map.origin_x()) / res));
    int ri = static_cast<int>(std::trunc((cy - map.origin_y()) / res));
    ci = std::max(0, std::min(ci, map.cols() - 1));
    ri = std::max(0, std::min(ri, map.rows() - 1));
    out.field[i] = inflated[static_cast<size_t>(ri) * map.cols() + ci];
    // Only terrain above BOTH endpoints can reject the hop.
    if (i != 0 && i != n - 1 && out.field[i] > t_max) {
      out.active[i] = 1;
      out.any_active = true;
    }
  }
  return out;
}

std::optional<double> clearance_floor_alpha(
  double x_s, double y_s, double t_s,
  double x_g, double y_g, double t_g,
  const Map2D5 & map, const std::vector<double> & inflated,
  double gate, double max_step)
{
  const auto sampled = arc_samples(x_s, y_s, t_s, x_g, y_g, t_g, map, inflated, max_step);
  if (!sampled) {
    return std::nullopt;
  }
  if (!sampled->any_active) {
    return -std::numeric_limits<double>::infinity();
  }

  const double X = sampled->X;
  const double Z = t_g - t_s;
  double max_T = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < sampled->u.size(); ++i) {
    if (!sampled->active[i]) {continue;}
    const double ua = sampled->u[i];
    const double numer = sampled->field[i] + gate - t_s - Z * (ua * ua) / (X * X);
    const double denom = ua * (X - ua) / X;
    max_T = std::max(max_T, numer / denom);
  }
  return std::atan(max_T);
}

}  // namespace ballistic
