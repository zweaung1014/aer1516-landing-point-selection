#!/usr/bin/env python3
"""Render a ballistic_planner_node snapshot to a PNG.

The node writes vis_planned_path/snapshot_<stamp>.json after every plan and
launches this script on it; the figure lands next to it as
planned_path_<stamp>.png. Layout mirrors hopcopter-ballistic-planning's
main.py: the top-down path plan on top, the three map-analysis panels below.
Self-contained on purpose: that repo is reference-only and is not imported.

Usage:
    visualize_planned_path.py [snapshot.json]   # default: newest snapshot
"""

import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.colors as mcolors  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_DIR = "/home/zweminhtetaung/CrazySim/data/data_ballistic_planner/vis_planned_path"
OBSTACLE = -1.0


def load(path):
    with open(path) as f:
        s = json.load(f)
    shape = (s["rows"], s["cols"])
    s["grid"] = np.array(s["grid"], dtype=float).reshape(shape)
    s["steep"] = np.array(s["steep"], dtype=float).reshape(shape)
    s["standable"] = np.array(s["standable"], dtype=float).reshape(shape)
    # null = +inf (OBSTACLE columns inflate to infinity)
    s["inflated"] = np.array(
        [np.inf if v is None else v for v in s["inflated"]], dtype=float
    ).reshape(shape)
    res = s["resolution"]
    s["extent"] = [s["origin_x"], s["origin_x"] + s["cols"] * res,
                   s["origin_y"], s["origin_y"] + s["rows"] * res]
    return s


def draw_path_figure(s):
    """Port of visualizer.Visualizer: map, hop rings, path, start/goal, body."""
    fig, ax = plt.subplots(figsize=(8, 8))
    x0, x1, y0, y1 = s["extent"]
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("A* Path Planning on 2.5D Map")

    # Elevation (Greys, normalised over non-obstacle cells); obstacles black.
    grid = s["grid"]
    obstacle = grid == OBSTACLE
    non_obstacle = grid[~obstacle]
    z_max = float(non_obstacle.max()) if non_obstacle.size else 1.0
    if z_max == 0.0:
        z_max = 1.0
    norm = mcolors.Normalize(vmin=0.0, vmax=z_max)
    cmap = plt.get_cmap("Greys").copy()
    cmap.set_bad("black", alpha=0.85)
    ax.imshow(np.ma.masked_where(obstacle, grid), origin="lower", extent=s["extent"],
              cmap=cmap, norm=norm, alpha=0.75, interpolation="nearest")
    sm = plt.cm.ScalarMappable(cmap=plt.get_cmap("Greys"), norm=norm)
    sm.set_array([])
    fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02).set_label("Elevation (m)")

    path = s["path"]
    if path:
        # hop_radii[i] is the ring available when departing waypoint i.
        for i, ((x, y), r) in enumerate(zip(path, s["hop_radii"])):
            ax.add_patch(mpatches.Circle(
                (x, y), radius=r, fill=False, edgecolor="deepskyblue",
                linewidth=0.8, linestyle="--", alpha=0.6,
                label="Hop radius" if i == 0 else None,
            ))
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color="red", linewidth=2.0, label="Path")
        ax.plot(xs, ys, "o", color="red", markersize=5, zorder=5)
        # Target apex the node publishes as each waypoint's z.
        for i, (x, y) in enumerate(path):
            ax.annotate(f"{i}: {s['target_apex'][i]:.2f} m", (x, y),
                        textcoords="offset points", xytext=(6, 6),
                        fontsize=7, color="darkred", zorder=6)

    ax.plot(*s["start"], "go", markersize=10, label="Start")
    ax.plot(*s["goal"], "r*", markersize=15, label="Goal")

    r_body = s["robot_radius"]
    mc = s["min_clearance"]
    for k, pose in enumerate((s["start"], s["goal"])):
        ax.add_patch(mpatches.Circle(
            pose, radius=r_body, fill=False, edgecolor="#1565c0",
            linewidth=1.4, alpha=0.9,
            label=f"Robot body (r={r_body:g} m)" if k == 0 else None,
        ))
        ax.add_patch(mpatches.Circle(
            pose, radius=r_body + mc, fill=False, edgecolor="#1565c0",
            linewidth=0.8, linestyle="--", alpha=0.6,
            label=f"Clearance envelope (+{mc:g} m)" if k == 0 else None,
        ))

    handles, labels = ax.get_legend_handles_labels()
    handles.append(mpatches.Patch(facecolor="black", alpha=0.85, label="Obstacle"))
    labels.append("Obstacle")
    # Below the axes: inside, it always lands on the path or the terrain.
    ax.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.08),
              ncol=4, fontsize=8, frameon=False)
    fig.tight_layout()
    return fig


def draw_map_analysis(s):
    """Port of visualizer.draw_map_analysis: steep / standable / inflated."""
    extent = s["extent"]
    reach = s["robot_radius"] + s["min_clearance"]
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5.5))
    fig.suptitle("Map analysis", fontsize=13)

    cmap1 = mcolors.ListedColormap(["#eeeeee", "#e65100"])
    im1 = ax1.imshow(s["steep"], origin="lower", extent=extent,
                     cmap=cmap1, vmin=0, vmax=1, aspect="equal")
    cbar1 = fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04, ticks=[0.25, 0.75])
    cbar1.ax.set_yticklabels(["flat", "edge"])
    ax1.set_title(f"Steep mask  (grade ≥ {s['steep_grade']:.2f})")

    cmap2 = mcolors.ListedColormap(["#c62828", "#66bb6a"])
    im2 = ax2.imshow(s["standable"], origin="lower", extent=extent,
                     cmap=cmap2, vmin=0, vmax=1, aspect="equal")
    cbar2 = fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04, ticks=[0.25, 0.75])
    cbar2.ax.set_yticklabels(["blocked", "standable"])
    ax2.set_title(f"Standable mask  (r = {reach:.2f} m)")

    field = np.where(np.isinf(s["inflated"]), np.nan, s["inflated"])
    cmap3 = plt.get_cmap("Blues").copy()
    cmap3.set_bad("black")
    im3 = ax3.imshow(field, origin="lower", extent=extent, cmap=cmap3, aspect="equal")
    fig.colorbar(im3, ax=ax3, fraction=0.046, pad=0.04).set_label("inflated height (m)")
    ax3.set_title(f"Inflated field  (r = {reach:.2f} m)")

    for ax in (ax1, ax2, ax3):
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    fig.tight_layout()
    return fig


def _arc_z(u, X, Z, z_s, alpha_s):
    """CoM height along the arc (Campana Eq. 2, xdot-free form)."""
    tan_a = np.tan(alpha_s)
    k = (X * tan_a - Z) / (X * X)
    return z_s + u * tan_a - k * u * u


def _nullable(values, fill):
    return np.array([fill if v is None else v for v in values], dtype=float)


def draw_hop_side_view(ax, i, hop, s, z_top):
    """Port of visualizer.draw_arc_side_view, fed by the C++ planner's samples.

    Inflated terrain (what the clearance gate reads), CoM arc, foot line,
    clearance envelope, and the tightest sample. Additions: raw ground along
    the centreline and the hop's target apex (CoM rise above takeoff).
    """
    leg = s["leg_length"]
    gate = s["min_clearance"]
    X, Z, t_s, alpha = hop["X"], hop["Z"], hop["t_s"], hop["alpha_s"]
    u = np.array(hop["u"], dtype=float)
    if u.size == 0 or X < 1e-9:
        ax.set_title(f"hop {i + 1}: no samples")
        return
    # OBSTACLE columns inflate to +inf: clamp to the panel top for drawing only.
    field = np.minimum(_nullable(hop["field"], np.inf), z_top)
    ground = np.array(hop["ground"], dtype=float)
    ground = np.where(ground == OBSTACLE, z_top, ground)

    min_c = hop["min_clearance"]  # None = nothing along the hop constrains it
    rejected = min_c is not None and min_c < gate
    colour = "#c62828" if rejected else "#2e7d32"

    ud = np.linspace(0.0, X, max(64, int(np.ceil(X / 0.01)) + 1))
    z_arc = _arc_z(ud, X, Z, t_s + leg, alpha)
    z_foot = z_arc - leg
    z_env = z_foot - gate

    floor = min(float(ground.min()), float(z_env.min())) - 0.1
    ax.fill_between(u, field, floor, step="mid", color="#8d6e63", alpha=0.55,
                    linewidth=0, label="Terrain (inflated by body)")
    ax.step(u, ground, where="mid", color="#4e342e", linewidth=1.0,
            label="Ground (raw)")
    ax.plot(ud, z_arc, color=colour, linewidth=2.0, label="CoM arc")
    ax.plot(ud, z_foot, color=colour, linewidth=1.0, alpha=0.6,
            label=f"Foot tip (arc − L, L={leg:g} m)")
    ax.plot(ud, z_env, color=colour, linewidth=1.0, linestyle="--", alpha=0.6,
            label=f"Clearance envelope (foot − {gate:g} m)")

    # Tightest sample among those the gate actually checks (the planner's
    # `active` mask); none active = nothing constrains the hop, no marker.
    env_s = _arc_z(u, X, Z, t_s, alpha) - gate
    active = np.array(hop["active"], dtype=bool)
    gap = np.where(active, env_s - field, np.inf)
    j = int(np.argmin(gap))
    if np.isfinite(gap[j]):
        ax.plot([u[j], u[j]], [env_s[j], field[j]], color=colour,
                linewidth=1.2, linestyle=":")
        ax.plot(u[j], env_s[j], "o", color=colour, markersize=5)

    # target_apex = apex_drop + Z: CoM rise from takeoff to apex.
    k = int(np.argmax(z_arc))
    z0 = t_s + leg
    ax.plot([0.0, ud[k]], [z0, z0], color="#1565c0", linewidth=0.8, linestyle=":")
    ax.annotate("", xy=(ud[k], z_arc[k]), xytext=(ud[k], z0),
                arrowprops=dict(arrowstyle="<->", color="#1565c0", lw=1.0))
    ax.plot(ud[k], z_arc[k], "^", color="#1565c0", markersize=6,
            label="Apex (target_apex = CoM rise)")
    ax.annotate(f"{hop['target_apex']:.2f} m", (ud[k], (z0 + z_arc[k]) / 2),
                textcoords="offset points", xytext=(5, 0), fontsize=8,
                color="#1565c0", va="center")

    verdict = "REJECT" if rejected else "ACCEPT"
    clear = "∞ (unconstrained)" if min_c is None else f"{min_c:+.3f} m"
    title = (f"hop {i + 1}: X={X:.2f} m, Z={Z:+.2f} m\n"
             f"{verdict}   {clear}   α = {np.degrees(alpha):.1f}°")
    if "alpha_leg" in hop:  # snapshots since stage 3: the leg angle, and the tilt it lands at
        title += (f"\nleg {np.degrees(hop['alpha_leg']):.1f}°, "
                  f"landing tilt {np.degrees(hop['land_tilt']):.1f}°")
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("u (m along hop)")
    ax.set_xlim(0, X)
    ax.grid(alpha=0.25)


def draw_side_strip(s, per_row=4):
    """One side view per hop, shared z axis so arc heights compare."""
    hops = s["hops"]
    n = len(hops)
    ncols = min(n, per_row)
    nrows = -(-n // per_row)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.5 * ncols, 3.8 * nrows),
                             sharey=True, squeeze=False)

    leg = s["leg_length"]
    tops = []
    for h in hops:
        ud = np.linspace(0.0, h["X"], 64)
        tops.append(float(_arc_z(ud, h["X"], h["Z"], h["t_s"] + leg, h["alpha_s"]).max()))
    z_top = max(tops) + 0.2

    for i, h in enumerate(hops):
        draw_hop_side_view(axes[i // per_row][i % per_row], i, h, s, z_top)
    for k in range(n, nrows * ncols):
        axes[k // per_row][k % per_row].axis("off")
    for r in range(nrows):
        axes[r][0].set_ylabel("z (m)")
    axes[0][0].set_ylim(top=z_top)

    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=8, frameon=False)
    fig.suptitle(f"Side view per hop (clearance gate = {s['min_clearance']:g} m)",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    return fig, nrows


def to_image(fig):
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height(physical=True)
    img = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
    plt.close(fig)
    return img


def main():
    if len(sys.argv) > 1:
        snapshot = sys.argv[1]
    else:
        found = sorted(glob.glob(os.path.join(DEFAULT_DIR, "snapshot_*.json")))
        if not found:
            print(f"No snapshot_*.json in {DEFAULT_DIR}")
            sys.exit(1)
        snapshot = found[-1]

    s = load(snapshot)
    images = [to_image(draw_path_figure(s))]
    ratios = [10]
    # Older (format 1) snapshots carry no per-hop data: skip the side views.
    if s.get("hops"):
        side_fig, side_rows = draw_side_strip(s)
        images.append(to_image(side_fig))
        ratios.append(4 * side_rows)
    images.append(to_image(draw_map_analysis(s)))
    ratios.append(3)

    # Stitch as in main.py: path large on top, side views, analysis row below.
    fig, axes = plt.subplots(
        len(images), 1, figsize=(10, 10 * sum(ratios) / 10),
        gridspec_kw={"height_ratios": ratios})
    for ax, img in zip(axes, images):
        ax.imshow(img)
        ax.axis("off")
    fig.tight_layout(pad=0.3)

    out = os.path.join(os.path.dirname(os.path.abspath(snapshot)),
                       f"planned_path_{s['stamp']}.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
