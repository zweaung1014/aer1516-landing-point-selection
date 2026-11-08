#!/usr/bin/env python3
"""Plot a LiDAR elevation map snapshot next to the known map and the error.

    plot_elevation_map.py [snapshot.json] [--show]

Reads lidar_elevation_mapper's snapshot_<stamp>_<n>.json (the newest in
data/data_elevation_map/ by default) or mapper_check's mapper_check_<world>.json,
and writes <snapshot>.png next to it:
  - the LiDAR map: measured heights; unseen cells with an upper bound hatched and
    shaded by the bound; unseen, unbounded cells (walls to the planner) black;
  - the known map;
  - the error, LiDAR - known, on measured cells;
  - the upper bound on every bounded cell, and where it is below the known terrain.
"""
import glob
import json
import os
import sys

import numpy as np
import matplotlib
if '--show' not in sys.argv:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

DATA = '/home/zweminhtetaung/CrazySim/data/data_elevation_map'


def grid(snap, key):
    v = snap.get(key)
    if v is None:
        return None
    a = np.array([np.nan if x is None else x for x in v], dtype=float)
    return a.reshape(snap['rows'], snap['cols'])


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if args:
        path = args[0]
    else:
        files = sorted(glob.glob(os.path.join(DATA, 'snapshot_*.json')), key=os.path.getmtime)
        if not files:
            sys.exit(f'no snapshot in {DATA}')
        path = files[-1]
    snap = json.load(open(path))
    el, ub, known = grid(snap, 'elevation'), grid(snap, 'upper_bound'), grid(snap, 'known')
    res = snap['resolution']
    x0, y0 = snap['origin_x'], snap['origin_y']
    ext = [x0, x0 + snap['cols'] * res, y0, y0 + snap['rows'] * res]
    unseen_bounded = np.isnan(el) & np.isfinite(ub)
    unknown = np.isnan(el) & np.isnan(ub)

    finite = [a[np.isfinite(a)] for a in (el, known) if a is not None]
    vmin = min(float(f.min()) for f in finite if f.size) if finite else 0.0
    vmax = max(float(f.max()) for f in finite if f.size) if finite else 1.0
    n = 4 if known is not None else 2
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n, 4.6 * (ext[3] - ext[2]) / (ext[1] - ext[0]) + 1.2),
                             squeeze=False)
    axes = axes[0]
    kw = dict(origin='lower', extent=ext, interpolation='nearest')

    ax = axes[0]
    im = ax.imshow(el, cmap='viridis', vmin=vmin, vmax=vmax, **kw)
    ax.imshow(np.where(unseen_bounded, ub, np.nan), cmap='Greys', vmin=vmin, vmax=vmax, alpha=0.6, **kw)
    ax.contourf(np.linspace(ext[0], ext[1], snap['cols']), np.linspace(ext[2], ext[3], snap['rows']),
                unseen_bounded.astype(float), levels=[0.5, 1.5], hatches=['////'], colors='none')
    ax.imshow(np.where(unknown, 1.0, np.nan), cmap=ListedColormap(['black']), **kw)
    if 'trajectory' in snap:
        t = np.array(snap['trajectory'])
        ax.plot(t[:, 0], t[:, 1], 'r-', lw=0.8)
    ax.set_title(f"LiDAR map: {np.isfinite(el).sum()} measured,\n{unseen_bounded.sum()} bounded "
                 f"(hatched), {unknown.sum()} unknown (black)", fontsize=9)
    fig.colorbar(im, ax=ax, fraction=0.046, label='height [m]')

    if known is not None:
        ax = axes[1]
        im = ax.imshow(known, cmap='viridis', vmin=vmin, vmax=vmax, **kw)
        ax.imshow(np.where(np.isnan(known), 1.0, np.nan), cmap=ListedColormap(['black']), **kw)
        ax.set_title('known map (black: obstacle)', fontsize=9)
        fig.colorbar(im, ax=ax, fraction=0.046, label='height [m]')
        ax = axes[2]
        err = el - known
        im = ax.imshow(err, cmap='RdBu_r', vmin=-0.2, vmax=0.2, **kw)
        e = np.abs(err[np.isfinite(err)])
        ax.set_title(f"LiDAR - known on measured cells\nmedian {np.median(e):.3f}, "
                     f"p95 {np.percentile(e, 95):.3f} m" if e.size else 'no overlap', fontsize=9)
        fig.colorbar(im, ax=ax, fraction=0.046, label='error [m]')
    ax = axes[-1]
    im = ax.imshow(ub, cmap='magma', vmin=vmin, vmax=max(vmax, vmin + 0.5), **kw)
    title = 'upper bound'
    if known is not None:
        bad = np.isfinite(ub) & np.isnan(el) & np.isfinite(known) & (ub < known - 0.05)
        if bad.any():
            ys, xs = np.nonzero(bad)
            ax.plot(x0 + (xs + 0.5) * res, y0 + (ys + 0.5) * res, 'cx', ms=4)
        title += f'\n{bad.sum()} unseen cells bounded below the known terrain (x)'
    ax.set_title(title, fontsize=9)
    fig.colorbar(im, ax=ax, fraction=0.046, label='bound [m]')
    for ax in axes:
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
    fig.suptitle(os.path.basename(path), fontsize=9)
    fig.tight_layout()
    out = os.path.splitext(path)[0] + '.png'
    fig.savefig(out, dpi=120)
    print(out)
    if '--show' in sys.argv:
        plt.show()


if __name__ == '__main__':
    main()
