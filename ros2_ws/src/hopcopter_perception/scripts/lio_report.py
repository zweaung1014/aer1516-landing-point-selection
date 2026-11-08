#!/usr/bin/env python3
"""FAST-LIO against Gazebo, from lio_anchor's log (stage 2 of docs/lidar_mapping_plan.md).

    lio_report.py [lio_<stamp>.csv]

Reads the newest data/data_lio/lio_*.csv by default and prints the stage-2 pass
criteria: position RMSE <= 5 cm and max <= 10 cm, attitude error <= 1 deg, no
jump in the error at touchdowns, no divergence. Touchdowns are the minima of
Gazebo's LiDAR height between hops.
"""
import csv
import glob
import os
import sys

import numpy as np

DATA = '/home/zweminhtetaung/CrazySim/data/data_lio'


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        files = sorted(glob.glob(os.path.join(DATA, 'lio_*.csv')), key=os.path.getmtime)
        if not files:
            sys.exit(f'no log in {DATA}')
        path = files[-1]
    rows = list(csv.DictReader(open(path)))
    if len(rows) < 10:
        sys.exit(f'{path}: only {len(rows)} rows')
    col = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    t, err, att = col('stamp'), col('err'), col('att_err_deg')
    ez, z = col('err_z'), col('truth_z')
    t = t - t[0]
    print(f'{path}: {len(rows)} scans over {t[-1]:.1f} s')
    rmse = float(np.sqrt(np.mean(err ** 2)))
    print(f'  position error: RMSE {rmse:.3f} m, median {np.median(err):.3f}, max {err.max():.3f} '
          f'(at {t[err.argmax()]:.1f} s); z RMSE {np.sqrt(np.mean(ez ** 2)):.3f}')
    print(f'  attitude error: median {np.median(att):.2f} deg, p95 {np.percentile(att, 95):.2f}, '
          f'max {att.max():.2f}')
    # Touchdowns: local minima of the LiDAR's height, at least 0.3 m below the last apex
    td = [i for i in range(1, len(z) - 1) if z[i] <= z[i - 1] and z[i] <= z[i + 1]]
    td = [i for i in td if z[max(0, i - 8):i + 1].max() - z[i] > 0.3]
    if td:
        jumps = [abs(err[min(i + 1, len(err) - 1)] - err[max(i - 1, 0)]) for i in td]
        print(f'  {len(td)} touchdowns: error change across a touchdown median {np.median(jumps):.3f}, '
              f'max {max(jumps):.3f} m')
    q = len(err) // 4
    print(f'  error by quarter of the run: ' +
          ', '.join(f'{np.sqrt(np.mean(err[k * q:(k + 1) * q] ** 2)):.3f}' for k in range(4)) + ' m RMSE')
    ok = rmse <= 0.05 and err.max() <= 0.10 and att.max() <= 1.0
    print(f'  stage-2 criteria (RMSE <= 0.05, max <= 0.10, attitude <= 1 deg): {"pass" if ok else "FAIL"}')


if __name__ == '__main__':
    main()
