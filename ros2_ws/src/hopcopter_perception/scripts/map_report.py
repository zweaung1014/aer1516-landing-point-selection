#!/usr/bin/env python3
"""The LiDAR map's score against the known map (stage 3 of docs/lidar_mapping_plan.md).

    map_report.py [score_<stamp>.csv]

Reads lidar_elevation_mapper's newest data/data_elevation_map/score_*.csv and
prints how the map evolved and its last state against the stage-3 criteria:
height error away from edges median <= 2 cm and 95th percentile <= 5 cm, no
false obstacles away from edges, no unseen cell bounded below the terrain.
"""
import csv
import glob
import os
import sys

DATA = '/home/zweminhtetaung/CrazySim/data/data_elevation_map'


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        files = sorted(glob.glob(os.path.join(DATA, 'score_*.csv')), key=os.path.getmtime)
        if not files:
            sys.exit(f'no score in {DATA}')
        path = files[-1]
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit(f'{path}: empty')
    print(f'{path}: {len(rows)} updates')
    step = max(1, len(rows) // 8)
    print('  t [s]   scans  coverage  bounded  unknown  err_med  err_p95  false  missed  bound_viol  ms/scan')
    for r in rows[::step] + [rows[-1]]:
        print(f"  {float(r['stamp']):7.1f} {int(r['scans']):6d}  {float(r['coverage']):7.2f}  "
              f"{int(r['bounded_unseen']):7d}  {int(r['unbounded']):7d}  {float(r['err_median']):7.3f}  "
              f"{float(r['err_p95']):7.3f}  {int(r['false_flat']):5d}  {int(r['missed']):6d}  "
              f"{int(r['bound_violations']):10d}  {float(r['scan_ms_mean']):7.1f}")
    last = rows[-1]
    ok = (float(last['err_median']) <= 0.02 and float(last['err_p95']) <= 0.05 and
          int(last['false_flat']) == 0 and int(last['bound_violations']) == 0)
    print(f"  last: median {float(last['err_median']):.3f}, p95 {float(last['err_p95']):.3f} m, "
          f"{last['false_flat']} false obstacles away from edges ({last['false_edge']} by an edge), "
          f"{last['bound_violations']} bounds below the terrain: {'pass' if ok else 'FAIL'}")


if __name__ == '__main__':
    main()
