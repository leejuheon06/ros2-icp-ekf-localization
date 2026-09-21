# Benchmark 01 — Odom vs Custom ICP

## Route

```text
START
  -> P1 (3.39557, -4.12722, 0.0)
  -> P2 (5.61326,  4.40286, 0.0)
  -> P3 (7.68631, -1.58174, 0.0)
```

Nav2 is used for planning, obstacle avoidance, costmaps, and control. Custom ICP provides the localization correction through the dynamic `map -> odom` TF. AMCL is not used in this benchmark.

## Measured Result

| Metric | Wheel Odometry | Custom ICP |
|---|---:|---:|
| Overall Position RMSE | 0.1149 m | 0.0428 m |
| Overall Yaw RMSE | 0.0179 rad | 0.0123 rad |
| START -> P1 Position RMSE | 0.0391 m | 0.0351 m |
| P1 -> P2 Position RMSE | 0.1121 m | 0.0321 m |
| P2 -> P3 Position RMSE | 0.1636 m | 0.0604 m |

The overall Position RMSE was approximately 62.7% lower for Custom ICP in this run. The result indicates that Wheel Odometry error accumulated over the longer route, while scan-to-map ICP limited the growth of map-relative position drift. ICP still retained residual error and did not reduce error to zero.

## Files

- `localization_benchmark.csv`: continuous Ground Truth / Odom / ICP samples and waypoint events
- `trajectory_comparison.png`: Ground Truth / Odom / ICP trajectory comparison
- `error_analysis.png`: position error trend and RMSE summary

## Limitation

This directory contains one representative simulation run. Repeated trials, strict estimator timestamp alignment, and processing-time / CPU statistics are planned for the final quantitative evaluation.
