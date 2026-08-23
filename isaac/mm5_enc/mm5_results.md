# MM5 results — 4 trials, soup_can

**Balancer: omni locomotion policy** (SONIC parked at C-39; CAMPAIGN 4.4 permits "or omni standing if MM4 slips").

**Success rate: 0/4 = 0.0%**

| outcome | count |
|---|---|
| `TOPPLED` | 2 |
| `DESCEND_TIMEOUT` | 2 |

| trial | seed | outcome | detail | obj start | obj end | stage times (s) |
|---|---|---|---|---|---|---|
| 1 | 20260902 | `TOPPLED` | tilt 30.0 deg during closure | [-2.5492, 2.2651, 0.9498] | [-2.5288, 2.3138, 0.9508] | APPROACH=6.005 REACH=3.795 DESCEND=10.75 GRASP=0.625 |
| 2 | 20260903 | `DESCEND_TIMEOUT` | 37 mm from grasp pose | [-2.6293, 2.23, 0.9498] | [-2.526, 2.2434, 0.9326] | APPROACH=6.005 REACH=1.065 DESCEND=15.005 |
| 3 | 20260904 | `DESCEND_TIMEOUT` | 39 mm from grasp pose | [-2.5573, 2.294, 0.9498] | [-2.4014, 2.3019, 0.9321] | APPROACH=6.005 REACH=5.285 DESCEND=15.005 |
| 4 | 20260905 | `TOPPLED` | tilt 30.3 deg during closure | [-2.6066, 2.3028, 0.9498] | [-2.5769, 2.3511, 0.9516] | APPROACH=6.005 REACH=4.895 DESCEND=5.51 GRASP=0.62 |
