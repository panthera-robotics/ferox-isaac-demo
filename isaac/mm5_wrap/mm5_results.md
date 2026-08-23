# MM5 results — 4 trials, soup_can

**Balancer: omni locomotion policy** (SONIC parked at C-39; CAMPAIGN 4.4 permits "or omni standing if MM4 slips").

**Success rate: 0/4 = 0.0%**

| outcome | count |
|---|---|
| `DESCEND_TIMEOUT` | 2 |
| `NO_GRIP` | 1 |
| `REACH_TIMEOUT` | 1 |

| trial | seed | outcome | detail | obj start | obj end | stage times (s) |
|---|---|---|---|---|---|---|
| 1 | 20260902 | `NO_GRIP` | hand closed, object rose -0.017 m | [-2.5492, 2.2651, 0.9498] | [-2.4374, 2.3847, 0.9326] | APPROACH=6.005 REACH=3.795 DESCEND=10.75 GRASP=3.005 LIFT=8.005 |
| 2 | 20260903 | `DESCEND_TIMEOUT` | 75 mm from grasp pose | [-2.6293, 2.23, 0.9498] | [-2.6293, 2.23, 0.9326] | APPROACH=6.005 REACH=1.005 DESCEND=15.005 |
| 3 | 20260904 | `REACH_TIMEOUT` | 110 mm from pre-grasp | [-2.5573, 2.294, 0.9498] | [-2.5573, 2.294, 0.9326] | APPROACH=6.005 REACH=30.005 |
| 4 | 20260905 | `DESCEND_TIMEOUT` | 46 mm from grasp pose | [-2.6066, 2.3028, 0.9498] | [-2.5601, 2.3416, 0.9327] | APPROACH=6.005 REACH=4.68 DESCEND=15.005 |
