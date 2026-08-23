# MM5 results — 10 trials, block_3cm

**Balancer: omni locomotion policy** (SONIC parked at C-39; CAMPAIGN 4.4 permits "or omni standing if MM4 slips").

**Success rate: 0/10 = 0.0%**

| outcome | count |
|---|---|
| `DESCEND_TIMEOUT` | 5 |
| `REACH_TIMEOUT` | 4 |
| `TOPPLED` | 1 |

| trial | seed | outcome | detail | obj start | obj end | stage times (s) |
|---|---|---|---|---|---|---|
| 1 | 20260992 | `DESCEND_TIMEOUT` | 47 mm from grasp pose | [-2.5777, 2.218, 0.95] | [-2.5777, 2.2181, 0.948] | APPROACH=6.005 REACH=0.99 DESCEND=120.005 |
| 2 | 20260993 | `REACH_TIMEOUT` | 90 mm from pre-grasp | [-2.6451, 2.2583, 0.95] | [-2.6451, 2.2583, 0.948] | APPROACH=6.005 REACH=30.005 |
| 3 | 20260994 | `TOPPLED` | tilt 36.9 deg during closure | [-2.6363, 2.1927, 0.95] | [-2.6227, 2.1941, 0.949] | APPROACH=6.005 REACH=0.72 DESCEND=59.78 GRASP=0.005 |
| 4 | 20260995 | `DESCEND_TIMEOUT` | 119 mm from grasp pose | [-2.6053, 2.2507, 0.95] | [-2.6053, 2.2507, 0.948] | APPROACH=6.005 REACH=1.255 DESCEND=120.005 |
| 5 | 20260996 | `REACH_TIMEOUT` | 178 mm from pre-grasp | [-2.5737, 2.3081, 0.95] | [-2.5737, 2.3081, 0.948] | APPROACH=6.005 REACH=30.005 |
| 6 | 20260997 | `DESCEND_TIMEOUT` | 93 mm from grasp pose | [-2.6574, 2.2257, 0.95] | [-2.6574, 2.2257, 0.948] | APPROACH=6.005 REACH=0.945 DESCEND=120.005 |
| 7 | 20260998 | `DESCEND_TIMEOUT` | 111 mm from grasp pose | [-2.6, 2.2252, 0.95] | [-2.6, 2.2252, 0.948] | APPROACH=6.005 REACH=0.97 DESCEND=120.005 |
| 8 | 20260999 | `REACH_TIMEOUT` | 96 mm from pre-grasp | [-2.607, 2.271, 0.95] | [-2.607, 2.271, 0.948] | APPROACH=6.005 REACH=30.005 |
| 9 | 20261000 | `REACH_TIMEOUT` | 101 mm from pre-grasp | [-2.6553, 2.2662, 0.95] | [-2.6553, 2.2662, 0.948] | APPROACH=6.005 REACH=30.005 |
| 10 | 20261001 | `DESCEND_TIMEOUT` | 152 mm from grasp pose | [-2.5831, 2.2852, 0.95] | [-2.5831, 2.2852, 0.948] | APPROACH=6.005 REACH=17.805 DESCEND=120.005 |
