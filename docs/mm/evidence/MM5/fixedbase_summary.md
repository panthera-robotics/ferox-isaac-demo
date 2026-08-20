
## The fixed-base variant — N=20

Base held kinematically, everything downstream of balance exercised for real. **Not the
robot**, and labelled as such on every row: the carry stage is skipped (carrying a
payload 1 m is meaningless when the base cannot move) and the base stands 0.38 m from the
object, inside the clearance Nav2 would demand of the table.

```
20 trials, soup_can, seeds 20260821..20260840
success 2/20 = 10.0%
taxonomy: REACH_TIMEOUT 10  LIFT_FAILED 8  SUCCESS 2  
stage timing (mean):  APPROACH=6.0s(n=20)  REACH=18.8s(n=20)  GRASP=2.0s(n=10)  LIFT=6.4s(n=10)  PLACE=2.0s(n=2)  RELEASE=1.5s(n=2)  RETREAT=2.0s(n=2)
```

**10 of 20 trials reached the pre-grasp** and entered GRASP — the first time in this program the pipeline has got past the reach at all.
