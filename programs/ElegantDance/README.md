# ElegantDance

A non-interactive showcase choreography: each of the six joints follows its own
sinusoid whose frequency and phase are drawn from a tiny LCG **seeded from the
live pose**, so the path is a different complex Lissajous-style figure on every
run. A single rise-and-fall envelope swells the amplitude, speed, and
acceleration to a fast peak mid-run and eases back down, then the arm returns to
its nominal pose. Motion is generated in **joint space** around a
well-conditioned, elbow-folded nominal pose with bounded amplitudes, so every
waypoint is reachable and far from singularities; peak speed stays under the
UR10's 2.09 rad/s base-joint limit.

Emits `textmsg` checkpoints (marker `dance/`): `dance/start`, `dance/at_nominal`,
`dance/done`.

## Run it

**From PolyScope (teach pendant / noVNC at http://localhost:6080):**
The `.urp` + `.installation` are already a matched pair. Copy them into the
controller's program directory, then Load + Play:

```bash
docker cp ElegantDance.urp          ur-docker-ursim-1:/ursim/programs/
docker cp ElegantDance.installation ur-docker-ursim-1:/ursim/programs/
# In PolyScope: Load Program -> ElegantDance.urp -> Play   (needs Remote mode for Dashboard 'play')
```

**From the host over the Primary interface (works in Local *or* Remote mode):**

```bash
# Reliable: hold the connection open for the whole piece (~45 s)
urctl run-script --raw --capture --collect-for 55 < ElegantDance.script
```

`--raw` because the file defines its own `def ElegantDance(): … end` and calls
it (don't let urctl wrap it again). The robot should be powered/RUNNING first
(`urctl bring-up`).

## Regenerate the .urp after editing the .script

```bash
python3 ../../scripts/urp_convert.py to-urp ElegantDance.script ElegantDance.urp \
    --name ElegantDance --installation ElegantDance
```

## Notes

- **No movej blend radius.** Because waypoints are randomized, two consecutive
  ones can occasionally land closer than a blend radius, which makes the
  controller abort with "blend radius too large for segment" — and since the
  seed differs every run, testing can't rule it out. Blends are therefore
  omitted; fluidity comes from fine waypoint spacing + high acceleration. To
  re-enable blends, add `r=...` to the `movej` and guarantee a minimum segment
  length (skip waypoints closer than the blend radius).
