# AppleStack

Picks up the three apples in `inputs/image.png` and stacks them at one spot.

## What's in the picture

A near top-down shot from above the mounting surface, to the right of the base.
Three apples: two **red** (A upper-middle, B left) and one **green** (C, right
of centre). Approximate pixel centres in the ~320×176 frame: A(150,53),
B(109,81), C(192,92).

## How positions were derived

There is no calibrated camera→base transform, so absolute table coordinates
can't be recovered from a single image. Instead:

- The apple **diameter** (~45 px) against a real ~0.075 m apple sets the table
  scale (~0.00167 m/px).
- Each apple's offset from the cluster centroid is converted to metres and the
  top-down image axes are mapped to the base frame (image up → base +X, image
  right → base −Y).
- The cluster is **anchored at a reachable reference pose** (`p_ref`) and the
  apples placed by those image-derived *relative* offsets — so the arrangement
  matches the photo and every pose is reachable on URSim.

On real hardware, set `p_ref` (and/or the per-apple offsets) from your
calibrated camera or taught poses; the relative layout is already correct.

## Gripper / simulation note

The gripper is modelled as **standard digital output 0** (True = closed). URSim
has no physical gripper or apples, so this runs the full motion + I/O sequence;
wire output 0 to your gripper (or a URCap) and the same program picks and stacks
for real. Emits `applestack/` `textmsg` checkpoints
(`start`, `grasped_*`, `stacked_*`, `done`).

## Run it

```bash
urctl bring-up
# Reliable host run (works in Local or Remote mode), ~30 s:
urctl run-script --raw --capture --collect-for 45 < AppleStack.script
```

Or from PolyScope: `docker cp AppleStack.urp AppleStack.installation` into
`/ursim/programs/`, then Load Program → AppleStack.urp → Play.

## Regenerate the .urp after editing the .script

```bash
python3 ../../scripts/urp_convert.py to-urp AppleStack.script AppleStack.urp \
    --name AppleStack --installation AppleStack
```
