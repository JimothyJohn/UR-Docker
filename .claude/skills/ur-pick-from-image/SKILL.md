---
name: ur-pick-from-image
description: >-
  Turn a photo of objects on the robot's work surface into a pick / pick-and-
  stack / pick-and-place program when there is NO calibrated camera->base
  transform. Use when asked to pick up, stack, sort, or place objects that are
  shown in an image/photo/camera frame (e.g. inputs/image.png) on a UR e-Series
  robot. Covers locating the objects, the assumptions you must state, and the
  scale-from-object-size + anchor-the-cluster trick that keeps poses reachable.
  Builds on ur-program-authoring for the program mechanics.
---

# Picking objects located in an image

The hard part isn't the motion (that's **ur-program-authoring**) — it's that one
image has **no metric camera→base calibration**, so you cannot recover true table
coordinates. Don't pretend you can. Instead, reproduce the objects' *relative*
arrangement faithfully and anchor it somewhere reachable, then say exactly what
you assumed. `programs/AppleStack` is the worked example.

## Steps

1. **Read the image.** Identify each object, its colour/label, and its
   approximate pixel centre. Note the frame size and the camera setup the user
   gave you (e.g. "top-down, to the right of the base").

2. **Get scale from a known object size**, not from guesswork. Measure an
   object's diameter in pixels; with a real size (an apple ≈ 0.075 m) the table
   scale is `real_size / pixel_size` (m/px). State it.

3. **Work in relative offsets.** Compute each object's offset from the cluster
   centroid in metres, then map the top-down image axes to the base frame —
   state the mapping you chose (e.g. image up → base +X, image right → base −Y).
   Absolute rotation/position of the cluster is unknowable from one image, so:

4. **Anchor the cluster at a reachable reference pose** (`p_ref` from a known-good
   config — see ur-program-authoring) and place each object as
   `pose_add(p_ref, p[dx, dy, drop, 0,0,0])`. The arrangement matches the photo
   and every pose stays inside the envelope on URSim. The per-object `[dx,dy]` and
   `p_ref` are the knobs to replace with real values once calibration exists.

5. **Generate the program** (pick: approach above → descend → close gripper →
   lift → carry → lower → open → retract). For a **stack**, place object *i* at
   `stack + n*object_diameter` in Z; biggest/most-stable on the bottom. Emit
   `textmsg` checkpoints per object and verify `*/done`.

## Always state up front

- It's URSim: no physical objects or gripper, so the program runs the motions +
  gripper-output toggles (a real pick on hardware once poses are taught/calibrated).
- The pixel→world mapping rests on stated assumptions (scale, axis mapping,
  anchor) — list them, expose them as the program's tunables, and **offer to
  swap in true coordinates** if the user gives camera height, its XY offset from
  the base, and the table Z.

Then hand off to **ur-program-authoring** for conventions, conversion, running,
and the blend-radius pitfall.
