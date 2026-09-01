# Program authoring best practices

Conventions for any program we author for a UR e-Series controller in this
repo — `.urp` node trees (via `urctl/urp_builder.py`), guided builds
(`urctl/guided.py`), and joint-space trajectories (`Robot.move_trajectory`).
These are *authoring* rules: they make the program readable on the pendant and
smooth to run. They are not enforced by the loader — PolyScope will happily load
a program that ignores all of them — so they live here as discipline.

Companion to the procedural how-to in
`.claude/skills/ur-program-authoring/SKILL.md` and the URP schema notes in
`CLAUDE.md`.

## 1. Name every waypoint by what it does

A waypoint name is shown to the operator in the Program tree. `TCP_1`, `TCP_2`
tells them nothing; `PickApproach`, `Grasp`, `Lift` tells them everything.

- **Name by function, not by index.** `Home`, `PickApproach`, `Grasp`, `Lift`,
  `PlaceApproach`, `Place`, `Retract` — the name says where the robot is and why.
- **Names are unique across the program** — one name, one meaning.
- **Reuse a name only for a genuinely identical position.** If the program
  returns to a point it already defined (back to `Home`, back to `PickApproach`),
  give the later waypoint the *same* name. Same name ⇒ same physical point; it
  signals the repeat to a reader and to anyone editing the teach point (move the
  point once, every reuse follows). Do **not** invent `Home2` / `BackHome` for a
  pose that equals `Home`.

Tooling: `Waypoint(name, q=[...])` takes the name verbatim — author with explicit
functional names. The guided REPL (`urctl guided`) derives the name from the
step description (`movej … ; pick approach` → `PickApproach`) and **reuses an
earlier waypoint's name when a step returns to the same physical pose**, so the
duplicate-position rule is applied automatically.

## 2. Keep moves under one Move node unless the speed must change

In PolyScope a **Move** node (`MoveJ` or `MoveL`) carries the speed/acceleration,
and holds one *or many* waypoint children that all share those parameters.

- **Group consecutive same-type waypoints into a single Move node.** A pick →
  lift → traverse → place path is one `MoveL` (or `MoveJ`) with six waypoints,
  not six Move nodes with one waypoint each.
- **Open a new Move node only when a segment needs a dramatically different
  speed or acceleration** — e.g. a slow, precise final approach to a grasp after
  a fast traverse. Speed/accel are per-Move, so a real speed change is the one
  legitimate reason to split.
- **Don't mix motion types in one node.** `MoveJ` and `MoveL` are different Move
  nodes by definition; switch nodes when you switch interpolation.

Tooling: `UrpProgram.movej(*waypoints, speed=, acceleration=)` and `.movel(...)`
already accept multiple waypoints and emit a single `<Move>`. Prefer

```python
prog.movel(
    Waypoint("PickApproach", q=...),
    Waypoint("Grasp", q=...),
    Waypoint("Lift", q=...),
)
```

over one `movel()` call per waypoint. Use `group=True` on successive calls to
merge them into a preceding compatible Move node. **Note the tension with rule 4
(comment every line):** a Comment node between two moves prevents them from
grouping, so you cannot both annotate every move *and* coalesce them. Grouping
wins when a run of moves shares one obvious purpose (give the group a single
comment); per-line comments win when each step needs its own note. The guided
session chooses per-line comments, so its moves are intentionally *not* grouped.

## 3. Blend every waypoint except stopping points

A blend radius rounds the corner at a waypoint so the robot carries momentum
through it instead of decelerating to zero and re-accelerating. Smoother, faster,
less wear.

- **Give intermediate waypoints a blend radius.** Anything the robot passes
  *through* on the way somewhere should blend.
- **A stopping point gets no blend (`r = 0`).** A waypoint is a stopping point
  when the robot must be exactly there:
  - the **last** waypoint of a path (a non-zero blend on the final `movej`/`movel`
    is a URScript error — the controller aborts),
  - any point immediately followed by an **I/O or action** that depends on exact
    pose (close gripper at `Grasp`, open at `Place`),
  - any point where precision matters more than smoothness.
- **Keep the radius smaller than half the shortest adjacent segment.** If two
  waypoints are closer than the blend radius the controller aborts with *"blend
  radius too large for segment."* For randomized/variable spacing, either omit
  blends or guarantee a minimum segment length. See the movej blend-radius
  pitfall in `.claude/skills/ur-program-authoring/SKILL.md`.

Tooling status:

- **URScript / trajectories — supported.** `Robot.move_trajectory(waypoints,
  blend_radius=...)` (tool `ur_move_trajectory`) blends every segment and
  automatically drops the blend on the final waypoint.
- **`.urp` node trees — supported.** `Waypoint(name, q=..., blend_radius=0.05)`
  emits `<motionParameters blendRadius="0.05"/>`; omit `blend_radius` (the
  default) for a stopping point. The attribute name/shape was read from the
  controller's own `WaypointNodeConversionStrategy` (not guessed) and is
  load-tested against URSim
  (`test_integration_ursim.py::TestUrpBuilderLoading::test_blended_waypoints_urp_loads`).
  Per the movej blend-radius pitfall, leave the final waypoint of a Move
  unblended. The guided session does not set blends (it can't tell live which
  points are stops) — add them when you author or post-process the `.urp`.

## 4. Comment every line

An operator *will* open this program to touch it up — change a position, retime a
move, add a step. Make that legible.

- **Every operation gets a Comment node** describing what it does in plain terms
  (`approach the part`, `close gripper`, `lift clear`), placed immediately before
  the node it explains.
- **Describe intent, not the obvious.** `lower onto part` beats `MoveL down`.
- See rule 2 for the grouping trade-off: comments and grouping compete at the
  tree level, and for guided builds comments win.

Tooling: the guided session emits a Comment before every recorded Move/Set, using
the step's description — so always pass a `; description` on guided commands. When
hand-authoring, call `prog.comment(...)` before each node.

## 5. Offer freedrive reteach when teaching positions

A scripted target is rarely the exact spot. When teaching a position, move to the
rough target, then let the operator hand-guide it to the precise pose and confirm
— and build later positions off the *adjusted* pose, not the original guess.

- **Flow:** move to target → enter freedrive → "hand-guide into place" → the
  operator taps **Yes** (record) / **No** or **Cancel** (skip) on the pendant →
  leave freedrive → record the achieved pose. The dialog is a three-button
  Yes/No/Cancel popup, not an OK box.
- **Relative steps chain off the new pose.** Because a relative move is computed
  from the live TCP, the next step starts wherever the operator left the arm.

Tooling: `urctl guided … --freedrive` turns this on; each move then calls
`Robot.reteach_in_freedrive(...)` (freedrive wrapped around a pendant
Yes/No/Cancel dialog) and records the hand-guided pose. No/Cancel skips recording
that step.

## Worked example — pick and place

Applying these rules to a pick-and-place (the `programs/PickPlace` shape):

| Waypoint        | Move node            | Blend     | Why                                    |
| --------------- | -------------------- | --------- | -------------------------------------- |
| `PickApproach`  | MoveL (traverse)     | blend     | passing through, above the part        |
| `Grasp`         | MoveL (traverse)     | **stop**  | exact pose, gripper closes here        |
| `Lift`          | MoveL (traverse)     | blend     | passing through on the way up          |
| `PlaceApproach` | MoveL (traverse)     | blend     | passing through, above the target      |
| `Place`         | MoveL (traverse)     | **stop**  | exact pose, gripper opens here         |
| `Retract`       | MoveL (traverse)     | **stop**  | last waypoint of the path              |

One `MoveL` node holds all six (same speed); blends on the three fly-through
points, stops at `Grasp`, `Place`, and the final `Retract`. If the descent to
`Grasp` needs to be slower than the traverse, *that* is when you split `Grasp`
(and its approach) into its own slower `MoveL` node — rule 2's exception.
