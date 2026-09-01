# InspectionBot

A guided, multi-station quality-inspection routine for Universal Robots
e-Series cobots, written in the same "teach by prompt" spirit as
[HelpfulBot](https://github.com/nickarmenta/HelpfulBot). Instead of writing
motion code, the operator walks through PolyScope popups and freedrive moves
to teach:

1. Wiring — which digital input means "part present", which digital outputs
   to drive on PASS / FAIL verdicts.
2. Timing — how long to dwell at each waypoint so a vision system has time
   to capture.
3. A safe HOME pose.
4. Up to eight inspection waypoints, added one at a time with an "another?"
   loop.

## Cycle

1. Wait for the part-present input to go high.
2. Move to home, then tour every taught waypoint with the configured dwell.
3. After each waypoint, prompt the operator (or an external vision agent)
   for a pass/fail verdict via popup.
4. Pulse the PASS or FAIL digital output high for 0.5 s based on whether
   every station passed.
5. Return home, wait for the operator to clear the fixture, re-arm.

## Files

| File                        | Purpose                                                   |
| --------------------------- | --------------------------------------------------------- |
| `InspectionBot.script`      | Human-readable URScript source (edit this).               |
| `InspectionBot.urp`         | Generated PolyScope program — load this from the pendant. |
| `InspectionBot.installation`| Paired installation file PolyScope requires.              |

Regenerate the `.urp` after editing the `.script`:

```bash
python3 ../../scripts/urp_convert.py to-urp \
  InspectionBot.script InspectionBot.urp \
  --installation InspectionBot \
  --directory /programs/InspectionBot
```

## Loading into URSim

Copy all three files into the simulator's `/ursim/programs/` directory
(`docker cp`), then either tap **File → Load Program** in PolyScope or use
the Dashboard server:

```bash
printf 'load InspectionBot.urp\nquit\n' | nc -q1 localhost 29999
```

PolyScope must be in **Remote Control** mode (toggle in the top-right of the
pendant UI) for `play` to be accepted over Dashboard.

## Caveats

- Waypoint storage is a fixed-size 8-pose array because URScript doesn't
  resize arrays. Raise the limit by widening the literal in
  `InspectionBot.script` if you need more stations.
- The verdict step is operator-driven (`request_boolean_from_primary_client`).
  For a real vision integration, replace those calls with reads from a
  result register the vision system writes via RTDE or Modbus.
- Generated `.urp` carries identity kinematics with the checksum check
  disabled; fine for URSim. On real hardware, save the program once in
  PolyScope after loading to bake in the actual robot's calibration.
