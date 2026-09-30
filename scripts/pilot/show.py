"""Pick-and-shuffle show: two picks, each phase captioned with its wall-clock time for the timelapse."""
# ruff: noqa  — scratch script from the 2026-09-25 hardware session (scripts/pilot/README.md)

import os as _os

HERE = _os.path.dirname(_os.path.abspath(__file__))

import json
import subprocess
import sys
import time

S = HERE
OUT = sys.argv[1]
EV = OUT + "/events.json"
T0 = json.load(open(OUT + "/t0.json"))["t0"]
events = []


def say(text):
    events.append({"t": time.time() - T0, "text": text})
    json.dump(events, open(EV, "w"), indent=1)
    print(f"[{events[-1]['t']:6.1f}s] {text}", flush=True)


def run(*args):
    out = subprocess.run(
        ["uv", "run", "python", f"{S}/pick.py", *map(str, args)],
        capture_output=True,
        text=True,
        cwd=_os.path.dirname(_os.path.dirname(HERE)),  # the checkout root
    )
    print(out.stdout.strip().splitlines()[-2:] if out.stdout else out.stderr[-300:], flush=True)
    return out.stdout


def state():
    import urllib.request

    req = urllib.request.Request(
        "http://127.0.0.1:7621/api/robot/state", data=b"{}", headers={"Content-Type": "application/json"}
    )
    return json.load(urllib.request.urlopen(req, timeout=30)).get("safety_mode", "")


def move_high():
    import urllib.request

    req = urllib.request.Request(
        "http://127.0.0.1:7621/api/robot/move",
        data=json.dumps(
            {"pose": [-0.2325, 0.2776, 0.06, -2.9309, 0.151, -0.1047], "velocity": 0.08, "tcp": [0] * 6}
        ).encode(),
        headers={"Content-Type": "application/json"},
    )
    return json.load(urllib.request.urlopen(req, timeout=60)).get("ok")


def pick(x, y, label, place_xy):
    say(f"Looking for the {label} from 30 cm up: the RealSense sees nothing closer than 20 cm")
    move_high()
    time.sleep(1.0)
    run("plan", x, y)
    say("Hover: fingertips 40 mm above the block, gripper turned across its short side")
    run("hover")
    time.sleep(0.8)
    say("Down to the top edge, then a look through both webcams: block between the fingers?")
    run("descend", 0)
    time.sleep(1.2)
    if "NORMAL" not in state():
        say("Protective stop: unlocking and backing off")
        import urllib.request

        urllib.request.urlopen(
            urllib.request.Request(
                "http://127.0.0.1:7621/api/robot/bring_up",
                data=b"{}",
                headers={"Content-Type": "application/json"},
            ),
            timeout=60,
        ).read()
        time.sleep(2)
        run("lift", 60)
        return False
    say("15 mm into the grasp")
    run("descend", 15)
    time.sleep(0.5)
    say("Close: the Hand-E stops on the foam and reports an object")
    out = run("grip")
    held = "'object_detected': True" in out
    say("Held: lifting" if held else "Nothing held: opening and lifting")
    if not held:
        run("release")
    run("lift", 100)
    time.sleep(0.8)
    if held:
        say("Carrying it 9 cm to a clear spot")
        run("move", place_xy[0] - x, place_xy[1] - y)
        time.sleep(0.5)
        say("Setting it down at the same grasp height")
        run("descend", 15)
        say("Release")
        run("release")
        time.sleep(0.5)
        say("Up and away")
        run("lift", 100)
    return held


say(
    "UR3e + Robotiq Hand-E, RealSense D435 on the wrist, two room webcams. Hand-eye solved this afternoon without a touch mark."
)
time.sleep(2.5)
ok1 = pick(-0.150, 0.299, "block placed earlier", (-0.245, 0.255))
ok2 = pick(-0.182, 0.390, "block nearest the chair", (-0.195, 0.221))
say(
    f"Done: {int(ok1) + int(ok2)} of 2 picks held. Every move was one safety-checked movel through the cockpit."
)
time.sleep(3.0)
open(OUT + "/STOP", "w").write("stop")
