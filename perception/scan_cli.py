"""``perception scan`` / ``locate`` / ``touch`` / ``latency-fit`` / ``part-info``
/ ``make-box`` — the monocular-scan commands (`docs/mono-scan.md`).

Kept out of :mod:`perception.cli` so the argparse tree there stays readable;
:func:`add_scan_commands` registers the subparsers and :func:`run_scan_command`
dispatches them (returns None for a command it doesn't own).

    perception scan --fake                                   # synthetic sweep + locate
    perception --cell ur3 scan --delta 0.15 0 0 --parts parts/block_50x30x30.stl
    perception locate captures/sweeps/scan-001 --parts parts/*.stl
    perception --cell ur3 touch --set table --label 1        # probe on the table, 3x
    perception --cell ur3 touch --set plate --label A        # probe in dimple A, B, C
    perception --cell ur3 table-from-depth                   # table plane from the depth (no probe)
    perception --cell ur3 latency-fit captures/sweeps/scan-001 --objects --save   # no plate
    perception --cell ur3 latency-fit captures/sweeps/plate-001                   # ChArUco plate
    perception part-info parts/block_50x30x30.stl
    perception make-box 0.05 0.03 0.03 --out parts/block_50x30x30.stl
"""

from __future__ import annotations

import glob
import json
import os
import sys
from pathlib import Path

from .sweep import (
    DEFAULT_DELTA_M,
    DEFAULT_SWEEP_ACCELERATION,
    DEFAULT_SWEEP_ROOT,
    DEFAULT_SWEEP_VELOCITY,
    ENV_CAM_LATENCY,
)
from .tableplane import ENV_TABLE_Z
from .touch import DEFAULT_PROBE_TCP, PLATE_LABELS, SETS, TABLE_LABELS

DEFAULT_PARTS_GLOB = "parts/*.stl"
DEFAULT_LATENCY_FILE_DIR = "captures/calibration"

SCAN_COMMANDS = ("scan", "locate", "touch", "latency-fit", "table-from-depth", "part-info", "make-box")


def _emit(result: dict) -> int:
    print(json.dumps(result, indent=2, default=str))
    return 0 if result.get("ok", True) else 1


def add_scan_commands(sub, *, add_camera_args, add_robot_args) -> None:
    sc = sub.add_parser("scan", help="one sweep move with frames + poses recorded, then locate the parts")
    add_camera_args(sc)
    add_robot_args(sc)
    sc.add_argument("--name", default="scan", help="sweep name (a numbered directory under --out)")
    sc.add_argument("--out", default=DEFAULT_SWEEP_ROOT, help=f"sweep root (default: {DEFAULT_SWEEP_ROOT}/)")
    sc.add_argument(
        "--delta",
        type=float,
        nargs=3,
        metavar=("DX", "DY", "DZ"),
        default=list(DEFAULT_DELTA_M),
        help="base-frame sweep in metres (default: 0.15 0 0)",
    )
    sc.add_argument("--velocity", type=float, default=DEFAULT_SWEEP_VELOCITY, help="m/s (default 0.15)")
    sc.add_argument("--accel", type=float, default=DEFAULT_SWEEP_ACCELERATION, help="m/s^2 (default 0.5)")
    sc.add_argument(
        "--latency",
        type=float,
        default=None,
        help=f"camera latency s (default: ${ENV_CAM_LATENCY}, the saved fit, or 0)",
    )
    sc.add_argument(
        "--table-z",
        type=float,
        default=None,
        help=f"level table at this base-frame z (default: ${ENV_TABLE_Z} or the touched plane)",
    )
    sc.add_argument("--parts", nargs="*", default=None, help=f"part STLs (default: {DEFAULT_PARTS_GLOB})")
    sc.add_argument("--no-locate", action="store_true", help="capture only")
    sc.add_argument(
        "--fake-latency", type=float, default=0.04, help="the synthetic camera's latency (with --fake)"
    )
    sc.add_argument(
        "--stream",
        choices=("color", "ir"),
        default="color",
        help="RealSense imager: color (rolling shutter) or ir (left imager, global shutter, emitter off)",
    )
    sc.add_argument("--no-depth-check", action="store_true", help="skip grading the result against the depth")
    _add_locate_args(sc)

    lo = sub.add_parser("locate", help="re-run the locator on a saved sweep")
    lo.add_argument("sweep_dir")
    lo.add_argument("--table-z", type=float, default=None)
    lo.add_argument("--parts", nargs="*", default=None)
    _add_locate_args(lo)

    to = sub.add_parser(
        "touch", help="record a probe touch (table 1/2/3, plate A/B/C) from the live flange pose"
    )
    add_robot_args(to)
    to.add_argument("--set", dest="which", choices=SETS, default=None)
    to.add_argument(
        "--label", default=None, help=f"table: {'/'.join(TABLE_LABELS)}; plate: {'/'.join(PLATE_LABELS)}"
    )
    to.add_argument(
        "--probe-tcp",
        type=float,
        nargs=6,
        default=None,
        metavar="V",
        help=f"flange->tip UR pose (default {DEFAULT_PROBE_TCP})",
    )
    to.add_argument(
        "--file", default=None, help="touch file (default: captures/calibration/touch_<cell>.json)"
    )
    to.add_argument(
        "--show", action="store_true", help="print the file and the derived plane/plate, record nothing"
    )
    to.add_argument("--clear", action="store_true", help="forget every touch in the file")

    lf = sub.add_parser(
        "latency-fit",
        help="fit the camera latency from a sweep: over the parts (--objects) or the ChArUco plate",
    )
    lf.add_argument("sweep_dir")
    lf.add_argument(
        "--objects", action="store_true", help="no plate: maximise the box-fit IoU over the parts"
    )
    lf.add_argument("--parts", nargs="*", default=None)
    lf.add_argument("--table-z", type=float, default=None)
    lf.add_argument(
        "--board", default=None, help="board.json (default: hardware/charuco-board/out/board.json)"
    )
    lf.add_argument("--touch-file", default=None)
    lf.add_argument(
        "--save", action="store_true", help="write the result to captures/calibration/latency_<cell>.json"
    )

    td = sub.add_parser(
        "table-from-depth", help="table plane from one RGB-D frame of the empty surface (no probe)"
    )
    add_camera_args(td)
    add_robot_args(td)
    td.add_argument("--no-save", action="store_true", help="print only")
    td.add_argument(
        "--file", default=None, help="where to save (default: captures/calibration/table_<cell>.json)"
    )
    td.add_argument(
        "--warmup", type=int, default=15, help="frames to discard for auto-exposure (default: 15)"
    )

    pi = sub.add_parser("part-info", help="oriented bounding box + resting poses of an STL")
    pi.add_argument("stl", nargs="+")

    mb = sub.add_parser("make-box", help="write a box STL (metres) — e.g. the 50x30x30 test block")
    mb.add_argument("lx", type=float)
    mb.add_argument("ly", type=float)
    mb.add_argument("lz", type=float)
    mb.add_argument("--out", required=True)


def _add_locate_args(p) -> None:
    p.add_argument("--min-area", type=int, default=150, help="drop blobs under this many px (default 150)")
    p.add_argument("--threshold", type=int, default=None, help="fixed gray threshold (default: Otsu)")
    p.add_argument("--min-views", type=int, default=3)
    p.add_argument("--no-refine", action="store_true", help="skip the model fit (raw parallax estimate)")
    p.add_argument("--moving-only", action="store_true", help="use only frames taken while the arm moved")


def _library(paths):
    from .partlib import PartLibrary

    if paths is None:
        paths = sorted(glob.glob(DEFAULT_PARTS_GLOB))
    return PartLibrary.from_paths(paths) if paths else None


def _plane(table_z, *, fallback=None):
    from .tableplane import Plane

    if table_z is not None:
        return Plane.from_z(table_z, source="--table-z")
    p = Plane.from_env()
    if p is not None:
        return p
    from .touch import TouchSet, default_touch_path

    path = default_touch_path()
    if os.path.isfile(path):
        tp = TouchSet.load(path).table_plane()
        if tp is not None:
            return tp
    return fallback


def _latency_path() -> str:
    cell = os.environ.get("UR_CELL", "").strip()
    return os.path.join(DEFAULT_LATENCY_FILE_DIR, f"latency_{cell}.json" if cell else "latency.json")


def _latency(explicit) -> float:
    from .sweep import latency_from_env

    if explicit is not None:
        return float(explicit)
    env = latency_from_env()
    if env:
        return env
    p = _latency_path()
    if os.path.isfile(p):
        return float(json.loads(Path(p).read_text()).get("latency_s", 0.0))
    return 0.0


def _locate(sweep, args, library):
    from .locate2d import locate_objects

    pairs = sweep.frame_pairs(moving_only=args.moving_only)
    objs = locate_objects(
        pairs,
        sweep.intrinsics,
        sweep.plane,
        library=library,
        min_area_px=args.min_area,
        threshold=args.threshold,
        min_views=args.min_views,
        refine=not args.no_refine,
    )
    return {"frames_used": len(pairs), "objects": [o.as_dict() for o in objs]}


def _next_dir(root: str, name: str) -> Path:
    r = Path(root)
    r.mkdir(parents=True, exist_ok=True)
    n = 1 + max(
        (
            int(p.name.rsplit("-", 1)[1])
            for p in r.glob(f"{name}-[0-9][0-9][0-9]")
            if p.name.rsplit("-", 1)[1].isdigit()
        ),
        default=0,
    )
    return r / f"{name}-{n:03d}"


def _scan(args, *, camera_from_args, robot_from_args, config) -> int:
    from .sweep import FakeRig, Sweep, run_sweep

    library = _library(args.parts)
    fake = bool(getattr(args, "fake", False)) or os.environ.get("PERCEPTION_FAKE", "").lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    latency = _latency(args.latency)
    depth_frame = None
    if fake:
        rig = FakeRig(latency_s=args.fake_latency)
        plane = _plane(args.table_z, fallback=rig.plane)
        rig.plane = plane if plane.source != "fake" else rig.plane
        sweep: Sweep = rig.run(
            delta=args.delta,
            velocity=args.velocity,
            acceleration=args.accel,
            latency_s=latency if args.latency is not None else None,
        )
    else:
        from .handeye import HandEye
        from .monocam import RealSenseMono
        from .posestream import PoseRecorder

        plane = _plane(args.table_z)
        if plane is None:
            return _emit(
                {
                    "ok": False,
                    "error": f"no table plane: pass --table-z, set {ENV_TABLE_Z}, "
                    "or record three `perception touch --set table` points",
                }
            )
        robot = robot_from_args(args)
        if robot is None:
            return _emit({"ok": False, "error": "the scan needs the robot (drop --no-robot)"})
        source = RealSenseMono(camera_from_args(args, config), stream=args.stream)
        source.open()
        try:
            robot.attach_camera(source.describe())
            handeye = robot.handeye if isinstance(robot.handeye, HandEye) else HandEye.from_env()
            offset = robot.tcp_offset()
            if offset is None and not robot.dry_run:
                return _emit({"ok": False, "error": "could not read the active TCP offset from the robot"})
            recorder = PoseRecorder(robot.config, tcp_offset=offset or [0.0] * 6)
            leg = handeye.flange_to_depth if source.frame_kind == "depth" else handeye.flange_to_color
            sweep = run_sweep(
                source,
                recorder,
                robot,
                intrinsics=source.intrinsics,
                flange_to_color=leg,
                delta=args.delta,
                velocity=args.velocity,
                acceleration=args.accel,
                latency_s=latency,
                plane=plane,
            )
            sweep.meta["handeye"] = handeye.as_dict()
            sweep.meta["stream"] = args.stream
            if source.last_rgbd is not None and not args.no_depth_check:
                from urctl.pose import Transform

                rgbd, t_rgbd = source.last_rgbd
                pose = sweep.flange_at(t_rgbd - latency)
                if pose is not None:
                    depth_frame = (rgbd, Transform.from_pose(pose).compose(handeye.flange_to_color))
        finally:
            source.close()
    out = _next_dir(args.out, args.name)
    sweep.save(out)
    result = {"ok": True, "sweep_dir": str(out), "sweep": sweep.summary(), "latency_s": latency}
    if not args.no_locate:
        located = _locate(sweep, args, library)
        result.update(located)
        if depth_frame is not None and located["objects"]:
            from .depthcheck import check_objects

            checks = check_objects(located["objects"], depth_frame[0], depth_frame[1], sweep.plane)
            for o, c in zip(result["objects"], checks, strict=True):
                o["depth_check"] = c
    return _emit(result)


def _table_from_depth(args, *, camera_from_args, robot_from_args, config) -> int:
    from urctl.pose import Transform

    from .tableplane import default_table_path, plane_from_depth

    robot = robot_from_args(args)
    if robot is None:
        return _emit({"ok": False, "error": "table-from-depth needs the robot for the flange pose"})
    camera = camera_from_args(args, config)
    camera.open()
    try:
        robot.attach_camera(camera.describe())
        frame = None
        for _ in range(max(0, args.warmup) + 1):
            frame = camera.read()
        fp = robot.flange_pose()
    finally:
        camera.close()
    if not fp.get("ok") or not fp.get("flange"):
        return _emit({"ok": False, "error": fp.get("error") or "could not read the flange pose", "robot": fp})
    leg = robot.handeye.flange_to_color if getattr(frame, "aligned", True) else robot.handeye.flange_to_depth
    t_base_cam = Transform.from_pose(fp["flange"]).compose(leg)
    try:
        plane, stats = plane_from_depth(frame, t_base_cam)
    except ValueError as exc:
        return _emit({"ok": False, "error": str(exc)})
    result = {
        "ok": True,
        "plane": plane.as_dict(),
        "stats": stats,
        "flange": fp["flange"],
        "handeye": robot.handeye.source,
    }
    if stats["tilt_from_base_z_deg"] > 10.0:
        result["warning"] = (
            f"surface is {stats['tilt_from_base_z_deg']:.1f} deg off the base Z axis — "
            "is the camera looking at the table?"
        )
    if not args.no_save:
        path = args.file or default_table_path()
        plane.save(path)
        result["saved"] = path
    return _emit(result)


def _touch(args, *, robot_from_args) -> int:
    from .touch import TouchSet, default_touch_path

    path = args.file or default_touch_path()
    ts = TouchSet.load_or_new(path)
    if args.probe_tcp is not None:
        ts.probe_tcp = [float(v) for v in args.probe_tcp]
    if args.clear:
        ts.table.clear()
        ts.plate.clear()
        ts.save(path)
    if args.show or args.clear:
        return _emit({"ok": True, "file": path, **ts.as_dict()})
    if not args.which or not args.label:
        return _emit({"ok": False, "error": "need --set table|plate and --label (or --show)", "file": path})
    robot = robot_from_args(args)
    if robot is None:
        return _emit({"ok": False, "error": "touch needs the robot"})
    fp = robot.flange_pose()
    if not fp.get("ok") or not fp.get("flange"):
        return _emit({"ok": False, "error": fp.get("error") or "could not read the flange pose", "robot": fp})
    try:
        tip = ts.record(args.which, args.label, fp["flange"])
    except ValueError as exc:
        return _emit({"ok": False, "error": str(exc)})
    ts.save(path)
    return _emit(
        {
            "ok": True,
            "file": path,
            "recorded": {args.which: {args.label.upper(): tip}},
            "flange": fp["flange"],
            **ts.as_dict(),
        }
    )


def _latency_fit(args) -> int:
    from urctl.pose import Transform

    from .charuco import DEFAULT_BOARD_JSON, BoardSpec, CharucoPnP, measured_camera_positions
    from .latency import fit_latency, fit_latency_by_objects
    from .sweep import Sweep
    from .touch import TouchSet, default_touch_path

    sweep = Sweep.load(args.sweep_dir)
    if not sweep.trajectory:
        return _emit({"ok": False, "error": "this sweep has no saved trajectory; capture it again"})
    if args.objects:
        plane = _plane(args.table_z, fallback=sweep.plane)
        if plane is None:
            return _emit({"ok": False, "error": "no table plane (pass --table-z)"})
        sweep.plane = plane
        library = _library(args.parts)
        if library is None:
            return _emit({"ok": False, "error": "--objects needs the part library (--parts or parts/*.stl)"})
        try:
            fit = fit_latency_by_objects(sweep, library)
        except ValueError as exc:
            return _emit({"ok": False, "error": str(exc)})
        return _emit(_save_latency({"ok": True, "sweep_dir": args.sweep_dir, **fit}, args.save))
    ts_path = args.touch_file or default_touch_path()
    if not os.path.isfile(ts_path):
        return _emit(
            {"ok": False, "error": f"no touch file at {ts_path}: record the plate dimples A/B/C first"}
        )
    plate = TouchSet.load(ts_path).plate_transform()
    if plate is None:
        return _emit({"ok": False, "error": "the touch file has no complete plate (A, B, C)"})
    spec = BoardSpec.load(args.board or DEFAULT_BOARD_JSON)
    pnp = CharucoPnP(spec, sweep.intrinsics)
    measured = measured_camera_positions(sweep.frames, pnp, plate)
    if len(measured) < 3:
        return _emit({"ok": False, "error": f"the board was detected in only {len(measured)} frames"})
    f2c = sweep.flange_to_color

    def cam_at(t):
        pose = sweep.flange_at(t)
        return None if pose is None else Transform.from_pose(pose).compose(f2c).translation

    fit = fit_latency(measured, cam_at)
    fit["board_frames"] = len(measured)
    fit["method"] = "charuco"
    return _emit(_save_latency({"ok": True, "sweep_dir": args.sweep_dir, **fit}, args.save))


def _save_latency(result: dict, save: bool) -> dict:
    if save:
        p = Path(_latency_path())
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(result, indent=2))
        result["saved"] = str(p)
    return result


def run_scan_command(args, *, camera_from_args, robot_from_args, config_from_args) -> int | None:
    if args.cmd not in SCAN_COMMANDS:
        return None
    if args.cmd == "scan":
        return _scan(
            args,
            camera_from_args=camera_from_args,
            robot_from_args=robot_from_args,
            config=config_from_args(args),
        )
    if args.cmd == "locate":
        from .sweep import Sweep

        sweep = Sweep.load(args.sweep_dir)
        plane = _plane(args.table_z, fallback=sweep.plane)
        if plane is None:
            return _emit({"ok": False, "error": "no table plane for this sweep (pass --table-z)"})
        sweep.plane = plane
        return _emit(
            {
                "ok": True,
                "sweep_dir": args.sweep_dir,
                "plane": plane.as_dict(),
                **_locate(sweep, args, _library(args.parts)),
            }
        )
    if args.cmd == "touch":
        return _touch(args, robot_from_args=robot_from_args)
    if args.cmd == "table-from-depth":
        return _table_from_depth(
            args,
            camera_from_args=camera_from_args,
            robot_from_args=robot_from_args,
            config=config_from_args(args),
        )
    if args.cmd == "latency-fit":
        return _latency_fit(args)
    if args.cmd == "part-info":
        from .partlib import Part

        try:
            return _emit({"ok": True, "parts": [Part.from_stl(p).as_dict() for p in args.stl]})
        except (OSError, ValueError) as exc:
            return _emit({"ok": False, "error": str(exc)})
    if args.cmd == "make-box":
        from .partlib import Part, write_box_stl

        p = write_box_stl(args.out, args.lx, args.ly, args.lz, name=Path(args.out).stem)
        return _emit({"ok": True, "path": str(p), "part": Part.from_stl(p).as_dict()})
    return None


def _stderr(msg: str) -> None:
    print(msg, file=sys.stderr)
