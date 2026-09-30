# TODO

## Needs the cell (UR3e + Hand-E; demo Monday 2026-09-28)

- 2026-09-25 — The survey sees 3 of 4 blocks from the three overlook poses: add a `--survey-pose` from the far side of the pile. The second look sometimes shifts a centre 20–30 mm and closes on nothing — 0.4 m looks like the edge of the D435's depth on foam; try surveying from 0.3 m.
- 2026-09-27 — Nick: "survey the entire FOV". Found at 18:45: a dry-run survey found **0** of the 2 blocks in view. In the evening light the foam reads 55–100 on its darkest channel and `WHITE_MIN` is a fixed 180. At 50 both blocks are found, at 60 neither — the floor is 20–40. The Hand-E also covers the lower-right quarter of the colour frame, and blobs touching the frame edge are dropped. Open: light the cell for the demo, make the white threshold follow the frame's exposure, or did "entire FOV" mean something else?
- 2026-09-25 — Mac Studio + D435: `--rs-lean` streams (first open errors once, the back-off re-open holds). Open: why the *first* open still loses — shave one more reset, or accept the one retry. Ruled out: software drift, headless-vs-desktop, a fresh daemon, the webcams, Spotify. The Mac is a dev box, not a camera host; Windows laptop (WSL2, verified 09-23) and Jetson are.
- 2026-09-12 — The native Windows path (`scripts/setup-windows.ps1`, `scripts/cockpit.ps1`, `REALSENSE_LIB` at the SDK's default `bin\x64\realsense2.dll`) has not been run on a Windows box. First run on the work laptop is the verification; paste the doctor output (`uv run perceptronics --cell ur20 doctor --stream --json`) if anything fails.

- 2026-09-28 — **PS5 pick kit 0.5.0, first time on the cell** (all verified only in CI/URSim so far): (1) install `urcap/dist/realsense-pilot-ps5-0.5.0.urcap`, look at both screens on the pendant (`urcap/realsense-pilot-ps5/screens/` are harness renders); (2) teach one pick area by three fingertip touches — the tilt it reports should be < 1°; (3) one Pick node, Robotiq gripper mode: does it activate/open/close the Hand-E and read OBJ right (never run on hardware); (4) deploy the camera computer to real Pi-class hardware with `scripts/deploy-pi.sh` and read the doctor.
- 2026-09-28 — The finger-room check uses pickplan's Hand-E finger zone (14 × 32 mm beside the open jaws). Measure the real pads once; a wrong zone either blocks good picks or lets a finger land on a neighbour.

## PolyScope X URCap (2026-09-26, `urcap/DEVELOPING.md`)

- Installed in the sim and verified headless (node loads, feed/hover/click against `make urcap-cockpit` on :7622). **Still owed:** a cockpit with a robot behind it — restart your live one with `--cors http://localhost:8000` (`scripts/cockpit-mac.sh` + that flag), set the node's cockpit URL to `http://localhost:7621`, click a block: base point + approach + reach, then **Move (cockpit)** on the UR3e. **Move (PolyScope)** (IK + auto-move; is `Pose.orientation` a rotation vector?) needs the sim's arm powered + Remote, or the real PolyScope X cell.
- Later, on the robot: a backend-container packaging of the cockpit (`containers:` + `devices: video` + `services: urcontrol-primary`) so the pendant needs no external host.

## Code (no robot needed)

- 2026-09-26 — `perceptronics calibrate` (orbit hand-eye) is built and tested on the fake cell; **run it once on the UR3e** (`--dry-run` first), then delete `scripts/pilot/orbit_cal*.py` + `wiggle.py`. `record.py`/`show.py`/`assemble.py` stay as the timelapse tooling. `place.py`'s lesson carries: a place spot needs the same clearance check as a pick.
- 2026-09-28 — **Pick node teach screen: redesign to ≤ 3 simple stages** with minimal clicks and input and plenty of visual feedback (Nick's UX rule, see Decisions). 0.4.0's seven-button part-size row is the opposite; don't ship it as is.

## Open questions for Nick

- 2026-09-02 — The on-controller GPU is a **Jetson** (decided 2026-09-25); which JetPack / L4T base to pin in `Dockerfile.perceptronics` for CUDA + SAM is still open.
- 2026-09-27 — First PolyScope 5 install of `realsense-pilot-ps5-0.1.0.urcap`: the first try on the UR3e was refused (no `Bundle-Category: URCap`, fixed 1892a21). The fixed build is on the "URE MODELS" stick (sha256 6e0318817a1e…): Settings → System → URCaps → + → Restart. Past the file checks the installer still runs a compatibility check and a trial OSGi install, and the node has never rendered on a PolyScope 5 pendant.
- 2026-09-27 — PS5 **RealSense Pick** node 0.3.0 (auto survey = first look from where the arm is, then halfway toward the block, centred; every stage logged; popup on failure) is on the "URE MODELS" stick. Its URScript has **not run on a controller yet**: did it install and pick? The first run's trace is in `~/Library/Logs/perceptronics/pick-server.log` (or `GET http://192.168.3.10:7631/api/pick/log`).
- 2026-09-27 — Delete the stale pre-rebase `feature/urcap5-pick-node` on the fork? Everything in it is in `refactor/prune-2026-09-25` (4df4259).

- 2026-09-28 — One **RealSense Pick node run = one part**: it goes to the picture points in turn until one shows a part, picks the first in the chosen order, runs its children (the after-pick routine), and the next run goes straight to the close look over the next part it already saw (no trip back to the picture point) — faster cycle. OK, or should one run clear every location?

- 2026-09-28 — Pick PC (`deploy/pi/`, never run on a board yet): bookworm's systemd 252 ignores the unit's restart back-off (trixie only) — OK, or target trixie? Leave the hand-eye line out of the PC's `cell.env` by default (else a fresh on-PC calibration loses to it)?
- 2026-09-28 — The librealsense build fetches nlohmann/json, fastcdr, yaml-cpp and sqlite at configure time (first install needs internet). Turn off rosbag support to cut that, or ship a prebuilt `.deb` per board?
- 2026-09-28 — Bracket tool bolts: the BOM lists M6 × 16 low-head (plate 6 mm + Hand-E's M6 × 10 → the README's "+8 mm" gives only 2 mm more engagement vs the flange's 8 mm limit). Measure before ordering.
- 2026-09-29 — **USB auto-install, first time on a pendant** (PR #16): `scripts/urcap5-usb.sh` now writes `urmagic_perceptronic.sh` on the stick; enable Settings → Security → General → *Run magic files* (+ *USB ports*), plug in with the arm powered off, expect `! USB !`, a restart, and Perceptronic 0.6.0 under Installation → URCaps. If it doesn't load: is the jar at `/root/.urcaps/com.nickarmenta.perceptronic.jar`, and does PolyScope need the symlink its own installer makes (`urcapSymLinkFolder`)? Read `urmagic_perceptronic.log` on the stick.
- 2026-09-29 — Click-through of the PolyScope X URCap in the sim from an SSH session is blocked on **Docker Desktop, which an SSH login can't launch** (`open -a Docker` → launchd "Domain does not support specified action"). Start it from the console / Screen Sharing (:5900), then `HOST_ARCH=arm64 make simx-up && make urcap-install && make urcap-cockpit` and forward 8000 + 7621. Or: is a headless Docker (colima) acceptable on the Studio so agents can start it?

- 2026-09-29 — **Perceptronic namespace**: the URCap is now owned by you personally (`Bundle-Vendor: Nick Armenta`), and the bundle / Java package became `com.nickarmenta.perceptronic` (PolyScope X vendor `nickarmenta`) — a guess at your personal reverse-DNS from the GitHub handle. Want a different namespace (e.g. your own domain)? It is one `perl -pi` over `urcap/` + tests, but it must happen before the 0.6.0 jar lands on the UR3e (a new symbolic name = a third URCap to remove).
- 2026-09-29 — The README screenshots (`urcap/perceptronic-ps5/screens/*.png`, 1000 × 560) came from a one-off harness that isn't in the repo; they still say "RealSense Pick" and lack the P mark. Re-render them (`preview5.py --snapshot` gives the whole 1280 × 772 window, not that framing) or accept the drift?
- 2026-09-29 — The P button's icon (30 px badge) and popup (420 px tall) are unverified on a pendant: if the header button looks wrong-sized or the popup clips, `ToolbarService.ICON_PX` / `HEIGHT_PX` are the two numbers.

## After the demo (2026-09-28)

- 2026-09-25 — (held until after the demo, Nick 09-27) Four pick-cycle faults could not be injected from the desk and are untested claims until someone does them once: pendant flipped to **Local** mid-run, a webcam or the D435 **unplugged** mid-run, the cockpit **restarted** while the routine is on a block, robot **power cut**. Note what the routine did in the field log.
- 2026-09-27 — Fix the flaky URCap e2e (2 of 4 fork runs + PR #16 red: PolyScope's "An error occurred while starting the application" boot dialog). Fix or delete, never retry. **Evidence 2026-09-28 (PR #7, same commit: `e2e` passed, `track`'s e2e failed):** the failing boot reached "web UI up" at 35 s and our install was accepted, then ~38 s later PolyScope X's web-bootstrapper was still installing its own URCaps (`Failed finding URCapX urconnect/ur-ethernetip/ur-profinet-g5 in urcaps folder`, urservice `not_found: universal-robots/java-backend`) and the app raised "An error occurred while starting the application". Suspect: "web UI 200" is not boot-complete, so installing then races the bootstrapper. Unproven — passing runs don't dump container logs; next step is to dump them on success too and compare, then gate the install on boot completion.
- 2026-09-27 — Reminder for Nick: `sudo chown -R nick captures` (sudo cockpit runs leave it root-owned).
- 2026-09-27 — Repo automation baseline: no `dev` branch; PRs go straight to `main`. Add `dev` + auto-merge + protection per ~/.claude/templates/github/. **Decided 2026-09-28: the repo needs a `dev` branch, cut from `refactor/prune-2026-09-25`** (main is 165 commits behind and has no URCap code; main catches up by Nick's manual merge).
- 2026-09-27 — `pick-cycle --dry-run` without `--via-cockpit` builds a dry-run `Robot` whose flange is a stand-in pose, so the survey's base-frame block positions are fiction. Use `--via-cockpit --dry-run` until fixed.
- 2026-09-27 — Retire the `perceptronics pick-server` sidecar (192.168.3.10:7631) once the cockpit runs the merged code: `pkill -f "pick-server --bind"` **before** relaunching the cockpit (both bind :7622), then point the pick node's URL back at `:7621`.
- 2026-09-27 — `tests/test_handeye.py`: `test_robotlink_locate_then_move_goes_through_the_tool_registry` and `test_robotlink_approach_defaults_come_from_the_cell_env` have failed since b41d795 halved the accelerations: they expect `a=0.3`, the code sends 0.1.

## Decisions (so they don't get re-asked)

- 2026-09-29 — Claude merges PRs itself once every check is green (Nick: "You do all of the merging automatically once they complete their CICD loops"). `dev` requires `gate` + `URCap5 gate`; the URCap matrix answers every PR, instantly when the URCap isn't touched.

- 2026-09-28 — **No 0.3.0 URCap release** (Nick): the `urcap5-v0.3.0` GitHub Release and tag (cut 12:30 from 174959b) were deleted. The committed `urcap/dist/realsense-pilot-ps5-0.3.0.urcap` stays until the ps5 branches replace it with 0.4.0 (the source is still 0.3.0 and a test pins the build).
- 2026-09-28 — The fork is **JimothyJohn/perceptronics** (renamed from universal-perceptronics). The numpy/OpenCV/Pillow extra is **`vision`** (`uv sync --extra vision`). The repo has a **`dev`** branch (cut from main 174959b) protected by the single `gate` check; PRs go to dev, dev → main is Nick's merge.
- 2026-09-29 — PS5 backwards compatibility (Nick): on 5.4–5.7 a pick area taught by touch uses the **nominal DH** (no calibration) — **accurate enough**; 5.9.0–5.9.3 (no image) are treated as **lacking the IK check** — fine; `check-tags` **compares the version inside each image** (`IMAGE_VERSIONS`); the committed jar is checked by **comparing compiled classes** (`urcap5.py compare`), not bytes, and CI rebuilds it with a different JDK to prove it.
- 2026-09-28 — Pick kit (Nick's answers): **reach** = base outer radius + 150 mm .. rated reach − 150 mm (as built); **pick order is FPV** — the directions of the camera's picture as the pendant shows it (as built); the Pick node **drives the Robotiq Hand-E itself** by default (as built); the camera computer's OS is **Debian** (arm64; the RevPi Connect 5 gets a Debian image, not RevPi OS); the PolyScope 5.x matrix jobs **must not block unrelated PRs** — they stay out of `dev`'s required checks (path-filtered, informational).
- 2026-09-28 — With no camera, every view shows an unmistakable **NO CAMERA CONNECTED** test card (Nick); a simulated picture is stamped the same way.

- 2026-09-28 — **Operator UI rule (Nick):** extremely user friendly — minimal clicks and input; the user is prompted through **at most 3 simple stages** with plenty of visual feedback. Applies to the URCap nodes' screens.
- 2026-09-28 — Pick segmentation goes **depth-above-the-table**, the table **fitted live in every frame** (no setup step; follows a pedestal/table change); part size stays as the filter.
- 2026-09-28 — Pick node setup = **3 stages: Show → Check → Test.** 1) tap one part in the live picture — the node **measures its size from that tap** (no typing); 2) watch the arm go above it (hold-to-move); 3) one test pick with a clear pass/fail.
- 2026-09-28 — The repo gets a `dev` branch cut from `refactor/prune-2026-09-25` (automation baseline); PRs land there, `dev` → release line stays Nick's manual merge.
- 2026-09-28 — **Project name: perceptronics**, everywhere — the `perceptronics` package, CLI, MCP server and `PERCEPTRONICS_*` variables included (supersedes `universal-perceptronics` from earlier the same day). **No PyPI package.** Don't touch Olympus-Controls (repo, org, links).

- 2026-09-27 — Demo: Nick drives the cockpit in the browser and the room watches the UR3e (e-Series) move; heuristic pick first, then a semantic object. Host: the Windows work laptop (trial tonight). The URCap and everything after target PolyScope X.
- 2026-09-27 — Keep the 09-25 orbit hand-eye solve (nothing moved). `pick-cycle --min-radius-m` defaults to 0.2 m.
- 2026-09-27 — URCap X target held at 10.13 (`target.json` `hold`) to match the local simulator image.
- 2026-09-27 — Public docs point at Olympus-Controls/UR-utils, not the personal fork.
- 2026-09-26 — Pruning round two: only `urctl-gui` goes (done); the 2-D `perceive` pipeline, `sysinfo`/snapshot and the guided wizard + sample programs **stay**; SAM extra stays for the Jetson. Done the same day, not after the demo.
- 2026-09-26 — `Controller` / `Gripper` protocols in `urctl/controller.py`; tool names canonical without the `ur_` prefix, `ur_*` kept as aliases. A Fanuc implementation is a later branch.
- 2026-09-26 — PolyScope X integration = a URCap X Application Node (`urcap/realsense-pilot`) talking to the cockpit over HTTP with CORS; the cockpit stays the one place the camera is opened.

- 2026-09-25 — **Final deployment: a Jetson next to a UR running PolyScope X**, with room for a second Jetson on other arms. Controller IPs are always different — cells carry them (`perceptronics/cells/*.env`); `ur20.env` is filled on site, not in the repo.
- 2026-09-25 — **Monday 2026-09-28 demo is on the e-Series UR3e.** The platform must stay compatible for ~10 years; the demo view (`--demo`) and `docs/realsense-cell.html` stay through the demo.
- 2026-09-25 — Prune mandate: drop what is not inherent to gross vision guidance + robust agentic control; keep it portable (Windows, Mac, Jetson, Linux, new cameras) and open to other arms/controllers (Fanuc, PolyScope X).
- 2026-09-25 — **Monocular scanner dropped** ("don't worry about it for now"). Removed in `200d175`; the parent commit has it if the idea comes back. The ChArUco plate/probe prints, the `depth` extra's backend, the trigger-camera and line-laser purchases went with it.
- 2026-09-25 — Pick-cycle geometry (verified in person): approach along the **tool** axis; the Hand-E fingers travel along flange **Y**; a yaw about the down-pointing tool Z is the negative of the base-heading yaw. Each block is one compiled URScript program; the cockpit does vision only.
- 2026-09-25 — Hand-E: the Robotiq URCap daemon is loopback-only on the controller, so the gripper is driven from URScript (`urctl gripper`, `ur_gripper`, `POST /api/robot/gripper`).
- 2026-09-25 — Mac as D435 host: no (43 libusb resets per start; `docs/realsense.md` §Troubleshooting). `--rs-lean` is the flagged exception for dev.
- 2026-09-04 — Perception / send-to-robot work targets PolyScope X; the e-Series sim on the Mac Studio is abandoned (Xvfb dies under Rosetta, URControl under QEMU). URSim e-Series stays CI-only.
- 2026-09-12 — The UR3e in the test cell is **PolyScope 5 (e-Series)**; the UR20 is PolyScope X. Cells carry the platform, so both work; `UR_PLATFORM` keeps its `e-series` default (no global flip).
- 2026-09-12 — Bracket camera side: +Y, the tool-I/O side (Rev B); the UR20 print clocked 45° off its M8 socket.
- 2026-09-12 — Hand-eye calibration routine in the cockpit: **yes, build it** before the demo. (Built: touch-and-click 09-23, orbit 09-25.)
- 2026-09-12 — Repo push: fork `Olympus-Controls/UR-utils` and push the branch to the fork; PR from there.
- 2026-09-12 — D435 USB-C is on an **end face** of Nick's unit (not the back as in Intel's mesh); bracket clears it, spec §6 A1 updated.
- 2026-09-12 — SAM backend stays "wired, unverified" until a GPU box exists; the demo uses the stub segmenter.
- 2026-09-12 — Dependabot ignores `universalrobots/ursim_polyscopex` bumps (pinned 10.13.0; re-test new tags by hand).
- 2026-09-27 — **Approach by the fingertips, always** ("the tool offset is critical"): the default approach reference is `fingertip` (`PERCEPTRONICS_TIP_M`, 0.163 m on the UR3e's Hand-E), moves run with that TCP, the controller's active TCP is never used for it.
- 2026-09-29 — PolyScope X Pick node (URCap 0.3.0): the Robotiq gripper option assumes the Hand-E URCap *for PolyScope X* also serves `127.0.0.1:63352` on the controller like the e-Series one — unverified; if it doesn't, the node needs a PolyScope X gripper path (tool I/O / the URCap's own nodes). Which gripper is on the UR20?
