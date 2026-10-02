# Perceptronic — developer notes

*Installing it on a robot? See [README.md](README.md). This page is for working on the URCap.*

`urcap/perceptronic/` is a **URCap X** (PolyScope 10) Application Node: the
RealSense colour feed inside PolyScope's own UI, hover-to-measure, and a click
that becomes a base-frame point and an approach pose through the cockpit's
hand-eye — then a move, either through PolyScope's auto-move screen (operator
holds to move) or through the cockpit over Primary (urctl's safety envelope).
It is plain JavaScript: **no Angular, no webpack, no npm** — `urcap/urcapx.py`
packages and installs it with the stdlib.

<!-- urcap-target -->Built and tested for **PolyScope X 10.13.1** (held at 10.13, 10.13; simulator `universalrobots/ursim_polyscopex:10.13.0`, URCap SDK 6.5.65 / contribution-api 21.3.266) — pinned in [`target.json`](target.json) and kept current by `.github/workflows/urcap-track.yml`.<!-- /urcap-target -->

```
urcap/
  README.md                                install + use (for whoever downloads it)
  DEVELOPING.md                            this page
  urcapx.py                                package | install | list | delete (stdlib only)
  dist/perceptronic-<ver>.urcapx        the downloadable package (committed; see below)
  perceptronic/
    manifest.yaml                          vendorID advin, urcapID perceptronic
    perceptronic-frontend/
      contribution.json                    the applicationNode (tag advin-perceptronic) + one programNode (3D Pick)
      main.js                              the application node's presenter (a custom element): two tabs — the feed (Picture / Depth toggle, click → locate) and the pick areas on the arm's reach
      perceptronic-node.worker.js       its behavior worker (node factory / upgrade)
      pickscript.js                        the Pick node's settings + URScript + pose math + the drawings as SVG (worker, page and tests share it)
      pick.js                              the program node's presenter: the 3D Pick row + its dialog (no scrolling; Options = Part / Approach tabs)
      pick-node.worker.js                  the 3D Pick node's behaviors (label, validator, the whole script before "children" it does not have)
      assets/i18n/en.json                  node titles + supportive text (program.tree.nodes.<tag> for program nodes)
      assets/icons/perceptronic.svg      the P mark — a copy of ../perceptronic.svg (a test holds them equal)
      assets/icons/perceptronic-*.svg    the program nodes' toolbox icons
```

**`dist/` is committed and must match the source.** `urcapx.py package` is
reproducible (fixed owners/modes, one mtime derived from the contents so an
update never reuses the old Last-Modified/ETag), and
`tests/test_urcap.py::test_the_downloadable_package_is_the_current_source` fails
until the committed file equals a fresh build. After editing anything under
`perceptronic/`: `make urcap-package` and commit `urcap/dist/`. Bumping
`version` in `manifest.yaml` renames the file — delete the old one (the test
refuses leftovers) and update the link in README.md.

## Why it is shaped like this (sourced from the SDK, 2026-09-26)

Facts from `PolyScopeX_URCap_SDK` 6.5.65 — the SDK that pairs with our simulator
image (`ursim_polyscopex:10.13.0` = `0.18.96`; `contribution-api` 21.3.266,
`urcap-utils` 2.1.2, manifest schema 19.10.31; SDK 6.6.66 at the repo's HEAD
targets 10.14):

- **A frontend may be any standards-compliant web component.** The generator's
  "javascript (expert mode)" template is `class X extends HTMLElement` +
  `customElements.define`. PolyScope sets `applicationNode`, `applicationAPI`,
  `robotSettings`, `robotContext` on the element; state is saved with
  `applicationAPI.applicationNodeService.updateNode(node)`.
- **The behavior worker is threads.js.** `registerApplicationBehavior(b)` is
  `expose(b)` from threads 1.7 (contribution-api fesm2022 ~line 6205), so the
  worker here speaks that protocol directly (`init` / `run` / `running` /
  `result` / `error` messages, verified against threads 1.7.0's
  `dist/worker/index.js`) — `tests/test_urcap.py` runs it under node.
- **`.urcapx` is a gzipped tar with `manifest.yaml` first** (`package-urcap.js`
  + `tar-helper.js`); `urcap/urcapx.py package` does the same.
- **Two install endpoints.** The SDK's CLI posts field `urcapx_file` to the
  Robot-API (`/universal-robots/robot-api/urcaps/v1/urcaps/`,
  `urservice-helper.js`), which answers **403 unless the robot is in Remote
  mode** — from inside the container too ("must be in remote mode or originate
  from an internal URCap"). PolyScope's own System Manager (an internal URCap)
  uses `/universal-robots/urservice/api/v1/urcaps` instead, and that one
  accepted the package from the host in Local mode (2026-09-26, 10.13.0 sim):
  multipart `POST` of field **`urcapxFile`** → 201, a duplicate → 409
  `already_installed`, `DELETE …/<vendor>/<urcap>` → 200, `GET` → a plain
  JSON array. `urcap/urcapx.py` uses that endpoint (`--replace` = delete +
  install; there is no update verb). The SDK's `run-simulator` sets
  `DEVMODE=true` for the Robot-API path; our compose passes `URSIM_PX_DEVMODE`
  through, but it only matters at container creation.
- **The page is same-origin with PolyScope** (nginx serves web archives from
  `/var/urcaps`; no CSP on the 10.13 sim), so `fetch` to another host works
  the way any page's does: the **cockpit must send CORS headers** for
  PolyScope's origin — `perceptronics gui --cors http://<pendant-or-sim-host>:<port>`
  (`PERCEPTRONICS_CORS`). The docs allow direct REST from a frontend on a real
  robot; the sim itself cannot reach external devices, but the *browser* can.
- **No frontend API runs URScript.** `ApplicationPresenterAPI` offers
  `robotPositionService.getInverseKinematics(pose, qNear)` and
  `robotMoveService.autoMove(joints)` (UR's hold-to-move screen), and there is
  no script endpoint in the Robot-API either. Hence the two Move buttons.

Facts the **program nodes** (`pick.js`, the two `*-node.worker.js`) are built on — read
out of PolyScope 10.13's own bundles (`web-app/main.js`, `web-program-nodes/*`),
2026-09-29:

- **A program node's presenter renders inside its tree row** (`ur-inline-presenter` in a
  48 px `virtual-tree-item`): the Pick row is one line; the real screen is a
  **custom dialog** — `presenterAPI.dialogService.openCustomDialog(tag, inputData,
  {title, dialogSize: "XL", confirmText, raiseForKeyboard})`. PolyScope creates the tag's
  element, sets `inputData`, `presenterApi` (a `WebComponentDialogAPI`) and `afterOpen` on
  it, listens for `outputDataChange` / `canSave` / `closeDialog` DOM events, and closes it
  from its own footer. `inputData` is passed by reference (UR's own nodes hand their
  `presenterAPI` through it), so the dialog saves through the row's
  `programNodeService.updateNode` as it goes.
- **Behaviors:** `registerProgramBehavior` is `expose` like the application one; the
  worker answers `factory`, `programNodeLabel` (`[{type: "primary"|"secondary", value}]`
  — PolyScope prefixes the row with the i18n title itself), `validator` (`{isValid,
  errorMessageKey}`), `generateCodeBeforeChildren` / `generateCodeAfterChildren`,
  `allowsChild`, `upgradeNode`, `onLifeCycleHook`.
- **A ScriptBuilder crosses the worker boundary as `{type: "$$ScriptBuilder", script,
  currentIndent}`** (PolyScope rebuilds `new ScriptBuilder(script, currentIndent)` and
  `append`s it: the lines at the parent's indent, then the children `currentIndent`
  levels deeper — so the before-children builder ends with the open block and
  `currentIndent = depth`, the after-children one carries `-depth`). Lines keep their own
  leading spaces; empty lines are dropped.
- **The application context arrives serialized:** `{type: "$$ApplicationContext",
  contributions: {contributionList: [...]}, frames: {framesList}}` — our application node
  is the entry whose `type` / `parentType` is `advin-perceptronic` (cockpit URL,
  areas, tip, robot model).
- **Program variables** are declared from the presenter with
  `variableService.createVariable(name, "boolean" | "integer")`; the declaration
  (`{id, name, valueType, _IDENTIFIER}`) is stored in the node, the script writes
  `global <name> = …` (what UR's Assignment node emits for a declaration).
- **The drawings are the PolyScope 5 node's** (`Diagrams.java`: the order tiles, the part
  as a box or a cylinder, the approach from the side, the map of the arm's reach) as SVG strings
  from `pickscript.js` — pure functions, so the tests hold `orderGrid` to the Python
  detector's numbering and every drawing to well-formed, escaped SVG.
- **Older PolyScope X (the release matrix, 2026-09-30; the floor is 10.8):** `robotPositionService.convertJointPositionsToTcpPose`
  is 10.10+ (without it Move (PolyScope) / Check approach take the flange target as the
  TCP and say so); `variableService` is 10.12+ (before it, `symbolService.generateVariable`
  declares the program variables — a URVariable with the same `name`). The row declares
  them *before* opening the dialog: the dialog edits the node object it is handed, and a
  declaration landing on the row's copy afterwards was saved over (seen on 10.10, where
  the older service answers slower).
- **The URScript the node writes is the PolyScope 5 node's** (`PickScript.java`) —
  `pickscript.js` is its port, `tests/test_urcapx_pick.py` holds it to the Python pick
  server's parser the way `tests/test_urcap5_pick.py` holds the Java. PolyScope X's script
  editor lists every function it uses (`get_inverse_kin_has_solution`,
  `socket_read_ascii_float`, …); verified by playing it in the 10.13 sim.

## Running the mock-up (sim on this Mac)

```bash
make simx-up                       # PolyScope X 10.13.0 on http://localhost:8000
make urcap-cockpit                 # a synthetic cockpit on :7621, CORS for the sim's origin
make urcap-install                 # package + install (no Remote mode needed; --replace if already there)
```

Then in PolyScope X: refresh the browser, **Application** → **RealSense
Pilot**. Leave the cockpit URL empty (= this host on :7621, the cockpit's
normal port) or set it to `http://localhost:7621` (Save; it persists in the
node), and the feed appears; hover reads depth, a click marks the pixel,
segments and calls locate. The fake cockpit has no robot behind it, so locate
answers "robot unreachable" there — the base point, approach pose, reach and
the Move buttons need a cockpit with a robot link (below).

For the real camera: restart your cockpit with CORS, e.g.

```bash
sudo python3 -m perceptronics --cell ur3 gui --rs-lean --cors http://localhost:8000
```

(same port as the fake one, so the node's URL doesn't change; stop `make
urcap-cockpit` first). Move (cockpit) then drives the
UR3e exactly as the cockpit's own Move does; Move (PolyScope) is only
meaningful on a PolyScope X controller (the sim's arm), where IK + auto-move act
on *that* robot.

### "no cockpit at … (Failed to fetch)"

The browser says only *Failed to fetch* whether the cockpit refused the page's
origin or nothing answered, so the node probes the cockpit URL again with a
`no-cors` request (it resolves when a server is up, whatever its CORS list)
and says which it is:

| The node says | Meaning | Fix |
| --- | --- | --- |
| *running but refuses this page (origin X)* | The cockpit is up but X is not in its `--cors` list, or the cockpit predates `--cors`. The cockpit's stderr names the origin once (`CORS: refused a page from X`), and `GET /api/info` lists it under `cors.refused`. | Restart with `--cors X`. The origin is exact: `http://localhost:8000`, `http://127.0.0.1:8000` and `http://192.168.3.10:8000` are three different origins. |
| *nothing answers at …* | Wrong host or port, the cockpit is not running, or it is bound to loopback while the browser is on another machine. | `--bind 0.0.0.0` on a trusted cell network; `localhost` in the node means the machine running the browser (the pendant's own browser on a robot). |
| *answered HTTP 404 on /api/color.png* | The cockpit predates the URCap routes. | Update and restart it. |

The cockpit also answers Chromium's Private Network Access preflight
(`Access-Control-Allow-Private-Network`) for allowed origins, for a page on a
LAN address calling a cockpit on loopback.

## Staying on UR's newest release

The URCap targets exactly one PolyScope X release, the newest patch of the newest
minor UR has release notes for, pinned in [`target.json`](target.json): the notes
URL, the simulator image **by digest**, and the SDK release UR titled for that minor
with the versions its JavaScript template uses (`contribution-api`, `urcap-utils`,
`threads`, manifest spec). A new minor *replaces* the pin; older minors are not built for.

| | |
| --- | --- |
| `python3 urcap/track.py check` (`make urcap-track`) | exit 1 with the reasons when UR has moved on (new minor or patch, a moved simulator tag, an SDK component bump) |
| `python3 urcap/track.py update` | re-resolve and rewrite `target.json` + the `urcap-target` lines in README.md and this page |
| `python3 urcap/track.py compat` (`make urcap-compat`) | every PolyScope member the node calls or implements (`track.API_SURFACE`, held to `main.js` + the worker by a test) is still in the pinned `contribution-api` typings (UR's npm feed); `manifest.yaml` validates against the SDK's manifest spec; the template still uses the `threads` the worker's hand-written protocol was verified against; the SDK's simulator is the notes' robot image by digest |
| `python3 urcap/psx_matrix.py run --version all --rmi` (`make urcapx-matrix`) | the same e2e against the newest PolyScope X releases on Docker Hub from the URCap's floor, 10.8 (`RELEASES` / `FLOOR`; `check-tags` flags a newer one), one summary; `.github/workflows/urcapx-matrix.yml` runs it per release on changes to the URCap and weekly (not a required check) |
| `python3 urcap/e2e.py` (`make urcap-e2e`) | boots the pinned simulator, installs a fresh build over urservice, checks nginx serves the packaged bytes, then headlessly: node renders, goes live on a `--fake` cockpit, hover depth, click → `/api/segment`, PolyScope's `getKinematicInfo` / `getJointPositions` / FK → IK round trip, the saved cockpit URL survives a reload. About 2 min on the Mac (arm64 image); `--keep` leaves the sim up |

Nothing needs a login: the notes, the SDK (`UniversalRobots/PolyScopeX_URCap_SDK`),
Docker Hub and UR's npm feed (`pkgs.dev.azure.com/polyscopex`) are public.
`.github/workflows/urcap-track.yml` runs `update` → unit + `compat` → `e2e.py`
weekly and opens a PR (with the notes' breaking changes and URCap/API sections)
when all pass, or a `urcap-attention` issue when something needs a person — an API
member gone, a new manifest rule, a `threads` bump, a node that no longer loads.
`.github/workflows/urcap-e2e.yml` runs the e2e on every change under `urcap/`.
A PR that touches the tracker (`track.py`, `e2e.py`, `target.json`, the workflow) runs
the tracker workflow dry: live sources, full tests, no PR or issue.
A patch UR ships without a simulator build (10.13.1 had none) is tested on the
minor's newest simulator; a minor whose SDK or simulator is not out yet waits, and
becomes an issue after three weeks.

## What is verified and what is not

| Verified (2026-09-26) | How |
| --- | --- |
| Source tree consistent (manifest ↔ contribution ↔ tag ↔ i18n) | `tests/test_urcap.py` |
| Package = gzipped tar, manifest first, LICENSE + frontend inside | test |
| Install: POST when absent, PUT when present, `--replace` deletes first, 403 named | test against a Robot-API look-alike |
| Worker speaks the threads protocol (init/running/result/error) | test under node |
| Cockpit CORS (allow-listed origins only, preflight, exposed `X-Seq`) + `GET /api/color.png` | test |
| The node tells a CORS refusal (your pre-`--cors` cockpit on :7621, a cockpit with a different origin) from a dead port, and goes live on a CORS-enabled one; the cockpit logs the refused origin | headless Chromium on the sim, four cockpit URLs |
| The sim accepts the package (201), lists it, deletes it, serves the four frontend files | `urcap/urcapx.py install --replace` against `ursim_polyscopex:10.13.0` |
| On **10.14.0** (SDK 6.6.66, contribution-api 22.14.184): install + serve as packaged, node renders, live feed, hover, click → segment, `getKinematicInfo` 6 rows, FK → `getInverseKinematics` round trip to 7e-15 rad, URL persists; a package with a broken `main.js` fails at *node renders* | `urcap/e2e.py` against `ursim_polyscopex:10.14.0@sha256:f306cd98…`, 2026-09-27 |
| The node loads in PolyScope X: worker + presenter fetched, element present, i18n title shown | headless Chromium on the sim (Playwright) |
| Feed live from a CORS-enabled cockpit, hover depth, click → segment → locate, cockpit URL persisted through `updateNode` and a reload | same, against `make urcap-cockpit` |
| Locate → base point / approach / reach, Move (cockpit) | **not yet** — needs a cockpit with a robot link (your live one with `--cors`) |
| Move (PolyScope): IK + auto-move accept the pose, `Pose.orientation` is a rotation vector | **not yet** — needs the PolyScope X arm powered and a located target |

Open: `Pose.orientation` for `getInverseKinematics` is assumed to be the UR
rotation vector (the SDK's `Pose` doc says only "x, y, z components"); a
mismatched convention shows up as the auto-move screen targeting a rotated
tool — compare against the cockpit's approach pose on first use.

## On the robot (later)

On the Jetson-next-to-a-PolyScope-X-robot deployment the cockpit runs on the
Jetson and the pendant's browser reaches it over the cell network — the same
URCap, CORS for the pendant's origin. The alternative the SDK offers is a
**backend container** declared in the manifest (`containers:` with
`devices: [{type: video}]` hot-plug hooks and `services: [urcontrol-primary]`),
which would put the cockpit inside PolyScope's Docker and reach the controller
on `urcontrol-primary:30001`; that is a packaging step on top of
`Dockerfile.perceptronics`, not a rewrite.
