# ENTERPRISE.md — from a pick kit to lights-out machine tending

Written 2026-10-04 for Nick's review. Every fact about today's state is from the repo at
`dev` (commit of this file's PR) and is cited by path; every number that is a proposal
says so. Decisions Nick has to make are collected in §10 and nowhere else.

The one-line product: **a UR owner running a small parts shop loads a tray of blanks in the
evening; the robot feeds the machine all night; the parts are in a bin in the morning.**

Today we sell the first third of that: the robot finds the blank and puts the gripper on
it. This document is the plan for the other two thirds — the place into the machine and
the discipline to keep running unattended — and for the chain of hardware, firmware and
software that lets us ship it to a stranger's cell and keep it working without us
standing next to it.

---

## 1. Where we stand (facts, 2026-10-04)

**What a customer can get today** (`docs/pick-kit.md`, `site/public/index.html`):

| Piece | State | Evidence |
| --- | --- | --- |
| 3D Pick URCap for PolyScope 5 (`io.advin.perceptronic` 0.9.0) | Built, 23 PolyScope versions 5.4–5.26 green in CI (`urcap5-matrix.yml`) | `urcap/dist/perceptronic-ps5-0.9.0.urcap` |
| 3D Pick URCap for PolyScope X (0.7.0) | Built, 10.8–10.14 green in CI (`urcapx-matrix.yml`); screens still the 0.8.0 layout | `urcap/dist/perceptronic-0.7.0.urcapx`, `urcap/perceptronic-ps5/README.md` §PolyScope X |
| Pick PC (Pi, cockpit, pick server, firewall, DHCP for the robot) | Deployed and verified on one Pi 5; flashable image boots in ~30 s | `deploy/pi/README.md` §Verified on a board, `deploy/pi/image/README.md` |
| Setup portal + update bundles (network from a browser, upload an update, auto-rollback) | Code + 115 tests, **never on a board** | PR #59, `perceptronics/setupportal.py`, `deploy/pi/perceptronics-admin` |
| Camera bracket | Rev B.2 printed and on the UR3e; **Rev C and the UFACTORY print are unprinted** | `hardware/d435-tool-bracket/README.md` §7 (13 unchecked boxes) |
| BOM | Priced, ≈ $785 per cell with a Pi 4 (≈ $740 for what one cell consumes) | `hardware/BOM.md` |
| Site + datasheet + UR Quickstart PDF | perceptronics.advin.io is live but **serves 0.8.0 / 0.6.0**; the any-robot rewrite is PR #55 | `site/` |
| UFACTORY 850 driver | Against UFACTORY's simulator only | `urctl/ufactory.py`, `ufactory-sim.yml` |

**What has never happened** (`PLUG-AND-PLAY.md` §Not verified, `SETUP.md`, `TODO.md`):

- No URCap newer than 0.5.0 has been on a pendant. 0.6.0 → 0.9.0 exist as renders only.
- The Pi has never talked to the real UR3e. The UR3e has been unplugged since 2026-09-29.
- No pick has been made through the URCap on a real arm. The real picks so far ran from
  the Mac's cockpit (2026-09-25/27).
- The kit board is a Pi 4 (`BOM.md` K1); only a Pi 5 has ever been run.
- No place move, no machine handshake, no gripper control in the node (by decision: the
  customer's program opens before and closes after, `docs/pick-kit.md` §Program).

**The honest summary:** we have a well-tested *picking* component and a well-tested
*deployment* story, both one real-robot day away from being provable, and **zero** of
the machine-side product. Launching tomorrow means launching the pick kit as a beta and
committing publicly to the tending product with dates. It does not mean a shop runs
lights-out tomorrow.

---

## 2. The customer and the job

**Who.** A job shop with 1–5 CNC machines (mills, lathes, Swiss, presses, saws) and one UR
e-Series or UR20, owner-operated, 3–20 people. They bought the UR for machine tending and
it tends one machine during the day with a hand-loaded tray or a fixtured grid. The
machine stops when the tray is empty or the shift ends. They cannot afford an integrator
for every job change and they will not learn vision programming.

**The job to be done.** Run the machine through the night on one tray of blanks, and
through the weekend on three. Concretely:

1. Load: the operator tips blanks onto a tray or a flat surface. No fixture, no order.
2. Feed: the robot takes the next blank, loads it into the chuck/vise, cycles the machine.
3. Unload: the robot takes the finished part out and drops it in a bin (or sets it on a tray).
4. Keep going: until the tray is empty, the bin is full, or something is wrong.
5. Stop safely and say why: a text or an email, and a pendant screen that says what to do.
6. Resume: the morning operator refills the tray and presses one button.

**What they are paying for.** Not the camera. The *hours the machine runs with nobody in
the building*. A 3-axis mill that earns $60–120/h lit-out for 8 extra hours a night is
worth $100k+/year to that shop. That is the number the kit price hangs off, not the BOM.

**What we are not.** Not bin picking, not inspection, not a cobot cell integrator, not a
machine builder. The machine, the gripper, the fixtures and the risk assessment are theirs.
We own: finding the part, putting it in the machine, keeping the loop honest.

---

## 3. Product definition: three tiers, one hardware kit

| Tier | What it does | Exists? | Price (proposal) |
| --- | --- | --- | --- |
| **Find** (today's pick kit) | Finds every blank of the taught size on a flat surface, numbers them, brings the gripper to the grip. Customer's program does everything else. | Yes (beta, unproven on a pendant) | Kit $1,950; URCap free (MIT) |
| **Tend** | Find + the tending program template: place into the chuck/vise, machine handshake over the UR's I/O, unload to a bin, tray-empty stop, one-button resume. Installed by us or a partner in a day. | No. §5 is the build. | Kit + $2,500 install/commissioning + $150/month per cell support |
| **Lights-out** | Tend + the unattended discipline: part-present proof at every step, machine-alarm stop, text/email alerts off the plant network, a cycle log and a morning report, remote diagnostics. | No. §5. | +$100/month per cell |

Why these prices (proposal, Nick decides): BOM ≈ $785 + bracket + assembly/test ≈ 1.5 h
→ ≈ $950 landed cost per kit; $1,950 is ~50 % gross margin and far below any integrator's
first invoice. Tend's install fee is one day of a person on site. The monthly fee is what
funds the thing nobody budgets for: the 2 a.m. phone call. Robotiq's and Vention's
machine-tending bundles are $15k–45k; we are the "already own the robot" tier below them.

**One hardware kit for all tiers.** The tiers are software and service. A Find customer
upgrades to Tend by installing a bundle through the setup portal and booking a day.

---

## 4. The chain: every step, an owner, a trigger, a done-when

"Someone makes it happen" is the rule. For launch the owner of nearly every row is Nick;
the point of the table is that each row is a *job* with a checklist that a second person
(or a routine) can take over without asking. **A = automated today, M = manual with a
script, N = nothing exists.** The automation column says what to build so the row runs
without Nick.

### 4.1 Hardware

| # | Step | Owner | Trigger | How (today) | Done when | A/M/N | To automate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| H1 | Order parts for N kits | Nick | Stock < 2 kits | `hardware/BOM.md` by hand; D435 lead 6–8 wk + tariff; Pi 4 price moves | Parts in the bin, serials logged | N | A `hardware/stock.csv` + a weekly routine that reads it and opens an issue "order N kits" with the BOM lines and last-seen prices |
| H2 | Print the bracket | Builder | Kit order | `bracket.py` → STL in `hardware/d435-tool-bracket/out/`; PPA-CF, settings in README §5 | README §7 checklist all ticked for this print | M | Checklist as a form on the kit's build record (H5) |
| H3 | Assemble the kit | Builder | Printed bracket + parts | Bracket + D435 + screw + clip; Pi in DIN case + PSU + SD; cables | Photo of the kit, weight, serial label | N | A one-page `hardware/ASSEMBLY.md` with the photo sequence; label printer template |
| H4 | Burn-in | Builder | Assembled kit | None today | Pi booted the image, D435 streamed 30 fps for 1 h on this unit's cable, `doctor` all green, thermals logged | N | `scripts/burn-in.sh <pi>`: runs the doctor + a 1 h `/api/info` fps/temperature watch, writes `burn-in-<serial>.json` |
| H5 | Build record | Builder | Burn-in pass | None | `kits/<serial>.json`: BOM lot, bracket print, image build id, burn-in result, who | N | The burn-in script writes it; a routine posts a weekly "kits ready" summary |

### 4.2 Firmware (the pick PC image)

| # | Step | Owner | Trigger | How (today) | Done when | A/M/N | To automate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| F1 | Build the image | Nick | A release tag `v*` | `scripts/pi-image.sh build nick@pickpc` (12 min on the Pi with the librealsense cache) | `target/pi-image/*.img.xz` + sha256 + manifest | M | A self-hosted arm64 runner (the pickpc itself, or a second Pi on the bench) runs `build.sh` on every `v*` tag and attaches the image to the GitHub Release |
| F2 | Publish the image | Nick | F1 | Hand-upload + `imager.json` on the site (source is on a wip branch, not on dev) | perceptronics.advin.io/imager.json points at the new image, Raspberry Pi Imager lists it | M | Release workflow uploads and rewrites `imager.json`; `site.sh sync` |
| F3 | Flash + seed a kit | Builder | H3 | `scripts/pi-image.sh flash` (macOS) + `seed` (hostname, key, address, optional Wi-Fi, robot host) | Card boots, cockpit 200 on :7621, D435 30 fps | M | Flash station = Raspberry Pi Imager with the published image; seeding becomes unnecessary once the portal (F5) is the way in |
| F4 | Image smoke test | CI | F1 | Nothing in CI; a loop-mount fsck by hand | fsck clean, units enabled, wheel version matches the tag | N | `tests/test_pi_image.py` gets a loop-mount stage on the arm64 runner |
| F5 | Field update | Customer or us | A release | PR #59: upload `perceptronics-update-*.tar` at `http://<pc>:7621/setup`, or `scripts/pi-update.sh push`; health check + auto-rollback | `/api/info` reports the new version; previous release kept | M (unmerged, untested on a board) | The release workflow builds the bundle; the morning report (§6) says which cells are behind |
| F6 | Rollback | Customer or us | A bad update | `install.sh --rollback` / the portal's rollback on health failure | Old version serving | M | — |

### 4.3 Software (URCaps, cockpit, site)

| # | Step | Owner | Trigger | How (today) | Done when | A/M/N | To automate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | Change lands | Claude + CI | PR into `dev` | `gate` + `URCap5 gate` required; auto-merge on green | Merged | A | — |
| S2 | URCap matrix | CI | URCap paths touched; weekly cron | 5.4–5.26 in URSim; 10.8–10.14 in the PSX sim | All green | A | — |
| S3 | Release a URCap | Nick | Version bump in `bundle.properties` / `manifest.yaml` | `make urcap5-package` or `make urcap-package`, commit `dist/`, tag `urcap5-v<ver>` / `urcapx-v<ver>`; `release-urcap5.yml` checks the committed jar is the tag's build and publishes | GitHub Release with the file, sha256 and `urmagic_perceptronic.sh` | A after the tag | A routine that tags when `dist/` changes on `main` (Nick: tags today are `urcap5-v0.5.0` and `urcapx-v0.3.0`; dist is 0.9.0 / 0.7.0 — the public releases are four versions behind) |
| S4 | Promote `dev` → `main` | Nick | When Nick says | PR, hand-merged (#61 tonight) | `main` = `dev` | M | Stays manual by rule |
| S5 | Publish the site | Nick | `main` moved | `site/site.sh sync` from the laptop with `site/.env` | Live page shows the `dist/` versions and sha256 | M | A `deploy-site.yml` on push to `main` with an OIDC role scoped to the one bucket + distribution (deploy-chain PR, draft, Nick merges) |
| S6 | Datasheet / Quickstart PDFs | Claude | Page or version change | Local headless Chrome via `site/build.py --pdf`; a sha256 stamp fails CI until reprinted | One page; stamps match | M | A Playwright step in CI prints and commits the PDFs on `dev` (the stamp test then only guards drift) |
| S7 | USB install stick | Builder | Kit ships | `scripts/urcap5-usb.sh` makes the FAT32 stick with the `.urcap` + magic file | Stick in the box, labelled with the version | M | Part of H3 |

### 4.4 Install at the customer

| # | Step | Owner | Trigger | How (today) | Done when | A/M/N | To automate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| I1 | Pre-call | Nick | Order | Email: robot model, PolyScope version, gripper, machine, part sizes | `kits/<serial>.json` has the cell profile | N | A form on the site that writes the profile; the Quickstart PDF goes out with the kit |
| I2 | Mount + cable | Customer | Kit arrives | UR Quickstart PDF (`site/public/quickstart-ur.html`, PR #55) | Camera on the wrist, Pi on the DIN rail, both cables in | M | — |
| I3 | Network | Customer | I2 | Robot gets 192.168.3.3 from the Pi's DHCP; pendant's Cockpit field defaults to 192.168.3.20 | Pendant shows the picture | A (by design, untested on a real robot) | — |
| I4 | URCap install | Customer | I3 | Stick + "Run magic files", or Settings → URCaps → + | Node appears in the Program tab | A (magic file never run on a robot) | — |
| I5 | Calibrate | Customer | I4 | Installation node → hand-eye (touch-and-click, 4–6 views) | Doctor's hand-eye line green; a located point within 3 mm of the mark | M (touch-and-click verified 09-23; orbit never on the UR3e) | A one-tap "Calibrate" that runs the orbit and reports RMS, like the cockpit does |
| I6 | Teach the part + tray | Customer | I5 | 3D Pick node: tap a part, check approach | Green parts, numbered | M (0.9.0 screens never on a pendant) | — |
| I7 | Tend program | Us (Tend tier) | I6 | **Nothing today** — §5 | The template runs one full cycle with the machine | N | The template ships in the URCap; the install is filling six I/O fields |
| I8 | Acceptance | Us + customer | I7 | **Nothing today** | §7's acceptance test signed | N | A checklist on the pendant (the node's Test step) that writes the result to the Pi |

### 4.5 Keep it running

| # | Step | Owner | Trigger | How (today) | Done when | A/M/N | To automate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| R1 | Know a cell is running | Us | Always | Nothing; the Pi has no internet by design | A morning report per cell | N | §6 — the Pi posts a daily summary + alerts to a plant-network mail/webhook the portal configures |
| R2 | Diagnose a stop | Us | Alert | `GET /api/pick/log`, `captures/`, the audit log on the Pi; only over SSH on the same subnet | Cause named within an hour | M | The alert carries the last 20 events + the last picture; the portal has a "Send diagnostics" button that bundles them |
| R3 | Fix and ship | Claude + Nick | Diagnosis | Normal PR flow + F5 | Cell updated | M | — |
| R4 | Spares | Builder | Stock < 1 kit | H1 | A spare kit on the shelf for every 5 in the field | N | H1's routine |

---

## 5. What has to be built for Tend and Lights-out

The pick stays what it is. Everything below is *around* it, in the customer's own program
where Nick decided the gripper and the place belong (`docs/pick-kit.md` §Program; decision
2026-10-02 "3D Pick keeps its structure"). Build order is the order of risk.

### 5.1 Prove the pick on a pendant (before anything else)

Not new code. One day on the UR3e with the Pi on the cell cable:

- URCap 0.9.0 via the stick and the magic file (`TODO.md` 09-30, 10-01).
- Pi ↔ UR3e: Dashboard, RTDE, the DHCP lease, the pendant reaching :7621.
- Calibrate, teach, pick 20 blanks in a row through the node. Record misses.
- Measure what the datasheet estimates: part centre, heading, cycle to the grip.

Everything in this document is a promise until this day happens. **It is the launch
blocker; it is not tomorrow's blocker** because tomorrow we launch the beta and the plan.

### 5.2 The tending program template (`programs/MachineTend/`, URCap 1.0)

A PolyScope program the customer loads, with the I/O and poses as named variables at the
top. It uses only what PolyScope 5.4 has, so the matrix covers it.

```
Loop forever
  Open gripper                       ← customer's gripper node (Hand-E, OnRobot, pneumatic DO)
  3D Pick (tray)                     ← ours, as today
  If rs_pick_found
    Close gripper
    Lift
    Wait DI machine_door_open  (timeout → fault "door")
    MoveJ chuck_approach
    MoveL chuck_insert
    Set DO chuck_close ; Wait DI chuck_closed (timeout → fault "chuck")
    Open gripper ; MoveL chuck_retract
    Set DO door_close ; Set DO cycle_start
    Wait DI cycle_done  (timeout = 2 × taught cycle → fault "machine")
    Set DO door_open ; Wait DI machine_door_open
    MoveL chuck_insert ; Close gripper ; Set DO chuck_open ; Wait DI chuck_open
    MoveL chuck_retract ; MoveJ bin_drop ; Open gripper
  Else
    Popup "Tray empty — refill and press Continue"      ← this is the lights-out stop
    Halt
```

Deliverables:

- The template `.urp` + `.script`, built with `urctl/urp_builder.py` as script nodes (the
  real-robot rule, `CLAUDE.md` §Build motion programs with script nodes).
- An **I/O map screen** in the Installation node: six fields (door open, door close,
  chuck close, chuck closed, cycle start, cycle done) + a gripper choice. Generates the
  variables the template reads. Nothing else on the screen (operator-UI rule: ≤ 3 stages).
- **Fault vocabulary**: `tray_empty`, `door`, `chuck`, `machine`, `part_lost`, `protective_stop`,
  each a popup with the one sentence of what to do, and a `LOG` line to the Pi.
- Machine side: a dry-contact / 24 V I/O recipe for the three common cases (Haas, Fanuc
  0i/30i M-code DO + door-open input, a generic PLC) in `docs/machine-io.md`. The
  "M-code to open the door" is what every tending integrator sells first; it is a day of
  reading manuals, not code.

### 5.3 Unattended discipline (Lights-out)

- **Part-present proof.** After every close, read the gripper position (Hand-E reports it
  through the same socket `urctl gripper` uses; a pneumatic gripper gives a DI). A grip
  at full close = `part_lost`. Needs a `Gripper.status()` surface in the template.
- **Tray-empty stop** = `rs_pick_found == False` twice in a row from the picture point.
  Then halt with the popup. This is already what the node returns; the template makes it
  a clean stop instead of a loop.
- **Bin-full** = a part count the operator sets (`parts_per_bin`), or a DI from a bin
  sensor. Count first; sensors later.
- **Machine alarm** = a DI; stop, do not open the door, popup names the machine.
- **Protective stop** mid-cycle: the pick server already logs; the template leaves the
  part where it is and tells the operator which step it stopped in. No auto-resume into
  a machine.
- **Resume** = Continue on the pendant from the Tray-empty popup; from any other fault,
  the operator confirms the machine state first (a Yes/No popup per fault, the pattern
  `urctl/guided.py` uses).
- **Cycle log** on the Pi: one JSON line per part (time, pick number, pose, grip
  position, machine cycle time, fault). `audit.jsonl` exists; it needs rotation
  (`deploy/pi/README.md` §Open items) and this record type.

### 5.4 Alerts and the morning report (the only internet the Pi gets)

The Pi has no internet by design and stays that way on the cell port. The setup portal
(PR #59) already lets the operator put the Pi's *second* interface on the plant network.
Add to the portal: an SMTP relay or a webhook URL (Slack/Teams/Twilio), a test button.

- **Alert** on every fault: cell name, fault, step, the last picture (`/api/color.png`),
  the last 20 events. Sent by the cockpit process; nothing new listens.
- **Morning report** at a time the operator sets: parts done, faults, longest cycle,
  camera fps/temperature, free disk, version, "update available".
- **Us**: the same report, if the customer opts in, to a mailbox a routine reads. That is
  R1 and R2 without building a cloud.

### 5.5 Hardware for the shop floor

- **Pi 4 vs Pi 5**: the BOM says Pi 4; only a Pi 5 has run. Time the detector on a Pi 4
  (the datasheet's 0.9 s is a Pi 5 number) or change the BOM to the Pi 5 + its 5 A PSU.
  One afternoon. (`BOM.md` §Not verified.)
- **Enclosure**: the Pi in a DIN case in the customer's cabinet is fine; the D435 is not
  IP-rated. For coolant-mist cells, a printed shroud with a lens window is Rev D's job.
  Not for launch; say so on the datasheet ("dry cells").
- **Cable**: the Newnex screw-lock cable is single-source. Qualify the passive 5 m alternate
  (`BOM.md` O3) on the first build.
- **D435 supply**: 6–8 weeks and a tariff, and Cognex is buying RealSense (close expected
  Q4 2026). Hold 10 units of stock from the first revenue; qualify the D455 as the
  fallback (same SDK, same bracket face within a millimetre — verify) in Q1.

---

## 6. Timeline

**Tomorrow (launch day) — "the beta is public, the plan is public".**

1. `dev` → `main` (#61) once #55 and #59 are in. ✔ in progress tonight.
2. Tag `urcap5-v0.9.0` and `urcapx-v0.7.0` so the GitHub Releases match `dist/`.
3. `site/site.sh sync`: the any-robot page, 0.9.0 / 0.7.0 downloads, the Quickstart PDF,
   the one-page datasheet.
4. Publish the Pi image `imager.json` from a real source on `main` (today it is on a wip branch).
5. Beta terms on the site: free URCap, kit at the Find price, "Tend early access: install
   dates from <month>". One email address, one form.
6. This document reviewed by Nick → decisions in §10 made → moved to `docs/` as the plan of record.

**Week 1 — prove it (§5.1).** Plug the UR3e in. One day on the pendant with the Pi. Fix
what breaks (expect the magic-file install, the DHCP lease and a pendant layout surprise).
Reprint the datasheet with measured numbers where the estimates were.

**Weeks 2–3 — Tend on our own cell.** The template (§5.2), the I/O screen, the fault
vocabulary. No machine here: a "machine" is a box with a DI/DO breakout and a 20 s timer
(a UR DO looped to a DI through a relay is enough). Fifty cycles unattended on the bench.

**Weeks 4–6 — first Tend customer.** One shop, one machine, one part, a free install in
exchange for a week of logs and a reference. Lights-out alerts (§5.4) ship to them first.

**Weeks 6–10 — Lights-out tier.** Part-present, bin-full, machine alarm, morning report.
Second and third customers at the Tend price. The weekly routines take over H1/F1/S3.

**Quarter 2 — the second connector.** UFACTORY 850 on a real arm (the driver exists), then
whichever vendor the first ten customers ask for. The template is URScript today; the
second connector needs it expressed in that controller's language. That is when the
`Controller` protocol grows `set_digital_output` / `wait_digital_input` (it has neither
today, `urctl/controller.py`).

---

## 7. Acceptance tests (what "done" means at each gate)

| Gate | Test | Pass |
| --- | --- | --- |
| Kit burn-in (H4) | 1 h stream on the kit's own cable, doctor green | 30 fps sustained, no `Frame didn't arrive`, CPU temp < 70 °C |
| Pick on a pendant (§5.1) | 20 blanks of one size, random layout, 3 tray positions | ≥ 19 grips within ±3 mm of the taught offset; zero wrong-part grips; no protective stop |
| Tend on the bench (§5.2) | 50 cycles with the relay "machine" | 50/50; every forced fault (pull a wire) names itself correctly within 1 timeout |
| Tend at a customer (I8) | One shift attended, then one shift unattended | ≥ 95 % cycles complete; every stop was a named fault with a correct instruction |
| Lights-out (§5.3) | 3 consecutive nights | Zero unexplained stops; the morning report matched the bin |
| Field update (F5) | Push a bundle to a running cell | Live within 2 min, rollback on a deliberately broken bundle within 5 |

---

## 8. Risks, in order

1. **It has never run on a pendant.** Mitigation: §5.1 is week 1, before any customer date.
2. **Camera supply.** D435 lead 6–8 weeks, tariff, Cognex acquisition. Mitigation: stock
   10, qualify the D455, keep the bracket parametric (it is).
3. **Safety and liability.** The integrator owns the risk assessment (datasheet footer).
   Tend puts the robot's hand in a machine; the template must never open a door or start
   a cycle without the DI that proves the state. Put "the customer's risk assessment
   covers the tending program" in the Tend terms, and keep the machine-side I/O design
   with the customer's electrician, not us.
4. **Unauthenticated :7621/:7622 and unsigned update bundles** (Nick's decision, noted).
   Fine on an isolated cell port; the moment the plant interface goes on (§5.4), the
   portal login is the only lock. Minimum before Lights-out ships: a forced password
   change on first login, rate-limited; bundles stay checksum-only unless Nick says otherwise.
5. **One person.** Every row in §4 is Nick until a Builder exists. The routines (H1, F1,
   S3, R1) are how Nick does not become the bottleneck; the Builder is the first hire,
   part-time, once 5 kits a month ship.
6. **The Pi 4.** Untested; swap to the Pi 5 if the detector is > 2 s there. Cheap to decide.
7. **PolyScope X parity.** The X node is still the 0.8.0 layout. UR20 and new e-Series
   ship on X; a Find customer on X gets the older screen until parity (standing rule).

---

## 9. What launches tomorrow, exactly

- `main` = everything in `dev` tonight (3D Pick 0.9.0, pick PC image + setup portal,
  UFACTORY 850, bracket Rev C, any-robot site).
- Releases `urcap5-v0.9.0`, `urcapx-v0.7.0`, the Pi image on `imager.json`.
- perceptronics.advin.io: any-robot pitch, UR connector downloads, Quickstart PDF,
  one-page datasheet, **a Tend / Lights-out section that says "early access, installs from
  <date>"** and asks for robot, PolyScope version, machine, part.
- This file, reviewed.

Not tomorrow, and the site must not imply it: a pick verified on a pendant, a tending
program, alerts, or any cell running unattended.

---

## 10. Decisions for Nick (answer in place; this is the only list)

1. **Prices** (§3): Find kit $1,950? Tend install $2,500 + $150/mo? Lights-out +$100/mo?
   Or a single price and no subscription?
2. **Launch wording**: "beta" on the Find kit, "early access" on Tend — yes? What date
   goes on "installs from"?
3. **Pi 4 or Pi 5** for the kit (§5.5). My recommendation: Pi 5 + the 5 A PSU; the Pi 4
   saves $25 and costs a validation we have not done.
4. **Tend lives in the customer's program** (template, §5.2) — confirmed? The alternative
   is a second node ("Place") that does the machine side; it is nicer on screen and much
   more code, and it ties us to every machine's quirks.
5. **Alerts off the plant network** (§5.4): SMTP + webhook in the portal, opt-in copy to
   us — acceptable? This is the first time the Pi talks outbound.
6. **Forced password change on first portal login** before the plant interface is enabled
   — yes? (Bundles stay unsigned per your 2026-10-03 decision.)
7. **The first Tend customer**: a shop you already know, free install for logs and a
   reference. Who?
8. **Name**: "3D Pick" vs "Perceive" (TODO 10-02) — decide before the 1.0 URCap with the
   I/O screen, since the template and the docs will say it everywhere.
9. **Site deploy from CI** (S5) and **image build on a self-hosted runner** (F1): both are
   deploy-chain changes; draft PRs for you to merge, or leave manual for the first ten kits?
10. **The Builder** (§8.5): when, and is it a person or a contract print/assembly shop?

Everything else in this plan Claude can start on the morning after you answer, in the
order of §6, one PR per row of §4 that says "to automate".
