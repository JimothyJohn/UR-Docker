# The cockpit on Windows 11 with Docker Desktop

One script takes a Windows 11 laptop with Docker Desktop from a fresh clone to the
RGB-D cockpit in the browser, RealSense included. Nothing else is installed on
Windows: no Python, no RealSense SDK. The image is the Jetson's
(`Dockerfile.perceptronics`: librealsense v2.58.4, libusb backend); what is
Windows-specific is getting the camera into it and the ports out.

```
D435 --USB--> Windows --usbipd-win--> WSL 2 kernel --/dev/bus/usb--> container
                                                                     perceptronics-gui
browser, pendant, URCap <-- 127.0.0.1 (or -Lan) :7621 / :7622 <------+
```

**Status, 2026-09-30:** built and run on the Mac (arm64) with the synthetic camera:
image builds, the cockpit and the pick server answer on the published ports, and the
launcher's logic was exercised under PowerShell 7 against stand-in `docker` / `usbipd`
commands. **Not yet run on a Windows machine, and no camera has streamed through
usbipd into this container.** The D435 itself streams under WSL 2 (verified
2026-09-23, outside Docker). The first run on the laptop is the verification - if a
step fails, the script's `FAILED:` line and `-Status` output are what to send back.

## You need

- Windows 11, Docker Desktop on the **WSL 2 engine** (the default; Settings -> General),
  in Linux-containers mode.
- The D435 on a USB 3 port, on its own cable. Close Intel RealSense Viewer.
- This repository (`git clone`), a PowerShell window in it.

## Three steps

```powershell
# 1. Docker only, synthetic camera. The first build compiles librealsense (~10 min), then it is cached.
powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1 -Fake

# 2. The real camera. Installs usbipd-win and shares the camera: two UAC prompts, once.
powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1

# 3. Your robot.
powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1 -Cell mycell -Lan
```

Each ends by printing the camera, the frame rate and the robot it talks to, and opens
<http://localhost:7621>. Do them in order: each one adds exactly one thing that can fail.

Step 2 takes the camera away from Windows (one owner at a time) until `-Down`.

### Step 3: your cell

Copy `deploy\windows\cell.env.example` to `deploy\windows\mycell.env` (any name;
git-ignored), fill in `UR_HOST` and the tool length. `-Cell ur3` instead uses the
shipped UR3e profile **including Nick's hand-eye solve and webcams**, which is right
only on Nick's bench. `-RobotHost 192.168.3.3` overrides the address for one run;
`-DryRun` validates and audits robot actions and sends nothing.

Calibrate before trusting a pick: cockpit `/classic` -> Calibrate hand-eye -> Apply.
The solve lands in `captures\calibration\handeye_mycell.json` on the Windows side and
survives rebuilds.

`-Lan` is what lets the pendant and the URCap reach the cockpit: it publishes `:7621`
(HTTP) and `:7622` (the Perceptronic Pick node's socket) on every interface instead of
`127.0.0.1` and adds one Windows Firewall rule admitting them **from the local subnet
only** (a UAC prompt, once). The cockpit has no login and moves a robot: use `-Lan` on
the cell network, not on the office Wi-Fi. The URCap's Cockpit field is
`http://<this PC's address on the robot's subnet>:7621`; the script prints the candidates.

## Without the script

Where policy blocks `.ps1` files, type the same commands yourself: execution policy
stops script files, not commands entered at the prompt. This is all the script does.

```powershell
# once, in an ADMINISTRATOR terminal
winget install --id dorssel.usbipd-win -e     # or the .msi from github.com/dorssel/usbipd-win/releases
usbipd list                                   # note the RealSense's BUSID, e.g. 2-3
usbipd bind --busid 2-3

# every start (and after a reboot or re-plug), in a normal terminal in the repository
usbipd attach --wsl --busid 2-3               # Docker Desktop must be running
$env:COCKPIT_ARGS = "--no-robot"              # camera only; leave it out once UR_CELL is set
docker compose -f docker-compose.windows.yml up -d --build
```

Then open <http://localhost:7621>. For the robot and the pendant, set these before `up`
instead of `COCKPIT_ARGS`:

```powershell
$env:UR_CELL = "/cells/mycell.env"            # deploy\windows\mycell.env, or a shipped name: ur3
$env:PERCEPTRONICS_PUBLISH = "0.0.0.0"        # what -Lan does; default is 127.0.0.1
# once, administrator - the firewall half of -Lan:
New-NetFirewallRule -DisplayName "perceptronics cockpit" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 7621,7622 -RemoteAddress LocalSubnet
```

`$env:PERCEPTRONICS_FAKE = "1"` is `-Fake` (skip the usbipd lines). Stop with
`docker compose -f docker-compose.windows.yml down` and `usbipd detach --busid 2-3`.
After the first `up` the container (`perceptronics-windows`) is in Docker Desktop's
Containers list and can be stopped, started and its log read there - but Docker Desktop
has no button for the USB step: `usbipd attach` is typed each time. If the cockpit was
already running when you attached, restart the container.

## Day to day

| | |
| --- | --- |
| start / after a reboot / after re-plugging the camera | re-run the same command (an attach survives neither) |
| what is going on | `scripts\docker-windows.ps1 -Status` |
| the cockpit's log | `scripts\docker-windows.ps1 -Logs` |
| stop, give the camera back to Windows | `scripts\docker-windows.ps1 -Down` |
| after `git pull` | re-run the same command (it rebuilds what changed) |

The container restarts with Docker Desktop (`restart: unless-stopped`), but the camera
is not re-attached by itself after a reboot: re-run the script.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `Docker Desktop is not on the WSL 2 engine` | Hyper-V backend: usbipd attaches to WSL, not to the Hyper-V VM | Settings -> General -> *Use the WSL 2 based engine* |
| `no RealSense in the list above` | not plugged in, or Windows names it differently | `usbipd list`, then `-BusId 2-3` |
| `usbipd attach exited ...` | usbipd's own message is printed above the line. Usual ones: a camera app holds the device; an old WSL kernel; a third-party firewall blocking TCP 3240 between WSL and Windows | close RealSense Viewer / Teams / Camera; `wsl --update`; allow 3240 for the WSL adapter |
| `the cockpit is up but the camera delivers no frames` | the container started before the camera was attached, or the link is marginal | re-plug, re-run the script; `-Logs` shows librealsense's error |
| picture at 640x480 @ 15 | the camera enumerated as USB 2 (cable, hub, port): the D435 offers no 848x480 colour there and the cockpit takes the best pair it lists | a USB 3 port and the camera's own cable; `-Status` shows `usb 2.x` / `3.x` |
| the pendant or the URCap cannot reach `:7621` | started without `-Lan`, or this PC has no address on the robot's subnet, or a third-party firewall | `-Lan`; `ping` the controller from Windows; `-Status` |
| every click reads OUT OF REACH / `reach_check: sphere` | the container cannot reach the controller's Primary port | `UR_HOST` in your cell file; from Windows: `Test-NetConnection <robot> -Port 30001` |
| port 7621 already in use | a native cockpit (`scripts\cockpit.ps1`) is running | stop it; one cockpit per camera |
| no webcam panels | the image has no ffmpeg and Docker gets only the RealSense | by design here; the extra views are a native-install feature |

The simulators are a separate compose file: `docker compose up -d ursim` (the e-Series
sim is an amd64 image and runs natively on this laptop), then
`-RobotHost host.docker.internal` with an e-series cell points the cockpit at it
(not yet tried).

## What it leaves on the machine

usbipd-win (winget `dorssel.usbipd-win`, a Windows service), the camera marked *Shared*
in usbipd (`usbipd unbind --busid <id>` undoes it), with `-Lan` the firewall rule
`perceptronics cockpit (7621-7622, local subnet)`, the Docker image
`perceptronics/perceptronics:local` and the `captures\` folder.
