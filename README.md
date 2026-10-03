# perceptronics

A depth camera on the robot's wrist finds the part; the **3D Pick** node on the
pendant puts the gripper on it. This page gets it onto a Universal Robots e-Series
(PolyScope 5).

## Install it: one USB stick

**1. Download these two files** (both, from the same place, don't rename them):

- [`perceptronic-ps5-0.9.0.urcap`](https://github.com/JimothyJohn/perceptronics/raw/main/urcap/dist/perceptronic-ps5-0.9.0.urcap)
- [`urmagic_perceptronic.sh`](https://github.com/JimothyJohn/perceptronics/raw/main/urcap/dist/urmagic_perceptronic.sh)

**2. Copy them onto a USB stick**, at the top, not in a folder. The stick must be
FAT32 (most sticks are). Eject it properly before you pull it out.

**3. On the pendant, once per robot:** ☰ → **Settings** → **Security** → **General**
→ turn on **Run magic files** and **USB ports**.

**4. Power the arm off** (the robot stays on) and stop any program. **Plug the stick
in.** The pendant shows **! USB !**, then the robot restarts by itself. That's the
install.

**5. Use it.** **Installation** → **URCaps** → **Perceptronic**: type the camera
computer's address. In your program: *your gripper's Open* → **3D Pick** → *your
gripper's Close*.

If nothing happens at step 4: the arm was on or a program was running (you'll get a
popup saying to restart; restart the robot), or the magic files setting is off. You
can always install by hand: ☰ → **Settings** → **System** → **URCaps** → **+** → pick
the `.urcap` → **Open** → **Restart**. What happened is written to
`urmagic_perceptronic.log` on the stick.

The camera computer (a Raspberry Pi with the D435 on its USB port) is set up once:
[docs/pick-kit.md](docs/pick-kit.md). PolyScope X has no magic files; its URCap
installs through System Manager: [urcap/README.md](urcap/README.md).

## Developers: install from a PC over SSH

Same two files, same script, no stick: copy them to the robot and run the script
there. Your PC must be on the robot's network.

**Turn SSH on (PolyScope 5.10 and later):** ☰ → **Settings** → **Security** →
**Secure Shell** → enter the admin password → tick **Enable SSH Access**.

**Log in as `root`.** The factory default password is **`easybot`**
([UR: resetting passwords](https://www.universal-robots.com/articles/ur/robot-care-maintenance/resetting-passwords/)).
If it was changed, ask whoever owns the robot. Put your key on it so you never type
the password again (`ssh-copy-id`, or **Secure Shell** → **Manage Authorized Keys** on
the pendant), and change the default with `passwd`
([UR: secure setup](https://www.universal-robots.com/articles/ur/cybersecurity/secure-setup-of-ur-cobots/)):

```bash
ROBOT=192.168.1.50                       # your robot's IP address
ssh-copy-id root@$ROBOT                  # once: key login from now on
```

**Install** from a checkout of this repo (or wherever you downloaded the two files):

```bash
ssh root@$ROBOT 'mkdir -p /tmp/perceptronic'
scp urcap/dist/perceptronic-ps5-0.9.0.urcap urcap/dist/urmagic_perceptronic.sh root@$ROBOT:/tmp/perceptronic/
ssh root@$ROBOT 'bash /tmp/perceptronic/urmagic_perceptronic.sh'
```

The script checks the file's checksum, puts it where PolyScope's own URCaps screen
would, and restarts the controller when the arm is off and no program runs; otherwise
it tells you to restart. `URMAGIC_RESTART=always` (before `bash`) restarts regardless,
`=never` leaves it to you. Its log is `/tmp/perceptronic/urmagic_perceptronic.log`.

**After changing the URCap:** `make urcap5-package` rebuilds `urcap/dist/` (the `.urcap`
and the `.sh` with its new checksum), then run the three lines above again.
`scripts/urcap5-usb.sh` writes a stick from a Mac without the `._` files Finder leaves.

## More

- [docs/architecture.md](docs/architecture.md): how it works, the code map, the cockpit's API, adding a robot, running it all without hardware
- [urcap/perceptronic-ps5/README.md](urcap/perceptronic-ps5/README.md): the PolyScope 5 URCap in full: upgrading, every screen
- [docs/pick-kit.md](docs/pick-kit.md): the kit, the camera computer, setting up a cell
- [docs/realsense.md](docs/realsense.md): the camera, depth quality, hand-eye
- [CLAUDE.md](CLAUDE.md): working notes, protocols, and the gotchas that cost real time

Working on perceptronics itself: `make install-dev`, `make test`, `make lint`; PRs go
to `dev` ([docs/architecture.md](docs/architecture.md#development)).
