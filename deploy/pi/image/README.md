# The pick PC image

One image that you flash to any new pick PC: Raspberry Pi OS Lite (64-bit) with librealsense,
the cockpit service, the udev rules and the firewall already installed. Each board gets its
own identity from a seed written to the card after flashing. The seed sets the host name,
the user and SSH key, the cell address, and the robot. This replaces the first deploy's
10-minute librealsense compile and its need for internet on the PC.

```
laptop                               Pi (any arm64 Debian; the pick PC itself is fine)
scripts/pi-image.sh build nick@pi ─▶ deploy/pi/image/build.sh (sudo)
                                       base image (pinned, sha256) → grow → chroot
                                       → install.sh --image → generalise → shrink → xz
target/pi-image/*.img.xz ◀── scp ───
scripts/pi-image.sh flash … diskN  ─▶ SD card
scripts/pi-image.sh seed /Volumes/bootfs --hostname … ─▶ user-data, network-config, meta-data
```

## Build

    scripts/pi-image.sh build nick@10.0.0.56 [--cell ur3] [--robot-host 192.168.3.3]

The Pi needs to be aarch64 with passwordless sudo (or run the command from a terminal so sudo
can prompt), about 12 GB free and internet access. `~/pi-image/cache` on the Pi keeps the base
download and a tarball of the built librealsense, so only the first build compiles it. The
image, its `.sha256` and a `.manifest.txt` land in `target/pi-image/`. The manifest lists the
base image, the wheel, the librealsense stamp and every package with its version. Diff two
manifests to see what changed between builds.

Every board made from the image has `/etc/perceptronics/image-release`, which says which build
it came from.

**What the build does not touch:** the build host's firewall and services. `install.sh --image`
enables the units without starting anything, `policy-rc.d` stops package scripts from starting
daemons in the chroot, and the librealsense compile runs on the host's disk (a bind mount), not
in the image.

**What it removes:** SSH host keys (regenerated on first boot), the apt lists and cache, the
logs, and the build's `resolv.conf`. `machine-id` stays `uninitialized` as Raspberry Pi ships
it, so each board makes its own. Root is shrunk to its contents + 256 MiB, and the first
boot's `resize` (in `cmdline.txt`) grows it to fill the card.

## Flash (macOS)

    diskutil list external
    scripts/pi-image.sh flash target/pi-image/<name>.img.xz disk4

The flash command refuses the Mac's boot disk and anything that isn't removable or external.
It shows the disk and asks you to type its id before it erases it. It writes with `sudo dd`,
so run it in a terminal. Raspberry Pi Imager's "Use custom" works too, but **don't** apply
Imager's OS customisation: it writes the same three files the seed does.

## Seed: what makes the card one board

    scripts/pi-image.sh seed /Volumes/bootfs --hostname pickpc2 --user nick \
        --ssh-key ~/.ssh/id_ed25519.pub --address 192.168.3.21/24 \
        --robot-host 192.168.3.3 [--cell ur3] [--wifi-ssid NAME]
    diskutil eject /Volumes/bootfs

| Flag | Becomes |
| --- | --- |
| `--hostname` | the host name (`<name>.local` over mDNS, avahi is in the image) |
| `--user`, `--ssh-key` | the login user, **SSH key only** (no password; `ssh_pwauth: false`), passwordless sudo like Raspberry Pi OS's default user |
| `--address` | eth0's static address. Omit it for DHCP. `--gateway` / `--dns` only if the network has a way out; the cell normally has none |
| `--wifi-ssid` | optional Wi-Fi. The password comes from `PERCEPTRONICS_WIFI_PSK` or a prompt, never the command line |
| `--robot-host`, `--cell`, `--allow-from` | on the first boot, `install.sh --reconfigure` rewrites `/etc/perceptronics/cell.env` and the firewall for this robot (offline: the wheel is in the image). Log: `/var/log/perceptronics-seed.log` |

Without the robot flags the board keeps the image's default `cell.env` (the cell it was built
with). Every value is validated, and the files are written as JSON (valid YAML), so nothing
typed can add a key or a command. `cloud-init schema` accepts both files (checked on the Pi,
2026-10-02). Each seed gets a new `instance-id`, so re-seeding a card that has already booted
runs the first-boot setup again.

## Checking a fresh board

    ssh nick@192.168.3.21 'cat /etc/perceptronics/image-release; cloud-init status --long; sudo perceptronics-doctor'

## Iteration log

Each rewrite of the test board gets one entry: what was built, what happened on the board,
and what changed because of it.

| # | Date | Build | Result on the board | Lesson / change |
| --- | --- | --- | --- | --- |
