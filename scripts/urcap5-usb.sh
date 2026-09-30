#!/usr/bin/env bash
# Put the current URCaps on a USB stick for the pendant, safely: no macOS "._"
# AppleDouble files (PolyScope's URCap picker would list "._perceptronic-….urcap" and
# fail on it), checksum-verified, then ejected. On the stick afterwards:
#
#   perceptronic-ps5-<ver>.urcap      the PolyScope 5 URCap
#   urmagic_perceptronic.sh        installs it by itself when the stick goes into a
#                                     PolyScope 5 robot with "Run magic files" enabled
#                                     (scripts/urmagic_perceptronic.sh, filled in)
#   perceptronic-<ver>.urcapx         the PolyScope X URCap, for System Manager (PolyScope X
#                                     runs nothing from a stick)
#
#   scripts/urcap5-usb.sh                 # the stick named "URE MODELS"
#   scripts/urcap5-usb.sh "MY STICK"      # another FAT32 stick
#   URCAP5_USB_MAGIC=0 scripts/urcap5-usb.sh   # no magic file: install by hand
#
# By hand on the pendant: ☰ → Settings → System → URCaps → + → the file → Open → Restart.
set -euo pipefail
cd "$(dirname "$0")/.."

volume="/Volumes/${1:-URE MODELS}"
urcap="$(find urcap/dist -maxdepth 1 -name 'perceptronic-ps5-*.urcap' | sort | tail -1)"
[[ -d "$volume" ]] || { echo "no stick at $volume — plug it in (or pass its name)"; exit 1; }
[[ -f "$urcap" ]] || { echo "no $urcap — make urcap5-package"; exit 1; }

urcapx="$(find urcap/dist -maxdepth 1 -name 'perceptronic-*.urcapx' | sort | tail -1)"
magic_src="scripts/urmagic_perceptronic.sh"
symbolic="$(sed -n 's/^Bundle-SymbolicName=//p' urcap/perceptronic-ps5/bundle.properties)"
[[ -n "$symbolic" ]] || { echo "no Bundle-SymbolicName in urcap/perceptronic-ps5/bundle.properties"; exit 1; }

# replace any earlier build of the URCaps (and the magic file + its log) on the stick
find "$volume" -maxdepth 1 \( -name 'perceptronic-ps5-*.urcap' -o -name '._perceptronic-ps5-*' \
  -o -name 'perceptronic-*.urcapx' -o -name '._perceptronic-*.urcapx' \
  -o -name 'urmagic_perceptronic.*' -o -name '._urmagic_perceptronic.*' \) -delete
cp -X "$urcap" "$volume/"
[[ -f "$urcapx" ]] && cp -X "$urcapx" "$volume/"
dot_clean -m "$volume" 2>/dev/null || true
sync

want="$(shasum -a 256 "$urcap" | cut -d' ' -f1)"
got="$(shasum -a 256 "$volume/$(basename "$urcap")" | cut -d' ' -f1)"
[[ "$want" == "$got" ]] || { echo "checksum mismatch on the stick — try again"; exit 1; }
if find "$volume" -maxdepth 1 -name '._*' | grep -q .; then
  echo "warning: ._ files remain on the stick"
fi
echo "copied $(basename "$urcap") (sha256 ${want:0:12}…) to $volume"
[[ -f "$urcapx" ]] && echo "copied $(basename "$urcapx") (PolyScope X: install it through System Manager)"

if [[ "${URCAP5_USB_MAGIC:-1}" == 1 ]]; then
  # the magic file, with this build's name, sha256 and bundle id filled in
  sed -e "s|@URCAP_FILE@|$(basename "$urcap")|" -e "s|@URCAP_SHA256@|$want|" \
    -e "s|@SYMBOLIC_NAME@|$symbolic|" "$magic_src" > "$volume/urmagic_perceptronic.sh"
  dot_clean -m "$volume" 2>/dev/null || true
  sync
  echo "wrote urmagic_perceptronic.sh: a PolyScope 5 robot with Settings → Security → General →"
  echo "  'Run magic files' on installs the URCap by itself when the stick goes in (log on the stick)"
fi
diskutil eject "$volume" >/dev/null && echo "ejected — take it to the pendant"
