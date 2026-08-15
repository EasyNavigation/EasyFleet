#!/usr/bin/env bash
set -euo pipefail

# Run this script from the vendor directory: easynav_tools/easynav_tools/vendor
# It will place textual/ into the current directory (vendor/)
PKG_VENDOR_DIR="$(cd "$(dirname "$0")" && pwd)"
TMPDIR=$(mktemp -d)
cleanup() { rm -rf "$TMPDIR"; }
trap cleanup EXIT

# set the textual version you want
TEXTUAL_V="6.2.1"

echo "Vendoring textual==${TEXTUAL_V} into $PKG_VENDOR_DIR"
python3 -m pip download "textual==${TEXTUAL_V}" --no-deps -d "$TMPDIR"

# extract everything into TMPDIR/src
mkdir -p "$TMPDIR/src"
for f in "$TMPDIR"/*; do
  case "$f" in
    *.tar.gz|*.tgz)
      tar -xzf "$f" -C "$TMPDIR/src"
      ;;
    *.zip|*.whl)
      # wheels and zip dists are ZIP-format
      unzip -q "$f" -d "$TMPDIR/src"
      ;;
    *)
      # ignore other files
      ;;
  esac
done

# find and copy textual package dir into vendor
found=0
# search for a directory named textual (maxdepth to avoid long scans)
while IFS= read -r -d '' pkgdir; do
  echo "Found textual source at: $pkgdir"
  rm -rf "$PKG_VENDOR_DIR/textual"
  cp -a "$pkgdir" "$PKG_VENDOR_DIR/textual"
  found=1
  # copy license files if present into textual dir
  find "$pkgdir" "$TMPDIR/src" -maxdepth 2 -type f -iname "LICENSE*" -print0 | while IFS= read -r -d '' lic; do
    cp -n "$lic" "$PKG_VENDOR_DIR/textual/" || true
  done
done < <(find "$TMPDIR/src" -maxdepth 4 -type d -name "textual" -print0)

if [ "$found" -eq 0 ]; then
  echo "ERROR: textual package directory not found in downloaded archives."
  echo "Contents of $TMPDIR:"
  ls -al "$TMPDIR" || true
  exit 2
fi

echo "Vendoring complete. Commit $PKG_VENDOR_DIR/textual (with LICENSE) into git."
