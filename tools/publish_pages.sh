#!/usr/bin/env bash
# Publish the 3D viewer to GitHub Pages: https://youngmoolahbaby.github.io/custom-skate/
#
# The page (viewer/boot.html) and the four builds' data go on the gh-pages branch, replaced each
# time with a single commit, so the ~60 MB of mesh data never piles up in the repo's history.
# GitHub can't build it (the scans aren't in git), so it's published from here after a build:
#
#   for cfg in "out/boot ice open" "out/boot_roller roller open" "out/boot_v ice v" "out/boot_roller_v roller v"; do
#     set -- $cfg; BOOT_VIEWER=1 cargo run --release -- boot scans/stand-in/022_fBW_R.ply $1 $2 $3
#   done
#   tools/publish_pages.sh
#
# Publish the stand-in foot only: the page is public, and a boot built on Jon's own scan carries
# his foot's shape.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo"

files=(
  out/boot/viewer/boot.txt out/boot/viewer/manifest.json out/boot/viewer/under.txt
  out/boot_roller/viewer/boot_roller.txt out/boot_roller/viewer/manifest_roller.json out/boot_roller/viewer/under_roller.txt
  out/boot_v/viewer/boot_v.txt out/boot_v/viewer/manifest_v.json
  out/boot_roller_v/viewer/boot_roller_v.txt out/boot_roller_v/viewer/manifest_roller_v.json
)
for f in "${files[@]}"; do
  [ -f "$f" ] || { echo "missing $f: build all four first (see the top of this script)"; exit 1; }
  size=$(wc -c < "$f")
  [ "$size" -lt 50000000 ] || { echo "$f is $size bytes: GitHub warns over 50 MB and refuses over 100 MB"; exit 1; }
done
# Both throats of a skate share its holder or chassis file, so the two builds must agree.
cmp -s out/boot/viewer/under.txt out/boot_v/viewer/under.txt || { echo "the ice holder differs between the two throats' builds: rebuild them"; exit 1; }
cmp -s out/boot_roller/viewer/under_roller.txt out/boot_roller_v/viewer/under_roller.txt || { echo "the roller chassis differs between the two throats' builds: rebuild them"; exit 1; }

site="$(mktemp -d)"
trap 'rm -rf "$site"' EXIT
# viewer/boot.html is the page's body and head contents; a standalone page needs its skeleton.
{
  printf '<!doctype html>\n<html lang="en">\n<meta charset="utf-8">\n'
  printf '<meta name="viewport" content="width=device-width, initial-scale=1">\n'
  printf '<meta name="description" content="A custom hockey skate from a foot scan, in 3D: the boot with its ice holder or roller chassis, part by part.">\n'
  cat viewer/boot.html
} > "$site/index.html"
for f in "${files[@]}"; do cp "$f" "$site/"; done
touch "$site/.nojekyll"

rev="$(git rev-parse --short HEAD)"
dirty=""
git diff --quiet HEAD -- src viewer || dirty=" (with uncommitted changes)"
git -C "$site" init -q -b gh-pages
git -C "$site" config user.name "$(git config user.name)"
git -C "$site" config user.email "$(git config user.email)"
git -C "$site" config core.autocrlf false
git -C "$site" add -A
msg="Viewer from main $rev$dirty"
[ -n "${PAGES_TRAILER:-}" ] && msg="$msg"$'\n\n'"$PAGES_TRAILER"
git -C "$site" commit -q -m "$msg"
git -C "$site" push -q -f "$(git remote get-url origin)" gh-pages
echo "published to gh-pages from $rev$dirty: https://youngmoolahbaby.github.io/custom-skate/"
