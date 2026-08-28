#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/images_fake"
DST_DIR="$SCRIPT_DIR/images"

mkdir -p "$DST_DIR"

mapfile -d '' files < <(find "$SRC_DIR" -mindepth 1 -maxdepth 1 -type f -print0)

if [[ ${#files[@]} -eq 0 ]]; then
	echo "No files to copy from $SRC_DIR"
	exit 0
fi

for f in "${files[@]}"; do
	cp -f "$f" "$DST_DIR/"
done

echo "Copied ${#files[@]} file(s) from $SRC_DIR to $DST_DIR"
