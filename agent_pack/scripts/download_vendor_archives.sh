#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
OUT="$ROOT/raw_vendor_archives"
mkdir -p "$OUT"

fetch() {
  local url="$1"
  local name="$2"
  echo "Fetching $name"
  if command -v curl >/dev/null 2>&1; then
    curl -L --fail --retry 3 -o "$OUT/$name" "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "$OUT/$name" "$url"
  else
    echo "Need curl or wget" >&2
    exit 1
  fi
}

fetch 'https://files.waveshare.com/upload/e/e6/WAVE_ROVER_demo.zip' 'WAVE_ROVER_demo.zip'
fetch 'https://files.waveshare.com/upload/5/51/WAVE_ROVER_DXF.rar' 'WAVE_ROVER_DXF.rar'
fetch 'https://files.waveshare.com/upload/b/b4/WAVE_ROVER_PDF.rar' 'WAVE_ROVER_PDF.rar'
fetch 'https://files.waveshare.com/upload/e/ef/WAVE_ROVER-EP_DXF.rar' 'WAVE_ROVER-EP_DXF.rar'
fetch 'https://files.waveshare.com/upload/a/ab/WAVE_ROVER-EP_PDF.rar' 'WAVE_ROVER-EP_PDF.rar'
fetch 'https://files.waveshare.com/upload/e/ec/WAVE_ROVER_MODEL_STL.rar' 'WAVE_ROVER_MODEL_STL.rar'
fetch 'https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf' 'General_Driver_for_Robots.pdf'
fetch 'https://files.waveshare.com/upload/0/0c/UGV01_BASE.zip' 'UGV01_BASE.zip'
fetch 'https://files.waveshare.com/upload/5/50/GENERAL-DRIVER-FOR-ROBOTS-STR-DXF.zip' 'GENERAL-DRIVER-FOR-ROBOTS-STR-DXF.zip'
fetch 'https://files.waveshare.com/upload/0/0a/GENERAL-DRIVER-FOR-ROBOTS-STR-PDF.zip' 'GENERAL-DRIVER-FOR-ROBOTS-STR-PDF.zip'
fetch 'https://files.waveshare.com/upload/8/8e/General_Driver_for_Robots_STEP.zip' 'General_Driver_for_Robots_STEP.zip'

echo
ls -lh "$OUT"
