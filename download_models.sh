#!/bin/sh
set -eu

destination=${1:?"usage: $0 /path/to/sd-card"}/models/s3
revision=165e966ea69410c332f9cafc452a15d3784436b0
base=https://raw.githubusercontent.com/espressif/esp-dl/$revision/models
mkdir -p "$destination"

checksum() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | cut -d ' ' -f 1
  else
    shasum -a 256 "$1" | cut -d ' ' -f 1
  fi
}

download() {
  source=$1
  name=$2
  expected=$3
  temporary="$destination/.$name.tmp"
  curl -fL --retry 3 -o "$temporary" "$base/$source/models/s3/$name"
  actual=$(checksum "$temporary")
  if [ "$actual" != "$expected" ]; then
    rm -f "$temporary"
    echo "checksum mismatch for $name" >&2
    exit 1
  fi
  mv "$temporary" "$destination/$name"
}

download pedestrian_detect pedestrian_detect_pico_s8_v1.espdl 3eb5827e12322728fe99ea9c86d71ea38aa5a5b70064abf7d75bc90140b74015

echo "Installed pedestrian model in $destination"
