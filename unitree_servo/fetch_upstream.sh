#!/usr/bin/env bash
# Fetch the third-party material this folder references but does not commit:
#   upstream/            Unitree's digital_servo v1.0.1 (the code under test)
#   docs/*.pdf           the two official manuals
#
# Idempotent: skips anything already present. unitree.com's image URLs are flaky
# through a proxy — they may answer 000 on the first attempt and 200 on the next,
# hence the retries. If github.com or unitree.com time out, bring the proxy up first
# (`clashon`, see the network-proxy skill) and run this again.
set -euo pipefail
cd "$(dirname "$0")"

TAG_URL="https://github.com/unitreerobotics/digital_servo/archive/refs/tags/v1.0.1.zip"
S288_DOC="https://www.unitree.com/images/%E6%97%A0%E5%88%B7%E6%95%B0%E5%AD%97%E8%88%B5%E6%9C%BAJ288S288%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C.pdf"
DEBUG_DOC="https://www.unitree.com/images/%E7%94%B5%E6%9C%BA%E8%B0%83%E8%AF%95%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C.pdf"

fetch() {  # fetch <url> <out> [attempts]
    local url="$1" out="$2" tries="${3:-3}" code i
    if [ -s "$out" ]; then echo "have  $out"; return 0; fi
    for i in $(seq 1 "$tries"); do
        code=$(curl -sL --retry 2 -m 180 -A "Mozilla/5.0" -w '%{http_code}' -o "$out" "$url" || echo 000)
        if [ "$code" = "200" ] && [ -s "$out" ]; then
            echo "got   $out  ($(stat -c%s "$out") bytes, attempt $i)"; return 0
        fi
        echo "retry $out (http=$code, attempt $i/$tries)"; sleep 2
    done
    echo "FAILED $out" >&2; return 1
}

if [ ! -d upstream/digital_servo-1.0.1 ]; then
    fetch "$TAG_URL" /tmp/digital_servo-v1.0.1.zip
    mkdir -p upstream
    unzip -q -o /tmp/digital_servo-v1.0.1.zip -d upstream
    # The Keil build products and the STM32 HAL sources are 108 MB of the 109 MB
    # archive and nothing here reads them. App/ (protocol.c/h, crc_ccitt.h) and
    # Core/ are what the report quotes, so those stay.
    rm -rf upstream/digital_servo-1.0.1/stm32/firmware/Drivers \
           upstream/digital_servo-1.0.1/stm32/firmware/MDK-ARM \
           upstream/digital_servo-1.0.1/python/*.docx \
           upstream/digital_servo-1.0.1/stm32/*.docx
    echo "got   upstream/digital_servo-1.0.1 (trimmed to App/Core/python/specs)"
else
    echo "have  upstream/digital_servo-1.0.1"
fi

mkdir -p docs
fetch "$S288_DOC"  docs/s288_manual.pdf
fetch "$DEBUG_DOC" docs/debug_manual.pdf

echo
echo "now runnable:  python3 verify_official_issues.py"
