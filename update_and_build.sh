#!/usr/bin/env bash  
# update_and_build.sh - update repozitara a build ArduPilot Rover pre Pixhawk1  
set -euo pipefail  
  
REPO=/home/kenai/ardupilot-4.7.Custom  
BOARD=Pixhawk1  
BRANCH=main             # over cez: git branch -a  
  
cd "$REPO"  
  
# lokalne zmeny: len varovanie, nic sa stashuje ani nemerguje  
if [ -n "$(git status --porcelain)" ]; then  
    echo "!!! VAROVANIE: lokalne zmeny v repozitari:"  
    git status --short  
    echo "!!! Pokracujem - pull moze zlyhat na konflikte."  
fi  
  
echo "=== Fetch + pull z origin/$BRANCH ==="  
git fetch origin  
git pull --rebase origin "$BRANCH"  
  
echo "=== Submoduly ==="  
git submodule update --init --recursive  
  
echo "=== Configure: $BOARD + extra_hwdef.dat ==="  
./waf configure --board "$BOARD" --extra-hwdef extra_hwdef.dat  
  
echo "=== Build: rover ==="  
./waf rover  
  
echo "=== Hotovo ==="  
echo "Vystup: $REPO/build/$BOARD/bin/ardurover.apj"
