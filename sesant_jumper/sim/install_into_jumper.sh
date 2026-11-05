#!/usr/bin/env bash
# Install the SesAnt model and the sesant.* tasks into a Jumper checkout (github.com/KingKongRobotics/jumper).
#   usage: ./install_into_jumper.sh /path/to/jumper
# Copies:  sesant.xml + meshes/  -> <jumper>/assets/sesant/
#          jumper_tasks/sesant/   -> <jumper>/tasks/sesant/
# and adds one import line to <jumper>/tasks/__init__.py (the registry's "one line per task" rule).
set -euo pipefail
J="${1:?usage: $0 /path/to/jumper}"
HERE="$(cd "$(dirname "$0")" && pwd)"
[ -f "$J/tasks/registry.py" ] || { echo "not a Jumper checkout: $J"; exit 1; }
mkdir -p "$J/assets/sesant"
rm -rf "$J/assets/sesant/meshes" "$J/tasks/sesant"
cp "$HERE/sesant.xml" "$J/assets/sesant/" && cp -R "$HERE/meshes" "$J/assets/sesant/"
cp -R "$HERE/jumper_tasks/sesant" "$J/tasks/sesant"
grep -q "from .sesant import flat as sesant_flat" "$J/tasks/__init__.py" || \
  python3 - "$J/tasks/__init__.py" <<'PY'
import sys, re
p = sys.argv[1]; s = open(p).read()
last = list(re.finditer(r"^from \.jumper import .*$", s, re.M))[-1]
s = s[:last.end()] + "\nfrom .sesant import flat as sesant_flat  # noqa: F401,E402" + s[last.end():]
open(p, "w").write(s)
PY
echo "installed: python scripts/train.py --task sesant.flat   (CPU: --backend native --device cpu --num_envs 64)"
