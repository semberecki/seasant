#!/usr/bin/env bash
# Rebuild every SesAnt part from the original RealAnt + Sesame files (expects ../../realant and ../../sesame-robot).
set -euo pipefail
cd "$(dirname "$0")"
[ -d sesame_step ] || { mkdir -p sesame_step; python scripts/step_export.py ../../sesame-robot/hardware/cad/Sesame-ESP32-v122.step sesame_step; }
python scripts/build.py work            # booleans -> work/asm.pkl
python scripts/checks.py work | tee ../assembly/verification_report.txt
python scripts/export.py work ..        # -> ../stl, ../assembly
python scripts/render_asm.py work && python scripts/print_sheet.py .. && python scripts/hero.py work ..
# v2: home puck (belly cone + charging funnel) -> ../v2 ; v3: StackChan neck + ant mandibles (needs v2) -> ../v3
python scripts/v2_puck.py work .. && python scripts/v2_render.py work ..
python scripts/v3_neck_jaw.py work .. && python scripts/v3_render.py work ..
# SesAnt Jumper: Jumper remix - two Jumper arms with two-jaw claws + MuJoCo model / Jumper training task (needs v2, v3) -> ../sesant_jumper
python scripts/jumper_arms.py work .. && python scripts/jumper_sim.py work .. && python scripts/jumper_render.py work .. && python scripts/jumper_animate.py work ..
python scripts/jumper_step.py work ..   # -> ../SesAnt Jumper.step (whole robot, one STEP assembly)
python scripts/print_sheet.py ../v2 && python scripts/print_sheet.py ../v3 && python scripts/print_sheet.py ../sesant_jumper
