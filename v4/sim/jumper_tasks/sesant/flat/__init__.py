"""sesant.flat -- flat-ground velocity tracking for SesAnt (the RealAnt-style walking task)."""

from __future__ import annotations

from ...registry import register
from ..common.assets import SESANT_ASSETS

register(
    id="sesant.flat",
    assets=SESANT_ASSETS,
    description="SesAnt quadruped (MG90S, 0.47 kg) flat-ground velocity tracking, legs only; neck and arms held in the carry pose",
    tags=("locomotion", "sesant", "flat"),
)
