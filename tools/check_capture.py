#!/usr/bin/env python3
"""Quick validation for hoverboard 50-Hz telemetry CSV captures.

Usage:
    python3 tools/check_capture.py hover_YYYYMMDD_HHMMSS_modeX.csv

This does not 'correct' ADC values. It separates bridge-valid samples from hard-
disabled/floating samples and reports raw current channels, STOP behavior, and
mode-independent sensor activity.
"""
from __future__ import annotations
import csv, statistics, sys
from pathlib import Path

RAW = ("dcl_raw", "rla_raw", "rlb_raw", "dcr_raw", "rrb_raw", "rrc_raw")
PHASE = ("rla_raw", "rlb_raw", "rrb_raw", "rrc_raw")

def num(row, key, default=0.0):
    try: return float(row.get(key, default) or default)
    except (TypeError, ValueError): return float(default)

def med(rows, key):
    vals=[num(r,key) for r in rows if r.get(key,"")!=""]
    return statistics.median(vals) if vals else float('nan')

def main(path: str) -> int:
    p=Path(path)
    with p.open(newline="", encoding="utf-8") as f:
        rows=list(csv.DictReader(f))
    if not rows:
        print("FAIL: CSV has no rows"); return 2

    mode=rows[0].get("mode", "legacy")
    has_valid="adc_current_valid" in rows[0]
    valid=[r for r in rows if (not has_valid or int(num(r,"adc_current_valid")) == 1)]
    stopped=[r for r in valid if int(num(r,"cmdL")) == 0 and int(num(r,"cmdR")) == 0]

    print(f"file={p.name} rows={len(rows)} mode={mode} valid_adc_rows={len(valid)} stopped_valid_rows={len(stopped)}")
    for k in RAW:
        vals=[num(r,k) for r in valid if r.get(k,"")!=""]
        if vals:
            print(f"{k:8s} median={statistics.median(vals):7.1f} min={min(vals):7.1f} max={max(vals):7.1f}")
    if stopped:
        print("STOP valid-phase medians:", " ".join(f"{k}={med(stopped,k):.0f}" for k in PHASE))
        bad=[(k,med(stopped,k)) for k in PHASE if not (1000 <= med(stopped,k) <= 3000)]
        print("STOP_ADC_WINDOW=" + ("WARN " + str(bad) if bad else "PASS"))
    else:
        print("STOP_ADC_WINDOW=NO_VALID_STOP_ROWS")

    for hall in ("hallL","hallR"):
        if hall in rows[0]:
            unique=sorted({r.get(hall,"") for r in rows})
            print(f"{hall}: unique={unique[:12]} count={len(unique)}")
    for rpm in ("rpmL","rpmR","encoder_rpm"):
        if rpm in rows[0]:
            nz=sum(1 for r in rows if abs(num(r,rpm)) > 0)
            print(f"{rpm}: nonzero_rows={nz}/{len(rows)} median={med(rows,rpm):.1f}")

    if has_valid:
        l=sum(1 for r in rows if int(num(r,"adc_left_valid")) == 1) if "adc_left_valid" in rows[0] else -1
        rr=sum(1 for r in rows if int(num(r,"adc_right_valid")) == 1) if "adc_right_valid" in rows[0] else -1
        print(f"ADC validity: both={len(valid)}/{len(rows)} left={l} right={rr}")
    else:
        print("NOTE: legacy CSV has no adc_current_valid flag; floating samples cannot be filtered automatically.")
    return 0

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__.strip()); raise SystemExit(2)
    raise SystemExit(main(sys.argv[1]))
