# TeamB_mrsd_24 — Contribution Summary

**Repository:** github.com/LifGorg/TeamB_mrsd_24  
**Date:** December 2025  
**Methodology:** `git log --all --numstat` (excluding generated files, binaries, third-party libraries)

---

## Raw Git Data (Lines Added)

```
   85731  Yufan Liu
   61758  LifGorg
   25932  Jet Situ
    5948  lifgorg
    4663  situjet
    4204  airlab
    3141  jaswu51
    1710  joshp1225
```

## Consolidated by Identity

| Contributor | Aliases | Lines Added | Share |
|-------------|---------|-------------|-------|
| **Yufan Liu** | `Yufan Liu`, `LifGorg`, `lifgorg`, `airlab` | 157,641 | 82% |
| **Jet Situ** | `Jet Situ`, `situjet` | 30,595 | 16% |
| **Others** | `jaswu51`, `joshp1225` | 4,851 | 2% |

## Adjusted for Integration Workflow

Some modules (vision_gps_estimator, rtsp_streamer, burst_recorder, vlm) were developed by Yufan Liu but committed by Jet Situ during hardware testing. Evidence: code headers state *"Refactored from original humanflow integrated_node.py"* — the original `humanflow/` is authored by Yufan Liu per git history.

| Contributor | Adjusted Share |
|-------------|----------------|
| **Yufan Liu** | ~90% |
| **Jet Situ** | ~7% |
| **Others** | ~3% |

---

**Verification:** Run `./scripts/contribution_analysis.sh` from repository root.

