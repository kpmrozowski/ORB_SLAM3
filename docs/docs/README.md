# Documentation

Practical guide to this ORB-SLAM3 fork — building it, preparing data, running it, scoring it, and what was learned.

## Contents

| # | Doc | For |
|---|---|---|
| 01 | [Repository Overview](01-repo-overview.md) | What this is, layout, branches |
| 02 | [Building the Code](02-building-code.md) | Dependencies + build steps |
| 03 | [Dataset Formats & Converting to EuRoC](03-dataset-formats-and-converting-to-euroc.md) | Input formats + the converters |
| 04 | [Running on Data](04-running-on-data.md) | Binary invocation + env recipes |
| 05 | [Evaluating Results](05-evaluating-results.md) | Metrics + scoring scripts |
| 06 | [The Algorithm & Modifications That Worked](06-vanilla-algo-all-good-mods.md) | Deep-dive: how it works + the wins |
| 07 | [Things That Did Not Improve](07-tested-not-improving-mods.md) | Deep-dive: honest failures / divergences |
| 08 | [Environment Variable Reference](08-environment-variable-reference.md) | Every `ORB_*` knob |
| 09 | [Raspberry Pi / ARM64 Deployment](09-raspberry-pi-arm-deployment.md) | Building & running on ARM |
| 10 | [Troubleshooting & Gotchas](10-troubleshooting-and-gotchas.md) | Quick fixes |

## See also

- [`../DEVELOPMENT_REPORT.md`](../DEVELOPMENT_REPORT.md) — the full two-week research report (precise metric definitions, quantitative tables).
- [`../diary/`](../diary/) — the day-by-day research log.
- [`../../tools/dataset_conversion/`](../../tools/dataset_conversion/) — the dataset converters.

## New here?

Read **01 → 02 → 03 → 04 → 05** in order to get a run scored end-to-end, then dip into **06/07** for the why.
