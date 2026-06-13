## 2026-05-27 camera focus autofocus readback

- Object: check whether the rover camera exposes the exact focus value after autofocus.
- Result: yes on the current UVC path. `cv2.CAP_PROP_FOCUS` and `v4l2-ctl` both report a numeric focus while autofocus is on.
- Current live reading on `Arducam_8mp` `/dev/video0`: `focus_automatic_continuous=1`, `focus_absolute=144`, OpenCV focus readback `144`.
- Earlier saved autofocus run in `data/live_visual_servo_runs/20260527_063815_autofocus_timing_probe_path/metadata.json` reported `focus_absolute=336`, so autofocus readback is not just a fixed dummy value.
- `/dev/video2` is not the node with focus controls; use `/dev/video0` for focus readback.
- Safe conclusion for calibration work: let autofocus settle at a given stand-off, read the reported numeric focus, then treat that value as the focus label for captures/calibration.
- Warning: this quick probe only proved focus readback. It did not prove that manual focus writes always take effect through the same OpenCV sequence. A test write to `500` reported success but the camera stayed at `144`.

## 2026-05-28 focus calibration set and model switching

- Proper solved focus values now available: `350`, `370`, `375`, `380`, `385`, `390`, `400`.
- Main dense calibration anchors are `370`, `380`, and `400`. `370` is the cleanest new set. `375`, `385`, and `390` are real solved models too, but they come from smaller image sets and should be treated as weaker than the dense anchors.
- `focus_0370`: 46 board observations used, residual RMS about `0.391 px`.
- `focus_0380`: 49 board observations used, residual RMS about `0.426 px`.
- `focus_0400`: 49 board observations used, residual RMS about `0.576 px`. Usable, but weaker than `370` and `380`.
- `focus_0375`, `0385`, `0390` solved from about `10..12` observations each. They are good enough to keep as exact focus-specific models, but not as strong as the dense anchors.
- The shared calibration loader in `tools/apriltag_pose.py` now auto-selects the exact solved model when manual `--focus-absolute` is set and no explicit `--model` is passed.
- Current exact focus-to-model mapping resolves correctly for `350`, `370`, `375`, `380`, `385`, `390`, and `400`.
- If an unsupported manual focus is requested, the tool now fails clearly and lists the available calibrated focus values instead of silently using the wrong model.
- If `--model` is passed explicitly, that still overrides the automatic focus lookup.
- If autofocus is enabled or no manual focus match is being requested, default model selection stays on the old canonical `focus_350` path instead of drifting to the newest calibration run.
