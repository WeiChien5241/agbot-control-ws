# Vision-nav pipeline performance

Measured pipeline timing per machine, for comparing configurations. Numbers
come from the `timing:` instrumentation the node logs every 5 s
(`src/agbot_vision_nav/timing_stats.py`) and from the offline
`scripts/benchmark_inference.py`. Add a row whenever you measure a new
machine or a new setting.

| Machine | Device | Inference | Control rate (proc) | e2e latency | Notes |
|---|---|---|---|---|---|
| GPU robot **cpr-j100-0864** | cuda:0 | 16 ms (p95 16) | ~24 Hz | ~48–63 ms (p95 ~80) | 2026-07-22, lab. Front Brio `/brio_front/image_raw/compressed`, 25 Hz camera. Camera is now the bottleneck, not the model. |
| CPU robot cpr-j100-0463 | cpu | — | ~2 Hz | — | No GPU; runs with `mpc_dt:=0.5`. Live row-following achieved. |
| Sim laptop (WSL2) | cpu | 165 ms | ~5.7 Hz | ~288 ms | RTF-limited sim. |

## How to read the timing line

```
timing: cam 25.0 Hz | proc 24.3 Hz | inf 16 ms (p95 16) | e2e 57 ms (p95 76) | dropped 15% (by design)
```

- **`cam`** — raw camera publish rate.
- **`proc`** — frames actually processed = your real control rate (cmd_vel rate).
- **`inf`** — model inference time, mean (p95).
- **`e2e`** — camera-stamp → cmd_vel latency (≈ one inference + a few tens of ms transport/decode).
- **`dropped`** — frames skipped by the latest-frame-wins buffer. **High % is BY DESIGN** — it just means frames arrive faster than they're processed. Low % means the machine is keeping up.

## Gotchas when reading numbers

- **Ignore the FIRST `timing:` line** after node start. On a GPU it shows a huge inference/e2e (e.g. 683 ms) — that is one-time CUDA warmup (context creation + kernel compilation), not representative. Every line after it is steady state.
- **Check the startup device line**: `Model loaded on device: cuda:0`. On a GPU machine, `cpu` there means torch has no usable CUDA — that is the #1 cause of slow inference. Confirm with `nvidia-smi` showing the python process while the node runs.
- For a **camera-free, warmup-excluded** number (mean/p50/p95), run the benchmark inside the model venv:
  ```bash
  source ~/agbot_venv/bin/activate
  cd ~/agbot_control_ws/src/agbot_vision_nav
  PYTHONPATH=src python3 scripts/benchmark_inference.py \
    --model config/exported_best.pt --image /path/to/a/640x480/frame.jpg
  ```
