# Wygu: CPU-pitch robot sandbox

`wygu.py` is a self-contained simulation. An Optimus-style robot samples real CPU utilization, maps it to a pitch in Hz, and runs a predict → decide → push → do → hear → correct loop. When the pitch reaches the threshold, it writes a C5 restart-chime WAV, resets its internal sandbox (EchoHypothesis, pose, controller), and logs the event.

## Getting started

Requires Python 3 with `numpy`, `psutil` and `scipy` (imported in `wygu.py`). The repo has no requirements file.

```bash
pip install numpy psutil scipy
python3 wygu.py
```

The README's `cd /home/workdir/artifacts/wygu` refers to the original sandbox path. Run the script from your clone instead.

## Configuration (constants at the top of `wygu.py`)

| Constant | Default |
|---|---|
| `PITCH_THRESHOLD_HZ` | 380 |
| `BASE_PITCH_HZ` | 220 |
| `CPU_TO_PITCH_SCALE` | 7.5 Hz per CPU % |
| `RESTART_CHIME_FREQ` | 523.25 (C5) |
| `FORCE_SPIKE_EVERY` | 6 (demo spike cadence) |

Outputs go to `sounds/restart_chime_rNNN_*.wav` and `logs/run_*.log`.

The repo has no tests, no CI and no LICENSE file. Copyright (c) 2026 Joshua Almeida
