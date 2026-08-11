# Wygu — CPU-Pitch Robot Sandbox

A complete, self-contained sandbox that places an Optimus-style robot inside a predictive-echo control loop.

## Behavior

- The robot continuously samples real **CPU utilization**.
- CPU % is mapped to an acoustic **pitch** (Hz).
- Whenever the pitch reaches or exceeds the configured threshold, the robot:
  1. Generates a **pre-selected restart chime** (C5 two-tone WAV).
  2. Fully **restarts its internal Sandbox** (clears EchoHypothesis, resets pose, re-seeds controller, clears pushed paths).
  3. Logs the event with timestamp.
- Between triggers the robot runs the classic predict → decide → push → do → hear → correct cycle.

All sound files and logs are written locally under `sounds/` and `logs/`. No external audio hardware is required.

## Quick start

```bash
cd /home/workdir/artifacts/wygu
python3 wygu.py
```

## Key configuration (top of `wygu.py`)

| Constant              | Meaning                                      | Default |
|-----------------------|----------------------------------------------|---------|
| `PITCH_THRESHOLD_HZ`  | Pitch that forces internal sandbox restart   | 380 Hz  |
| `BASE_PITCH_HZ`       | Pitch at 0 % CPU                             | 220 Hz  |
| `CPU_TO_PITCH_SCALE`  | Hz added per CPU percent                     | 7.5     |
| `RESTART_CHIME_FREQ`  | Frequency of the pre-selected sound          | 523.25 Hz (C5) |
| `FORCE_SPIKE_EVERY`   | Controlled CPU spike cadence (demo only)     | 6       |

## Artifacts produced

- `sounds/restart_chime_rNNN_*.wav` – one file per restart
- `logs/run_*.log` – full step-by-step transcript

## Architecture notes

The internal `InternalSandbox` is a deep-copy of the robot’s pose + EchoHypothesis. It is the only place the robot “thinks” about future echoes before acting. A pitch-triggered restart wipes that private world completely, forcing the robot to re-acquire its acoustic hypothesis from scratch.
