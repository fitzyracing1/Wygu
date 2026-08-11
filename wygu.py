#!/usr/bin/env python3
"""
Wygu — CPU-Pitch Robot Sandbox

A self-contained simulation sandbox containing an Optimus-style robot.
The robot maintains an *internal* predictive Sandbox (echo hypothesis).

Every control cycle:
  1. Sample real CPU utilization via psutil.
  2. Map CPU % → acoustic pitch (Hz).
  3. If the computed pitch reaches or exceeds a configurable threshold,
     the robot:
       - generates a pre-selected restart chime (WAV),
       - logs the event,
       - fully restarts its internal Sandbox (clears hypothesis, resets pose,
         resets counters, re-seeds the controller).
  4. Otherwise continues the normal predict-echo decide-do hear-correct loop.

No external audio device is required; sounds are written as WAV files.
All work stays inside the sandbox.
"""

from __future__ import annotations

import copy
import datetime
import os
import time
from pathlib import Path

import numpy as np
import psutil
from scipy.io import wavfile

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SAMPLE_RATE = 44100
PITCH_THRESHOLD_HZ = 380.0          # trigger restart when mapped pitch >= this
BASE_PITCH_HZ = 220.0               # A3
CPU_TO_PITCH_SCALE = 7.5            # Hz per percent CPU
RESTART_CHIME_FREQ = 523.25         # C5 – the pre-selected sound
RESTART_CHIME_DURATION = 0.55       # seconds
MAX_STEPS = 24
SLEEP_BETWEEN_STEPS = 0.22          # keep CPU load visible
FORCE_SPIKE_EVERY = 6               # inject controlled CPU spike every N steps for demo

ARTIFACT_ROOT = Path(__file__).resolve().parent
SOUNDS_DIR = ARTIFACT_ROOT / "sounds"
LOGS_DIR = ARTIFACT_ROOT / "logs"
SOUNDS_DIR.mkdir(parents=True, exist_ok=True)
LOGS_DIR.mkdir(parents=True, exist_ok=True)

SPEED_OF_SOUND = 343.0


# ---------------------------------------------------------------------------
# Sound generation (pre-selected restart chime)
# ---------------------------------------------------------------------------
def generate_restart_chime(path: Path, freq: float = RESTART_CHIME_FREQ) -> Path:
    """Synthesize a short, pleasant two-tone restart chime and write WAV."""
    t = np.linspace(0, RESTART_CHIME_DURATION, int(SAMPLE_RATE * RESTART_CHIME_DURATION), endpoint=False)
    # primary tone + soft fifth for character
    wave = 0.55 * np.sin(2 * np.pi * freq * t)
    wave += 0.25 * np.sin(2 * np.pi * (freq * 1.5) * t)
    # exponential decay envelope
    envelope = np.exp(-3.2 * t)
    wave *= envelope
    # soft fade-in
    fade = min(int(0.04 * SAMPLE_RATE), len(wave) // 4)
    wave[:fade] *= np.linspace(0, 1, fade)
    audio = (wave * 32767).astype(np.int16)
    wavfile.write(str(path), SAMPLE_RATE, audio)
    return path


# ---------------------------------------------------------------------------
# Minimal Axb-style components (kept self-contained)
# ---------------------------------------------------------------------------
class GroundStation:
    def __init__(self):
        self.vars = {
            "goal": np.array([4.0, 0.0]),
            "ground_ok": True,
            "obstacles": [np.array([2.0, 0.35])],
        }
        self.pushed_paths = []

    def pull(self):
        return {k: v for k, v in self.vars.items() if k != "obstacles"}

    def push(self, path):
        self.pushed_paths.append(path)


def ground_scan(station_vars, pose):
    heading = station_vars["goal"] - pose
    dist = float(np.linalg.norm(heading))
    return {
        "clear_ahead": station_vars["ground_ok"],
        "goal_dir": heading / max(dist, 1e-9),
        "goal_dist": dist,
    }


def echo_from(pose, scatterers):
    echoes = []
    for s in scatterers:
        d = float(np.linalg.norm(np.asarray(s) - pose))
        echoes.append({"tof_s": 2 * d / SPEED_OF_SOUND, "range_m": d})
    return sorted(echoes, key=lambda e: e["range_m"])


class Ears:
    def listen(self, pose, true_obstacles):
        return echo_from(pose, true_obstacles)


class EchoHypothesis:
    def __init__(self):
        self.scatterers = []
        self.heading = np.array([1.0, 0.0])

    def predict(self, pose):
        if not self.scatterers:
            return []
        return echo_from(pose, self.scatterers)

    def correct(self, pose, real_echoes):
        self.scatterers = [
            pose + self.heading * e["range_m"] for e in real_echoes
        ]

    def surprise(self, pose, real_echoes):
        pred = self.predict(pose)
        if not pred and not real_echoes:
            return 0.0
        if not pred or not real_echoes:
            return 1.0
        return abs(pred[0]["range_m"] - real_echoes[0]["range_m"])


class InternalSandbox:
    """The robot's private predictive sandbox (deep-copied hypothesis + state)."""

    def __init__(self, robot_state, hypothesis):
        self.state = copy.deepcopy(robot_state)
        self.hyp = copy.deepcopy(hypothesis)

    def simulate(self, action):
        pose = self.state["pose"] + action
        predicted = self.hyp.predict(pose)
        nearest = predicted[0]["range_m"] if predicted else np.inf
        d = 1.0 + max(0.0, 1.0 - nearest / 2.0)
        return {"pose": pose, "d": d, "predicted": predicted}


class AxbController:
    def __init__(self, c=1.0, n_guesses=6, seed=0):
        self.c = c
        self.n_guesses = n_guesses
        self.rng = np.random.default_rng(seed)

    def solve(self, A, b, d):
        return (np.linalg.pinv(A) @ b) * (1.0 / self.c) / d

    def guess_check_decide(self, robot_state, hyp, scan):
        A = np.eye(2)
        b = scan["goal_dir"] * min(scan["goal_dist"], 0.45)
        best = None
        for _ in range(self.n_guesses):
            trial_b = b + self.rng.normal(0.0, 0.04, size=2)
            probe = InternalSandbox(robot_state, hyp).simulate(trial_b)
            x = self.solve(A, trial_b, probe["d"])
            new_dist = np.linalg.norm(
                (robot_state["pose"] + x)
                - (robot_state["pose"] + scan["goal_dir"] * scan["goal_dist"])
            )
            score = -new_dist - (probe["d"] - 1.0)
            if best is None or score > best["score"]:
                best = {"x": x, "score": score, "d": probe["d"]}
        return best


# ---------------------------------------------------------------------------
# The Robot that owns the internal sandbox and reacts to CPU pitch
# ---------------------------------------------------------------------------
class WyguRobot:
    def __init__(self, name: str = "Wygu"):
        self.name = name
        self.station = GroundStation()
        self.ears = Ears()
        self.hyp = EchoHypothesis()
        self.ctrl = AxbController(seed=42)
        self.pose = np.array([0.0, 0.0], dtype=float)
        self.step_count = 0
        self.restart_count = 0
        self.last_pitch = BASE_PITCH_HZ
        self.log_lines: list[str] = []

    def cpu_to_pitch(self, force_spike: bool = False) -> float:
        """Map current CPU utilization to an acoustic pitch in Hz.
        Optionally burn a short CPU burst so the threshold can be demonstrated."""
        if force_spike:
            # short deterministic busy loop to raise instantaneous load
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 0.12:
                _ = sum(i * i for i in range(18000))
        cpu = psutil.cpu_percent(interval=0.04)
        pitch = BASE_PITCH_HZ + cpu * CPU_TO_PITCH_SCALE
        self.last_pitch = pitch
        return pitch

    def restart_internal_sandbox(self, pitch: float) -> Path:
        """Full internal restart + emit the pre-selected sound."""
        self.restart_count += 1
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        sound_path = SOUNDS_DIR / f"restart_chime_r{self.restart_count:03d}_{ts}.wav"
        generate_restart_chime(sound_path)

        # wipe internal predictive world
        self.hyp = EchoHypothesis()
        self.pose = np.array([0.0, 0.0], dtype=float)
        self.ctrl = AxbController(seed=42 + self.restart_count)
        self.station.pushed_paths.clear()
        self.step_count = 0

        msg = (
            f"[{ts}] RESTART #{self.restart_count}  "
            f"pitch={pitch:.1f} Hz ≥ {PITCH_THRESHOLD_HZ} Hz  "
            f"→ internal sandbox wiped  sound={sound_path.name}"
        )
        print(msg)
        self.log_lines.append(msg)
        return sound_path

    def step(self) -> dict:
        """One full control cycle. Returns status dict."""
        self.step_count += 1
        # force a CPU spike on a regular cadence so the pitch trigger is visible
        force = (self.step_count % FORCE_SPIKE_EVERY == 0)
        pitch = self.cpu_to_pitch(force_spike=force)

        # ---------- pitch trigger ----------
        if pitch >= PITCH_THRESHOLD_HZ:
            sound = self.restart_internal_sandbox(pitch)
            return {
                "status": "restarted",
                "pitch": pitch,
                "restart_count": self.restart_count,
                "sound": str(sound),
                "pose": self.pose.tolist(),
            }

        # ---------- normal predictive-echo cycle ----------
        env = self.station.pull()
        scan = ground_scan(env, self.pose)
        self.hyp.heading = scan["goal_dir"]

        if not scan["clear_ahead"]:
            status = "blocked"
            decision = None
        else:
            decision = self.ctrl.guess_check_decide(
                {"pose": self.pose}, self.hyp, scan
            )
            self.station.push(decision["x"].tolist())
            self.pose = self.pose + decision["x"]

            real = self.ears.listen(self.pose, self.station.vars["obstacles"])
            surprise = self.hyp.surprise(self.pose, real)
            self.hyp.correct(self.pose, real)
            status = "ok"

        info = {
            "status": status,
            "step": self.step_count,
            "pitch": pitch,
            "pose": np.round(self.pose, 3).tolist(),
            "goal_dist": round(scan["goal_dist"], 3),
            "restart_count": self.restart_count,
        }
        if decision:
            info["d"] = round(decision["d"], 3)
            info["surprise"] = round(surprise, 3)

        line = (
            f"[s{self.step_count:02d}] pitch={pitch:6.1f} Hz  "
            f"pose={info['pose']}  goal={info['goal_dist']:.2f}  "
            f"restarts={self.restart_count}"
        )
        print(line)
        self.log_lines.append(line)
        return info

    def run(self, max_steps: int = MAX_STEPS):
        print(f"=== {self.name} CPU-Pitch Sandbox started ===")
        print(f"Pitch threshold : {PITCH_THRESHOLD_HZ} Hz")
        print(f"Sounds dir      : {SOUNDS_DIR}")
        print("-" * 60)

        for _ in range(max_steps):
            status = self.step()
            if status.get("status") == "restarted":
                # brief pause after restart so the new cycle is visible
                time.sleep(0.15)
            else:
                time.sleep(SLEEP_BETWEEN_STEPS)

            # stop early if goal reached after a clean cycle
            if status.get("goal_dist", 99) < 0.18 and status.get("status") == "ok":
                print("→ Goal reached – clean exit")
                break

        # persist log
        log_path = LOGS_DIR / f"run_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        log_path.write_text("\n".join(self.log_lines) + "\n")
        print("-" * 60)
        print(f"Finished. Restarts: {self.restart_count}")
        print(f"Log written → {log_path}")
        return self.restart_count


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    robot = WyguRobot("Wygu")
    robot.run(max_steps=MAX_STEPS)
