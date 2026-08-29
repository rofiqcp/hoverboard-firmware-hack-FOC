#!/usr/bin/env python3
"""Interactive Arduino-monitor-like terminal for hoverboard USART3 control."""
from __future__ import annotations

import argparse
import shlex
import threading
import time
from pathlib import Path

try:
    from prompt_toolkit import PromptSession
    from prompt_toolkit.patch_stdout import patch_stdout
except ImportError as exc:
    raise SystemExit("Install: python3 -m pip install -r tools/requirements.txt") from exc

from hoverlink import (
    FB, MODE_NAMES, TELEMETRY_HZ, CsvLogger, HoverLink, Telemetry,
    available_ports, clamp_cmd,
)


def parse_lr(tokens: list[str]) -> tuple[int, int]:
    if len(tokens) == 1 and "," in tokens[0]:
        a, b = tokens[0].split(",", 1)
    elif len(tokens) == 2:
        a, b = tokens
    else:
        raise ValueError("expected: L,R  or  L R")
    return clamp_cmd(int(a)), clamp_cmd(int(b))


class TerminalApp:
    def __init__(self, port: str, baud: int, display_hz: float):
        self.link = HoverLink(port, baud)
        self.logger = CsvLogger(Path(__file__).resolve().parent / "logs")
        self.target_l = 0
        self.target_r = 0
        self.latest: Telemetry | None = None
        self.display_period = 1.0 / max(1.0, display_hz)
        self.last_display = 0.0
        self.stop_pending = False
        self.stop_zero_frames = 0
        self._lock = threading.Lock()
        self._run = True
        self.link.on_telemetry = self.on_telemetry
        self.link.on_debug = lambda line: print(f"[DBG] {line}")
        self.link.on_error = lambda err: print(f"[SERIAL ERROR] {err}")
        self.tx = threading.Thread(target=self.tx_loop, daemon=True)
        self.tx.start()

    def tx_loop(self) -> None:
        period = 1.0 / TELEMETRY_HZ
        deadline = time.monotonic()
        while self._run:
            with self._lock: l, r = self.target_l, self.target_r
            try: self.link.send_command(l, r)
            except Exception as exc:
                print(f"[TX ERROR] {exc}"); return
            deadline += period
            time.sleep(max(0.0, deadline - time.monotonic()))

    def on_telemetry(self, t: Telemetry) -> None:
        self.latest = t
        if self.logger.active:
            self.logger.write(t)  # every valid firmware frame = 50 Hz
        now = time.monotonic()
        if now - self.last_display >= self.display_period:
            print(t.summary())
            self.last_display = now

        if self.stop_pending:
            if t.stopped:
                self.stop_zero_frames += 1
            else:
                self.stop_zero_frames = 0
            if self.stop_zero_frames >= 3:
                self.stop_pending = False
                path, rows, missed = self.logger.stop()
                print("[STOP] cmdL=0 cmdR=0 confirmed after rate limiter")
                if path:
                    print(f"[CSV] saved {rows} rows @50Hz -> {path} (missed_seq={missed})")

    def set_target(self, l: int, r: int) -> None:
        with self._lock:
            self.target_l, self.target_r = clamp_cmd(l), clamp_cmd(r)

    def fully_stopped(self) -> bool:
        with self._lock: target_zero = self.target_l == 0 and self.target_r == 0
        return bool(target_zero and self.latest and self.latest.stopped and not self.stop_pending)

    def cmd_mode(self, mode: int) -> None:
        if mode not in MODE_NAMES: raise ValueError("mode must be 1..6")
        if not self.fully_stopped():
            print("[MODE] rejected: STOP and wait until cmd=0,0 first"); return
        self.link.send_debug(f"SET CTRL_MOD {mode}")
        print(f"[MODE] requested {mode}={MODE_NAMES[mode]}")

    def cmd_start(self, l: int, r: int) -> None:
        if not self.latest:
            print("[START] no telemetry yet"); return
        if self.latest.calibrating:
            print("[START] rejected: ADC calibration still active"); return
        if not self.fully_stopped():
            print("[START] rejected: controller is not at full STOP"); return
        path = self.logger.start(self.latest.mode)
        self.set_target(l, r)
        print(f"[CSV] recording every telemetry frame (50 Hz) -> {path}")
        print(f"[START] cmdL={l} cmdR={r}")

    def cmd_drive(self, l: int, r: int) -> None:
        if self.stop_pending:
            print("[DRIVE] wait for STOP ramp-down to finish"); return
        self.set_target(l, r)
        print(f"[DRIVE] target cmdL={l} cmdR={r}")

    def cmd_stop(self) -> None:
        self.set_target(0, 0)
        self.stop_pending = True
        self.stop_zero_frames = 0
        print("[STOP] target=0,0; firmware rate limiter remains active ...")

    def cmd_calibrate(self) -> None:
        if not self.fully_stopped():
            print("[CAL] rejected: STOP and wait until cmd=0,0 first"); return
        if self.logger.active:
            print("[CAL] rejected: finish/save the current CSV run first"); return
        self.link.send_debug("CALIBRATE")
        print("[CAL] requested: six current ADC offsets, 2000 samples (~125 ms @16 kHz)")

    def help(self) -> None:
        print("""
Commands
  mode N             change mode only at full STOP
                     1=FOC VLT  2=FOC SPD  3=FOC TRQ
                     4=SVPWM sensorless  5=6-step commutation  6=Sine PWM
  start L,R          start motor target + begin 50-Hz CSV recording
  drive L,R          change independent Left/Right target while running
  stop               target 0,0 through rate limiter; auto-save CSV at true stop
  calibrate          manual current-ADC calibration (STOP only)
  status             show latest fixed-width telemetry
  GET/SET/WATCH ...  raw firmware debug command over the same USART3
  quit               smooth stop, save CSV, close port
Examples
  mode 1
  start 50,50
  drive 100,80
  stop
""".strip())

    def run(self) -> None:
        print(f"Connected {self.link.port} @ {self.link.baud}; feedback={FB.size} bytes; telemetry={TELEMETRY_HZ} Hz")
        print("Left/Right independent control over USART3")
        print("modes: " + " | ".join(f"{k}={v}" for k, v in MODE_NAMES.items()))
        print("type 'help' for commands")
        session = PromptSession("hover> ")
        with patch_stdout(raw=True):
            while self._run:
                try:
                    line = session.prompt().strip()
                except (EOFError, KeyboardInterrupt):
                    line = "quit"
                if not line: continue
                try:
                    tok = shlex.split(line)
                    cmd = tok[0].lower()
                    args = tok[1:]
                    if cmd == "help": self.help()
                    elif cmd == "mode": self.cmd_mode(int(args[0]))
                    elif cmd == "start": self.cmd_start(*parse_lr(args))
                    elif cmd == "drive": self.cmd_drive(*parse_lr(args))
                    elif cmd == "stop": self.cmd_stop()
                    elif cmd in ("cal", "calib", "calibrate"): self.cmd_calibrate()
                    elif cmd == "status": print(self.latest.summary() if self.latest else "no telemetry")
                    elif cmd in ("quit", "exit"):
                        self.cmd_stop()
                        deadline = time.monotonic() + 8.0
                        while self.stop_pending and time.monotonic() < deadline: time.sleep(0.02)
                        break
                    elif cmd.upper() in ("GET", "SET", "WATCH", "SAVE", "INIT", "HELP"):
                        self.link.send_debug(line)
                    else:
                        print("unknown command; type 'help'")
                except (ValueError, IndexError) as exc:
                    print(f"[INPUT] {exc}")
        self.close()

    def close(self) -> None:
        self._run = False
        self.set_target(0, 0)
        if self.logger.active:
            path, rows, missed = self.logger.stop()
            print(f"[CSV] closed {rows} rows -> {path} (missed_seq={missed})")
        try: self.link.send_command(0, 0)
        except Exception: pass
        self.link.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="USART3 USB-UART port; auto-selects first port if omitted")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--display-hz", type=float, default=10.0, help="terminal redraw/print rate; CSV remains 50 Hz")
    ns = ap.parse_args()
    port = ns.port
    if not port:
        ports = available_ports()
        if not ports: raise SystemExit("No serial port found. Use --port /dev/ttyUSB0")
        port = "/dev/ttyUSB0" if "/dev/ttyUSB0" in ports else ports[0]
    TerminalApp(port, ns.baud, ns.display_hz).run()


if __name__ == "__main__":
    main()
