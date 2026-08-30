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
    FB, SENSOR_NAMES, COMM_NAMES, CONTROL_NAMES, TELEMETRY_HZ,
    CsvLogger, HoverLink, Telemetry, available_ports, clamp_cmd,
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
                print("[STOP] cmdL=0 cmdR=0 confirmed")
                if path:
                    print(f"[CSV] saved {rows} rows @50Hz -> {path} (missed_seq={missed})")

    def set_target(self, l: int, r: int) -> None:
        with self._lock:
            self.target_l, self.target_r = clamp_cmd(l), clamp_cmd(r)

    def fully_stopped(self) -> bool:
        with self._lock: target_zero = self.target_l == 0 and self.target_r == 0
        return bool(target_zero and self.latest and self.latest.stopped and abs(self.latest.rpm_l) <= 5 and abs(self.latest.rpm_r) <= 5 and not self.stop_pending)

    def cmd_mode(self, kind: str, side: str | None, value: int) -> None:
        kind = kind.lower()
        if kind == "controll":
            kind = "control"
        if kind not in ("sensor", "comm", "control"):
            raise ValueError("mode type must be sensor|comm|control")
        side_l = side.lower() if side else None
        if side_l not in (None, "left", "right", "l", "r"):
            raise ValueError("side must be left or right")
        limits = {"sensor": (1, 3), "comm": (1, 3), "control": (1, 4)}
        lo, hi = limits[kind]
        if not lo <= value <= hi:
            raise ValueError(f"{kind} must be {lo}..{hi}")
        if kind == "sensor" and value == 3 and side_l in (None, "right", "r"):
            raise ValueError("encoder AB (sensor 3) is Left-only")
        if kind == "control" and value == 4 and side_l not in ("left", "l"):
            raise ValueError("position control (control 4) requires explicit Left + encoder AB")
        if not self.fully_stopped():
            print("[MODE] rejected: STOP and wait until cmd=0,0 and |rpm|<=5 first"); return
        cmd = f"mode {kind}"
        if side_l: cmd += " " + ("left" if side_l in ("left","l") else "right")
        cmd += f" {value}"
        self.link.send_debug(cmd)
        print(f"[MODE] requested: {cmd}")

    def cmd_live(self, enabled: bool) -> None:
        self.link.send_debug("live on" if enabled else "live off")
        print(f"[LIVE] {'ON' if enabled else 'OFF'}")

    def cmd_start(self, l: int, r: int) -> None:
        if not self.latest:
            print("[START] no telemetry yet"); return
        if self.latest.calibrating:
            print("[START] rejected: ADC calibration still active"); return
        if not self.fully_stopped():
            print("[START] rejected: controller is not at full STOP"); return
        self.link.send_debug("live on")
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
        # Send zero immediately as well as through the periodic keepalive loop.
        self.link.send_command(0, 0)
        print("[STOP] immediate target=0,0; firmware zero-vector STOP requested")

    def cmd_calibrate(self) -> None:
        if not self.fully_stopped():
            print("[CAL] rejected: STOP and wait until cmd=0,0 first"); return
        if self.logger.active:
            print("[CAL] rejected: finish/save the current CSV run first"); return
        self.link.send_debug("CALIBRATE")
        print("[CAL] requested: six current ADC offsets: bridge OFF + proven 2000-sample recursive convergence (~125 ms) + controller state reset")

    def help(self) -> None:
        print("""
Commands
  mode sensor [left|right] N   N=1 openloop, 2 Hall, 3 encoder AB (Left only)
  mode comm [left|right] N     N=1 six-step, 2 sine PWM, 3 SVPWM
  mode control [left|right] N  N=1 PWM, 2 current, 3 speed, 4 position (Left encoder)
                               omit side => apply both (except sensor 3)
  live on | live off           enable/disable binary telemetry stream
  start L,R                    start target + LIVE ON + begin 50-Hz CSV
  drive L,R                    change Left/Right target while running
  stop                         immediate authoritative 0,0 + save CSV after confirmed stop
  calibrate                    current-ADC calibration (STOP only)
  status                       latest telemetry
  GET/SET/WATCH/SAVE ...       raw debug command
  quit                         stop, save CSV, close
Examples
  mode sensor 2
  mode comm 3
  mode control 3
  start 100,100
  stop
  mode sensor left 3
""".strip())

    def run(self) -> None:
        print(f"Connected {self.link.port} @ {self.link.baud}; feedback={FB.size} bytes; telemetry={TELEMETRY_HZ} Hz")
        print("Left/Right independent control over USART3")
        print("runtime modes: sensor 1/2/(3 Left), comm 1/2/3, control 1/2/3/(4 Left encoder)")
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
                    elif cmd == "mode":
                        if len(args) == 2: self.cmd_mode(args[0], None, int(args[1]))
                        elif len(args) == 3: self.cmd_mode(args[0], args[1], int(args[2]))
                        else: raise ValueError("mode <sensor|comm|control> [left|right] N")
                    elif cmd == "live":
                        if len(args) != 1 or args[0].lower() not in ("on","off"): raise ValueError("live on|off")
                        self.cmd_live(args[0].lower() == "on")
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
