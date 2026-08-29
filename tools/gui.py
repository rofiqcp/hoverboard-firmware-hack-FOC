#!/usr/bin/env python3
"""Complete PyQt5 GUI for USART3 control, 50-Hz telemetry and CSV logging."""
from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg
except ImportError as exc:
    raise SystemExit("Install GUI requirements: python3 -m pip install -r tools/requirements.txt") from exc

from hoverlink import (
    CMD_MAX, CMD_MIN, CPU_HZ, FOC_BUDGET_CYCLES, MODE_NAMES, TELEMETRY_HZ,
    CsvLogger, HoverLink, Telemetry, available_ports, clamp_cmd,
)


class SignalBridge(QtCore.QObject):
    telemetry = QtCore.pyqtSignal(object)
    debug = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)


class ValueLabel(QtWidgets.QLabel):
    def __init__(self, text: str = "--", parent=None):
        super().__init__(text, parent)
        f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        f.setPointSize(max(9, f.pointSize()))
        self.setFont(f)
        self.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)


class MotorPanel(QtWidgets.QGroupBox):
    def __init__(self, title: str):
        super().__init__(title)
        form = QtWidgets.QFormLayout(self)
        self.cmd = ValueLabel(); self.rpm = ValueLabel(); self.iq = ValueLabel()
        self.id = ValueLabel(); self.idc = ValueLabel(); self.hall = ValueLabel()
        form.addRow("Command", self.cmd)
        form.addRow("RPM", self.rpm)
        form.addRow("Iq / torque-axis", self.iq)
        form.addRow("Id / flux-axis", self.id)
        form.addRow("DC current", self.idc)
        form.addRow("Hall", self.hall)

    def update_motor(self, t: Telemetry, right: bool = False) -> None:
        if right:
            vals = (t.cmd_r, t.rpm_r, t.iq_r_a, t.id_r_a, t.idc_r_a, t.hall_r)
        else:
            vals = (t.cmd_l, t.rpm_l, t.iq_l_a, t.id_l_a, t.idc_l_a, t.hall_l)
        self.cmd.setText(f"{vals[0]:+d}")
        self.rpm.setText(f"{vals[1]:+d}")
        self.iq.setText(f"{vals[2]:+.2f} A")
        self.id.setText(f"{vals[3]:+.2f} A")
        self.idc.setText(f"{vals[4]:+.2f} A")
        self.hall.setText(f"{vals[5] & 7:03b}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hoverboard FOC — USART3 6-Mode Control & Telemetry")
        self.link: HoverLink | None = None
        self.bridge = SignalBridge()
        self.bridge.telemetry.connect(self.on_telemetry)
        self.bridge.debug.connect(self.append_console)
        self.bridge.error.connect(self.on_serial_error)

        self.logger = CsvLogger(Path(__file__).resolve().parent / "logs")
        self.target_l = 0; self.target_r = 0
        self.latest: Telemetry | None = None
        self.stop_pending = False; self.stop_zero_frames = 0
        self.last_arrivals = deque(maxlen=100)

        max_points = TELEMETRY_HZ * 20
        self.plot_t0 = time.monotonic()
        self.plot_time = deque(maxlen=max_points)
        self.plot_rpm_l = deque(maxlen=max_points); self.plot_rpm_r = deque(maxlen=max_points)
        self.plot_iq_l = deque(maxlen=max_points); self.plot_iq_r = deque(maxlen=max_points)
        self.plot_id_l = deque(maxlen=max_points); self.plot_id_r = deque(maxlen=max_points)
        self.plot_idc_l = deque(maxlen=max_points); self.plot_idc_r = deque(maxlen=max_points)
        self.plot_cycle = deque(maxlen=max_points)

        self.build_ui()
        self.refresh_ports()

        self.tx_timer = QtCore.QTimer(self)
        self.tx_timer.setInterval(round(1000 / TELEMETRY_HZ))
        self.tx_timer.timeout.connect(self.transmit_target)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plots)
        self.plot_timer.start(100)

        self.fit_to_screen()
        self.update_controls()

    # ---------- UI ----------
    def build_ui(self) -> None:
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8); root.setSpacing(6)

        top = QtWidgets.QGroupBox("Connection / Motor Control")
        g = QtWidgets.QGridLayout(top)
        self.port_combo = QtWidgets.QComboBox(); self.port_combo.setMinimumContentsLength(14)
        self.refresh_btn = QtWidgets.QPushButton("Refresh"); self.refresh_btn.clicked.connect(self.refresh_ports)
        self.baud_combo = QtWidgets.QComboBox(); self.baud_combo.addItems(["115200", "230400", "460800"]); self.baud_combo.setCurrentText("115200")
        self.connect_btn = QtWidgets.QPushButton("Connect"); self.connect_btn.clicked.connect(self.toggle_connection)
        self.mode_combo = QtWidgets.QComboBox()
        for mode, name in MODE_NAMES.items(): self.mode_combo.addItem(f"{mode} — {name}", mode)
        self.mode_btn = QtWidgets.QPushButton("Apply Mode"); self.mode_btn.clicked.connect(self.apply_mode)
        self.cmd_l = QtWidgets.QSpinBox(); self.cmd_r = QtWidgets.QSpinBox()
        for spin in (self.cmd_l, self.cmd_r):
            spin.setRange(CMD_MIN, CMD_MAX); spin.setSingleStep(10); spin.setKeyboardTracking(False)
        self.start_btn = QtWidgets.QPushButton("START + CSV"); self.start_btn.clicked.connect(self.start_run)
        self.drive_btn = QtWidgets.QPushButton("DRIVE"); self.drive_btn.clicked.connect(self.drive)
        self.stop_btn = QtWidgets.QPushButton("STOP + SAVE CSV"); self.stop_btn.clicked.connect(self.stop_run)
        self.cal_btn = QtWidgets.QPushButton("CALIBRATE ADC"); self.cal_btn.clicked.connect(self.calibrate)

        g.addWidget(QtWidgets.QLabel("Port"), 0, 0); g.addWidget(self.port_combo, 0, 1)
        g.addWidget(self.refresh_btn, 0, 2); g.addWidget(QtWidgets.QLabel("Baud"), 0, 3)
        g.addWidget(self.baud_combo, 0, 4); g.addWidget(self.connect_btn, 0, 5)
        g.addWidget(QtWidgets.QLabel("Mode"), 1, 0); g.addWidget(self.mode_combo, 1, 1, 1, 2); g.addWidget(self.mode_btn, 1, 3)
        g.addWidget(QtWidgets.QLabel("cmdL"), 2, 0); g.addWidget(self.cmd_l, 2, 1)
        g.addWidget(QtWidgets.QLabel("cmdR"), 2, 2); g.addWidget(self.cmd_r, 2, 3)
        g.addWidget(self.start_btn, 2, 4); g.addWidget(self.drive_btn, 2, 5)
        g.addWidget(self.stop_btn, 3, 4); g.addWidget(self.cal_btn, 3, 5)
        g.setColumnStretch(1, 1); g.setColumnStretch(2, 1)
        root.addWidget(top)

        self.tabs = QtWidgets.QTabWidget(); root.addWidget(self.tabs, 1)
        self.tabs.addTab(self.build_dashboard_tab(), "Dashboard")
        self.tabs.addTab(self.build_plot_tab(), "Graphs")
        self.tabs.addTab(self.build_console_tab(), "Raw / Console")
        self.statusBar().showMessage("Disconnected")

    def build_dashboard_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        motors = QtWidgets.QHBoxLayout()
        self.left_panel = MotorPanel("Motor Left")
        self.right_panel = MotorPanel("Motor Right — host direction normalized")
        motors.addWidget(self.left_panel); motors.addWidget(self.right_panel); layout.addLayout(motors)

        row = QtWidgets.QHBoxLayout()
        system = QtWidgets.QGroupBox("System / Telemetry"); sf = QtWidgets.QFormLayout(system)
        self.mode_value = ValueLabel(); self.dq_ref_value = ValueLabel(); self.voltage_value = ValueLabel()
        self.temp_value = ValueLabel(); self.cycle_value = ValueLabel(); self.status_value = ValueLabel()
        self.rate_value = ValueLabel(); self.seq_value = ValueLabel(); self.csv_value = ValueLabel("idle")
        sf.addRow("Mode", self.mode_value); sf.addRow("dq reference", self.dq_ref_value)
        sf.addRow("Battery", self.voltage_value); sf.addRow("Temperature", self.temp_value)
        sf.addRow("ISR cycles", self.cycle_value); sf.addRow("Status", self.status_value)
        sf.addRow("Telemetry RX", self.rate_value); sf.addRow("Sequence", self.seq_value); sf.addRow("CSV", self.csv_value)

        raw = QtWidgets.QGroupBox("Raw ADC"); rf = QtWidgets.QFormLayout(raw)
        self.raw_labels = {n: ValueLabel() for n in ("dcl", "rla", "rlb", "dcr", "rrb", "rrc")}
        for n, lab in self.raw_labels.items(): rf.addRow(n, lab)

        calibration = QtWidgets.QGroupBox("Current ADC Calibration"); cf = QtWidgets.QVBoxLayout(calibration)
        self.cal_progress = QtWidgets.QProgressBar(); self.cal_progress.setRange(0, 1000); self.cal_progress.setFormat("%p%")
        self.cal_info = QtWidgets.QLabel(
            "Automatic every boot: 2000 samples (~125 ms at 16 kHz). "
            "Manual CALIBRATE is allowed only after cmdL/cmdR have ramped fully to zero."
        ); self.cal_info.setWordWrap(True)
        cf.addWidget(self.cal_progress); cf.addWidget(self.cal_info); cf.addStretch(1)

        row.addWidget(system, 2); row.addWidget(raw, 1); row.addWidget(calibration, 2)
        layout.addLayout(row); layout.addStretch(1)
        return page

    def _new_plot(self, title: str, ylabel: str = ""):
        p = pg.PlotWidget(title=title); p.addLegend(); p.showGrid(x=True, y=True, alpha=0.25)
        p.setLabel("bottom", "Time", units="s")
        if ylabel: p.setLabel("left", ylabel)
        return p

    def build_plot_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        tabs = QtWidgets.QTabWidget(); layout.addWidget(tabs)
        pens = [pg.mkPen(pg.intColor(i, hues=8), width=2) for i in range(8)]

        self.rpm_plot = self._new_plot("RPM Left / Right", "RPM")
        self.rpm_l_curve = self.rpm_plot.plot(name="RPM L", pen=pens[0]); self.rpm_r_curve = self.rpm_plot.plot(name="RPM R", pen=pens[1])
        tabs.addTab(self.rpm_plot, "RPM")

        self.dq_plot = self._new_plot("Iq / Id comparison across modes", "Ampere")
        self.iq_l_curve = self.dq_plot.plot(name="Iq L", pen=pens[0]); self.iq_r_curve = self.dq_plot.plot(name="Iq R", pen=pens[1])
        self.id_l_curve = self.dq_plot.plot(name="Id L", pen=pens[2]); self.id_r_curve = self.dq_plot.plot(name="Id R", pen=pens[3])
        tabs.addTab(self.dq_plot, "Iq / Id")

        self.idc_plot = self._new_plot("DC link current", "Ampere")
        self.idc_l_curve = self.idc_plot.plot(name="Idc L", pen=pens[0]); self.idc_r_curve = self.idc_plot.plot(name="Idc R", pen=pens[1])
        tabs.addTab(self.idc_plot, "DC Current")

        self.isr_plot = self._new_plot("Motor ISR execution cycles", "cycles")
        self.isr_curve = self.isr_plot.plot(name="ISR cycles", pen=pens[0])
        self.isr_budget = pg.InfiniteLine(pos=FOC_BUDGET_CYCLES, angle=0, movable=False, label="16 kHz budget")
        self.isr_plot.addItem(self.isr_budget); tabs.addTab(self.isr_plot, "ISR")
        return page

    def build_console_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        layout.addWidget(QtWidgets.QLabel("Latest fixed-width telemetry line"))
        self.raw_line = QtWidgets.QPlainTextEdit(); self.raw_line.setReadOnly(True); self.raw_line.setMaximumBlockCount(4)
        f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont); self.raw_line.setFont(f); self.raw_line.setMaximumHeight(80)
        layout.addWidget(self.raw_line)
        layout.addWidget(QtWidgets.QLabel("Firmware debug / response"))
        self.console = QtWidgets.QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumBlockCount(1000); self.console.setFont(f)
        layout.addWidget(self.console, 1)
        send = QtWidgets.QHBoxLayout(); self.raw_cmd = QtWidgets.QLineEdit(); self.raw_cmd.setPlaceholderText("GET FOC_ISR_CYC  |  GET CTRL_MOD  |  HELP")
        self.raw_cmd.returnPressed.connect(self.send_raw_debug)
        self.raw_send_btn = QtWidgets.QPushButton("Send"); self.raw_send_btn.clicked.connect(self.send_raw_debug)
        send.addWidget(self.raw_cmd, 1); send.addWidget(self.raw_send_btn); layout.addLayout(send)
        return page

    def fit_to_screen(self) -> None:
        screen = QtWidgets.QApplication.primaryScreen()
        if not screen: return
        area = screen.availableGeometry()
        w = min(area.width(), max(820, int(area.width() * 0.92)))
        h = min(area.height(), max(600, int(area.height() * 0.90)))
        self.resize(w, h)
        fg = self.frameGeometry(); fg.moveCenter(area.center()); self.move(fg.topLeft())

    # ---------- Connection ----------
    def refresh_ports(self) -> None:
        current = self.port_combo.currentText(); ports = available_ports()
        self.port_combo.clear(); self.port_combo.addItems(ports)
        preferred = "/dev/ttyUSB0" if "/dev/ttyUSB0" in ports else current
        if preferred in ports: self.port_combo.setCurrentText(preferred)

    def toggle_connection(self) -> None:
        if self.link: self.disconnect_link()
        else: self.connect_link()

    def connect_link(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.warning(self, "Serial", "No serial port selected."); return
        try:
            self.link = HoverLink(port, int(self.baud_combo.currentText()))
            self.link.on_telemetry = self.bridge.telemetry.emit
            self.link.on_debug = self.bridge.debug.emit
            self.link.on_error = self.bridge.error.emit
            self.target_l = self.target_r = 0
            self.tx_timer.start()
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(f"Connected {port} @ {self.link.baud}")
            self.append_console(f"Connected {port} @ {self.link.baud}; firmware telemetry {TELEMETRY_HZ} Hz")
        except Exception as exc:
            self.link = None; QtWidgets.QMessageBox.critical(self, "Serial error", str(exc))
        self.update_controls()

    def disconnect_link(self) -> None:
        if not self.link: return
        moving = self.target_l != 0 or self.target_r != 0 or self.stop_pending or bool(self.latest and not self.latest.stopped)
        if moving:
            QtWidgets.QMessageBox.warning(self, "Safety", "STOP and wait for cmd=0,0 before disconnecting."); return
        self.tx_timer.stop()
        if self.logger.active: self.finish_csv()
        self.link.close(); self.link = None
        self.connect_btn.setText("Connect"); self.statusBar().showMessage("Disconnected"); self.update_controls()

    def on_serial_error(self, message: str) -> None:
        self.append_console(f"[SERIAL ERROR] {message}"); self.statusBar().showMessage(message)

    # ---------- Control ----------
    def is_fully_stopped(self) -> bool:
        return bool(self.target_l == 0 and self.target_r == 0 and self.latest and self.latest.stopped and not self.stop_pending)

    def transmit_target(self) -> None:
        if not self.link: return
        try: self.link.send_command(self.target_l, self.target_r)
        except Exception as exc: self.on_serial_error(str(exc))

    def apply_mode(self) -> None:
        if not self.link: return
        if not self.is_fully_stopped():
            QtWidgets.QMessageBox.warning(self, "Mode", "Mode can change only after STOP and cmd=0,0."); return
        mode = int(self.mode_combo.currentData())
        self.link.send_debug(f"SET CTRL_MOD {mode}"); self.append_console(f"[MODE] requested {mode}={MODE_NAMES[mode]}")

    def start_run(self) -> None:
        if not self.link or not self.latest: return
        if self.latest.calibrating:
            QtWidgets.QMessageBox.warning(self, "Calibration", "Wait for ADC calibration to finish."); return
        if not self.is_fully_stopped():
            QtWidgets.QMessageBox.warning(self, "Start", "Controller must be at full STOP first."); return
        if not self.logger.active:
            path = self.logger.start(self.latest.mode); self.append_console(f"[CSV] 50-Hz recording -> {path}")
        self.target_l = clamp_cmd(self.cmd_l.value()); self.target_r = clamp_cmd(self.cmd_r.value())
        self.append_console(f"[START] cmdL={self.target_l} cmdR={self.target_r}"); self.update_controls()

    def drive(self) -> None:
        if not self.link: return
        if self.stop_pending:
            QtWidgets.QMessageBox.information(self, "Stop", "Wait for rate-limited STOP to finish."); return
        self.target_l = clamp_cmd(self.cmd_l.value()); self.target_r = clamp_cmd(self.cmd_r.value())
        self.append_console(f"[DRIVE] target cmdL={self.target_l} cmdR={self.target_r}")

    def stop_run(self) -> None:
        if not self.link: return
        self.target_l = self.target_r = 0; self.stop_pending = True; self.stop_zero_frames = 0
        self.append_console("[STOP] target=0,0; firmware rate limiter remains active ..."); self.update_controls()

    def calibrate(self) -> None:
        if not self.link: return
        if not self.is_fully_stopped():
            QtWidgets.QMessageBox.warning(self, "Calibration", "STOP and wait for cmd=0,0 first."); return
        if self.logger.active:
            QtWidgets.QMessageBox.warning(self, "Calibration", "Finish/save the current CSV run first."); return
        ans = QtWidgets.QMessageBox.question(
            self, "Current ADC calibration",
            "Keep both wheels completely stationary. PWM will be disabled during calibration. Recalibrate all six current ADC offsets now?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if ans == QtWidgets.QMessageBox.Yes:
            self.link.send_debug("CALIBRATE"); self.append_console("[CAL] manual ADC offset calibration requested")

    def finish_csv(self) -> None:
        if not self.logger.active: return
        path, rows, missed = self.logger.stop()
        self.append_console(f"[CSV] saved {rows} rows @50Hz -> {path} (missed_seq={missed})")
        self.csv_value.setText("idle")

    # ---------- Telemetry ----------
    @QtCore.pyqtSlot(object)
    def on_telemetry(self, t: Telemetry) -> None:
        self.latest = t; now = time.monotonic(); self.last_arrivals.append(now)
        if self.logger.active: self.logger.write(t)  # every valid 50-Hz firmware frame

        self.left_panel.update_motor(t, False); self.right_panel.update_motor(t, True)
        self.mode_value.setText(f"{t.mode} — {t.mode_name}"); self.dq_ref_value.setText(t.dq_reference)
        self.voltage_value.setText(f"{t.battery_v:.2f} V"); self.temp_value.setText(f"{t.temperature_c:.1f} °C")
        self.cycle_value.setText(f"{t.foc_cycles} cyc | {t.foc_us:.2f} µs | {t.foc_load_pct:.1f}%")
        self.status_value.setText(t.status_text); self.seq_value.setText(str(t.telemetry_seq))
        if len(self.last_arrivals) >= 2:
            dt = self.last_arrivals[-1] - self.last_arrivals[0]
            hz = (len(self.last_arrivals) - 1) / dt if dt > 0 else 0.0
            self.rate_value.setText(f"{hz:.1f} Hz (target {TELEMETRY_HZ})")
        vals = (t.adc_dcl, t.adc_rla, t.adc_rlb, t.adc_dcr, t.adc_rrb, t.adc_rrc)
        for name, val in zip(self.raw_labels, vals): self.raw_labels[name].setText(str(val))
        self.cal_progress.setValue(t.calibration_permille)
        self.raw_line.setPlainText(t.summary())
        if self.logger.active: self.csv_value.setText(f"{self.logger.rows} rows | missed {self.logger.missed}")

        x = now - self.plot_t0
        self.plot_time.append(x); self.plot_rpm_l.append(t.rpm_l); self.plot_rpm_r.append(t.rpm_r)
        self.plot_iq_l.append(t.iq_l_a); self.plot_iq_r.append(t.iq_r_a); self.plot_id_l.append(t.id_l_a); self.plot_id_r.append(t.id_r_a)
        self.plot_idc_l.append(t.idc_l_a); self.plot_idc_r.append(t.idc_r_a); self.plot_cycle.append(t.foc_cycles)

        if self.stop_pending:
            self.stop_zero_frames = self.stop_zero_frames + 1 if t.stopped else 0
            if self.stop_zero_frames >= 3:
                self.stop_pending = False
                self.append_console("[STOP] cmdL=0 cmdR=0 confirmed after rate limiter")
                self.finish_csv(); self.update_controls()
        self.update_controls()

    def refresh_plots(self) -> None:
        x = list(self.plot_time)
        if not x: return
        self.rpm_l_curve.setData(x, list(self.plot_rpm_l)); self.rpm_r_curve.setData(x, list(self.plot_rpm_r))
        self.iq_l_curve.setData(x, list(self.plot_iq_l)); self.iq_r_curve.setData(x, list(self.plot_iq_r))
        self.id_l_curve.setData(x, list(self.plot_id_l)); self.id_r_curve.setData(x, list(self.plot_id_r))
        self.idc_l_curve.setData(x, list(self.plot_idc_l)); self.idc_r_curve.setData(x, list(self.plot_idc_r))
        self.isr_curve.setData(x, list(self.plot_cycle))

    def append_console(self, text: str) -> None:
        self.console.appendPlainText(text)

    def send_raw_debug(self) -> None:
        if not self.link: return
        line = self.raw_cmd.text().strip()
        if line:
            self.link.send_debug(line); self.append_console(f"> {line}"); self.raw_cmd.clear()

    def update_controls(self) -> None:
        connected = self.link is not None
        stopped = self.is_fully_stopped()
        calibrating = bool(self.latest and self.latest.calibrating)
        self.mode_btn.setEnabled(connected and stopped and not calibrating)
        self.start_btn.setEnabled(connected and stopped and not calibrating and not self.logger.active)
        self.drive_btn.setEnabled(connected and not self.stop_pending and not calibrating)
        self.stop_btn.setEnabled(connected)
        self.cal_btn.setEnabled(connected and stopped and not calibrating and not self.logger.active)
        self.raw_send_btn.setEnabled(connected); self.raw_cmd.setEnabled(connected)
        self.port_combo.setEnabled(not connected); self.baud_combo.setEnabled(not connected); self.refresh_btn.setEnabled(not connected)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        moving = self.link and (self.target_l != 0 or self.target_r != 0 or self.stop_pending or bool(self.latest and not self.latest.stopped))
        if moving:
            QtWidgets.QMessageBox.warning(self, "Safety", "STOP and wait for cmd=0,0 before closing the GUI.")
            event.ignore(); return
        self.tx_timer.stop()
        if self.logger.active: self.finish_csv()
        if self.link: self.link.close(); self.link = None
        event.accept()


def main() -> int:
    if hasattr(QtCore.Qt, "AA_EnableHighDpiScaling"):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(QtCore.Qt, "AA_UseHighDpiPixmaps"):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Hoverboard FOC Control")
    win = MainWindow(); win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
