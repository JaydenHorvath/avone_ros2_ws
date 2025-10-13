#!/usr/bin/env python3
import sys
import os
import threading
import collections
import cantools
import rclpy
from rclpy.node import Node

# ROS2 message types
from std_msgs.msg import (
    Bool, Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64
)

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QProgressBar,
    QScrollArea, QGroupBox, QGridLayout, QPushButton
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QKeySequence

# PyQtGraph for live plotting
import pyqtgraph as pg


# ---------- Thread-safe signal bridge ----------
class SignalBridge(QObject):
    update_signal = pyqtSignal(str, object, dict)


# ---------- ROS2 Node ----------
class AVONEDashboard(Node):
    def __init__(self, bridge, dbc_path):
        super().__init__('avone_dashboard')
        self.bridge = bridge

        try:
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f'Loaded DBC file: {dbc_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load DBC file: {e}')
            sys.exit(1)

        self.signal_values = {}
        self.callback_count = {}

        self._parse_dbc()
        self._create_subscriptions()
        self.create_timer(5.0, self._report_stats)

    def _parse_dbc(self):
        for msg in self.db.messages:
            for sig in msg.signals:
                self.signal_values[f"{msg.name}/{sig.name}"] = {
                    'name': sig.name,
                    'message': msg.name,
                    'unit': sig.unit or '',
                    'min': sig.minimum,
                    'max': sig.maximum,
                    'choices': getattr(sig, 'choices', None),
                    'value': None,
                }

    def _create_subscriptions(self):
        type_map = {
            'bool': Bool, 'int8': Int8, 'uint8': UInt8,
            'int16': Int16, 'uint16': UInt16,
            'int32': Int32, 'uint32': UInt32,
            'float32': Float32, 'float64': Float64
        }
        topics = dict(self.get_topic_names_and_types())

        for key, info in self.signal_values.items():
            msg, sig = info['message'], info['name']
            topic = f'/av1/{msg.lower()}/{sig.lower()}'
            self.callback_count[key] = 0

            msg_type = Float32  # default
            if topic in topics:
                full = topics[topic][0]
                base = full.split('/')[-1].lower()
                msg_type = type_map.get(base, Float32)

            try:
                self.create_subscription(
                    msg_type, topic,
                    lambda m, k=key: self._callback(m, k), 10
                )
                self.get_logger().info(f"Subscribed: {topic} ({msg_type.__name__})")
            except Exception as e:
                self.get_logger().warn(f"Failed {topic}: {e}")

    def _callback(self, msg, key):
        if key not in self.signal_values:
            return
        val = getattr(msg, 'data', msg)
        self.signal_values[key]['value'] = val
        self.callback_count[key] += 1
        self.bridge.update_signal.emit(key, val, self.signal_values[key])

    def _report_stats(self):
        active = sum(1 for v in self.callback_count.values() if v > 0)
        total = len(self.callback_count)
        self.get_logger().info(f"Active signals: {active}/{total}")


# ---------- GUI ----------
class DashboardGUI(QMainWindow):
    def __init__(self, enable_plots=True):
        super().__init__()
        self.setWindowTitle("AV.ONE Dashboard (Grouped + Plots)")
        self.setGeometry(100, 100, 1600, 1000)
        self.signal_widgets = {}
        self.enable_plots = enable_plots

        self.bridge = SignalBridge()
        self.bridge.update_signal.connect(self._update_signal)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        self.layout = QVBoxLayout(container)
        scroll.setWidget(container)
        self.setCentralWidget(scroll)

        self._apply_dark_theme()
        self._install_exit_shortcut()

        # Timer to update plots
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(200)  # 5 Hz refresh rate

    def _install_exit_shortcut(self):
        quit_button = QPushButton("Exit (Q/Esc)")
        quit_button.clicked.connect(self._exit_app)
        quit_button.setFixedWidth(150)
        quit_button.setStyleSheet("background-color:#b33a3a; color:white; font-weight:bold;")
        self.layout.addWidget(quit_button)

        self.addAction(self._make_shortcut("Q", self._exit_app))
        self.addAction(self._make_shortcut("Escape", self._exit_app))

    def _make_shortcut(self, key, func):
        from PyQt5.QtWidgets import QAction
        act = QAction(self)
        act.setShortcut(QKeySequence(key))
        act.triggered.connect(func)
        return act

    def _exit_app(self):
        QApplication.quit()

    def build_ui(self, signal_values):
        grouped = {}
        for key, info in signal_values.items():
            msg = info['message']
            grouped.setdefault(msg, []).append(info)

        for msg, sig_list in grouped.items():
            group = QGroupBox(msg)
            group.setFont(QFont('Arial', 11, QFont.Bold))
            grid = QGridLayout()
            row = 0

            for sig in sorted(sig_list, key=lambda x: x['name']):
                key = f"{msg}/{sig['name']}"
                name_lbl = QLabel(sig['name'])
                name_lbl.setMinimumWidth(250)

                value_lbl = QLabel('--')
                value_lbl.setAlignment(Qt.AlignRight)
                unit_lbl = QLabel(sig['unit'])
                unit_lbl.setMinimumWidth(60)

                # Optional progress bar
                bar = None
                if sig['min'] is not None and sig['max'] is not None:
                    bar = QProgressBar()
                    bar.setMinimum(int(sig['min']))
                    bar.setMaximum(int(sig['max']))
                    bar.setTextVisible(False)
                    bar.setMinimumWidth(150)
                    grid.addWidget(bar, row, 1)
                    grid.addWidget(value_lbl, row, 2)
                    grid.addWidget(unit_lbl, row, 3)
                else:
                    grid.addWidget(value_lbl, row, 1)
                    grid.addWidget(unit_lbl, row, 2)

                # Optional live plot
                plot_widget = None
                data_buffer = None
                if self.enable_plots:
                    plot_widget = pg.PlotWidget()
                    plot_widget.setYRange(-1, 1)  # auto-adjust later
                    plot_widget.setFixedHeight(100)
                    plot_widget.showGrid(x=True, y=True, alpha=0.3)
                    plot_curve = plot_widget.plot(pen=pg.mkPen('#00BFFF', width=2))
                    data_buffer = collections.deque(maxlen=100)
                    grid.addWidget(plot_widget, row, 4, 1, 1)
                else:
                    plot_curve = None

                grid.addWidget(name_lbl, row, 0)
                self.signal_widgets[key] = {
                    'value_label': value_lbl,
                    'bar': bar,
                    'choices': sig['choices'],
                    'plot_widget': plot_widget,
                    'plot_curve': plot_curve,
                    'data_buffer': data_buffer
                }
                row += 1

            group.setLayout(grid)
            self.layout.addWidget(group)

        self.layout.addStretch()

    def _update_signal(self, key, val, info):
        if key not in self.signal_widgets:
            return
        w = self.signal_widgets[key]
        lbl = w['value_label']
        if not lbl:
            return

        choices = w['choices']
        if choices:
            lbl.setText(str(choices.get(int(val), int(val))))
        else:
            lbl.setText(f"{val:.2f}" if isinstance(val, float) else str(val))

        bar = w['bar']
        if bar:
            try:
                bar.setValue(int(val))
            except Exception:
                pass

        if self.enable_plots and w['data_buffer'] is not None:
            try:
                w['data_buffer'].append(float(val))
            except Exception:
                pass

    def _refresh_plots(self):
        """Update all live plots"""
        if not self.enable_plots:
            return
        for w in self.signal_widgets.values():
            if w['plot_curve'] and w['data_buffer']:
                data = list(w['data_buffer'])
                w['plot_curve'].setData(data)
                if data:
                    ymin, ymax = min(data), max(data)
                    if ymin == ymax:
                        ymin -= 0.5
                        ymax += 0.5
                    w['plot_widget'].setYRange(ymin, ymax)

    def _apply_dark_theme(self):
        pg.setConfigOption('background', '#1e1e1e')
        pg.setConfigOption('foreground', '#e0e0e0')
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e1e; }
            QWidget { background-color: #1e1e1e; color: #e0e0e0; }
            QGroupBox { border: 1px solid #3d3d3d; border-radius: 5px; padding: 8px; margin-top: 1ex; }
            QLabel { color: #e0e0e0; }
            QProgressBar { border: 1px solid #3d3d3d; background-color: #2d2d2d; }
            QProgressBar::chunk { background-color: #2196F3; }
        """)


# ---------- Main ----------
def main():
    rclpy.init()
    dbc_path = '/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc'
    if not os.path.exists(dbc_path):
        print("DBC file not found:", dbc_path)
        sys.exit(1)

    app = QApplication(sys.argv)
    gui = DashboardGUI(enable_plots=True)  # toggle live plots here
    gui.show()

    node = AVONEDashboard(gui.bridge, dbc_path)
    gui.build_ui(node.signal_values)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
