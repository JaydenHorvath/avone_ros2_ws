#!/usr/bin/env python3

# dashboard(AV.ONE live signal dashboard)
# ----------------------------------------
# PyQt5 + PyQtGraph dashboard that visualises AV.ONE ROS 2 topics derived from the AV1 DBC.

# What it does:
#   - Loads the AV1 DBC file (cantools) and enumerates every message and signal.
#   - For each DBC signal, subscribes to the corresponding ROS topic published by the CAN→ROS bridge:
#       /av1/<message_name_lower>/<signal_name_lower>
#   - Displays each signal in a tabbed UI, grouped by message name:
#       - Live numeric value (and enum label if the signal has choices)
#       - Optional progress bar when DBC min/max exist
#       - Optional scrolling plot (last ~100 samples) via PyQtGraph
#   - Runs ROS 2 spinning on a background thread, and forwards updates into the GUI thread safely using Qt signals.

# Intended use:
#   - Fast bring-up tool for CAN and autonomy stack debugging.
#   - Confirm heartbeats, modes/states, faults, and sensor streams are alive without needing rqt.

# How to run:
#   - Start your CAN→ROS bridge (topics must exist), then:
#       ros2 run avone_dash dashboard
#   - Close with Q or Esc (or the Exit button).

# Key assumptions / gotchas:
#   - DBC path is currently hard coded (dbc_path). Will need to be updated with NUCAN git repository path
#   - Topic message types are inferred from the running ROS graph at startup. If topics appear later,
#     the type may default to Float32 and the subscription may not match.
#   - Many subscriptions are created (one per signal). On slower machines this may be heavy.


#!/usr/bin/env python3
import sys
import os
import threading
import collections
from functools import partial

import cantools
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# ROS2 message types
from std_msgs.msg import (
    Bool,
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Float32,
    Float64,
)

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QLabel,
    QProgressBar,
    QScrollArea,
    QGroupBox,
    QGridLayout,
    QPushButton,
    QTabWidget,
    QAction,
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QKeySequence

# PyQtGraph for live plotting
import pyqtgraph as pg


# =====================================================
# MANUAL CLASSIFICATION MAP
# =====================================================
CLASS_MAP = {
    "Drive": ["THROTTLE", "BRK", "RPM", "GEAR", "ROS", "CMD"],
    "Faults": ["FAULT", "ERROR", "TIMEOUT", "ERR"],
    "Sensors": ["IMU", "GPS"],
    "Motors": ["MOTOR", "CURR", "VOLT"],
    "System": ["HEARTBEAT", "MODE", "ARMED", "STATE", "HEARTBEAT", "HB"],
    "SLAB": ["MSGID", "DCDC", "CHARGER"],
}
# Any message not matching a keyword above goes into "Misc"


# ---------- Thread-safe signal bridge ----------
class SignalBridge(QObject):
    update_signal = pyqtSignal(str, object, dict)


# ---------- ROS2 Node ----------
class AVONEDashboard(Node):
    def __init__(self, bridge, dbc_path):
        super().__init__("avone_dashboard")
        self.bridge = bridge

        try:
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f"Loaded DBC file: {dbc_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC file: {e}")
            sys.exit(1)

        self.signal_values = {}
        self.callback_count = {}

        # Keep subscriptions alive and avoid duplicates
        self._subs = []
        self._subscribed = set()  # (topic, full_type_string)

        # QoS tuned for bag playback / best effort publishers
        self._qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._parse_dbc()

        # Initial attempt
        self._rescan_and_subscribe()

        # Keep checking for topics that appear later (eg when ros2 bag play starts)
        self.create_timer(1.0, self._rescan_and_subscribe)

        self.create_timer(5.0, self._report_stats)

    def _parse_dbc(self):
        for msg in self.db.messages:
            for sig in msg.signals:
                self.signal_values[f"{msg.name}/{sig.name}"] = {
                    "name": sig.name,
                    "message": msg.name,
                    "unit": sig.unit or "",
                    "min": sig.minimum,
                    "max": sig.maximum,
                    "choices": getattr(sig, "choices", None),
                    "value": None,
                }

    def _resolve_msg_class(self, full_type: str):
        full_type_map = {
            "std_msgs/msg/Bool": Bool,
            "std_msgs/msg/Int8": Int8,
            "std_msgs/msg/UInt8": UInt8,
            "std_msgs/msg/Int16": Int16,
            "std_msgs/msg/UInt16": UInt16,
            "std_msgs/msg/Int32": Int32,
            "std_msgs/msg/UInt32": UInt32,
            "std_msgs/msg/Float32": Float32,
            "std_msgs/msg/Float64": Float64,
        }
        if full_type in full_type_map:
            return full_type_map[full_type]

        # Fallback
        base = full_type.split("/")[-1].lower()
        base_type_map = {
            "bool": Bool,
            "int8": Int8,
            "uint8": UInt8,
            "int16": Int16,
            "uint16": UInt16,
            "int32": Int32,
            "uint32": UInt32,
            "float32": Float32,
            "float64": Float64,
        }
        return base_type_map.get(base, Float32)

    def _topic_for_signal(self, info: dict) -> str:
        msg, sig = info["message"], info["name"]
        return f"/av1/{msg.lower()}/{sig.lower()}"

    def _rescan_and_subscribe(self):
        topics = dict(self.get_topic_names_and_types())

        for key, info in self.signal_values.items():
            topic = self._topic_for_signal(info)
            self.callback_count.setdefault(key, 0)

            if topic not in topics:
                continue

            type_list = topics[topic]
            if len(type_list) > 1:
                self.get_logger().warning(
                    f"Topic has multiple types: {topic} -> {type_list}"
                )

            for full_type in type_list:
                sub_key = (topic, full_type)
                if sub_key in self._subscribed:
                    continue

                msg_cls = self._resolve_msg_class(full_type)
                try:
                    sub = self.create_subscription(
                        msg_cls, topic, partial(self._callback, key=key), self._qos
                    )
                    self._subs.append(sub)
                    self._subscribed.add(sub_key)
                    self.get_logger().info(f"Subscribed: {topic} ({full_type})")
                except Exception as e:
                    self.get_logger().warning(
                        f"Failed subscribe {topic} ({full_type}): {e}"
                    )

    def _callback(self, msg, key):
        if key not in self.signal_values:
            return
        val = getattr(msg, "data", msg)
        self.signal_values[key]["value"] = val
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
        self.setWindowTitle("AV.ONE Dashboard (Tabbed)")
        self.setGeometry(100, 100, 1700, 1000)
        self.signal_widgets = {}
        self.enable_plots = enable_plots

        self.bridge = SignalBridge()
        self.bridge.update_signal.connect(self._update_signal)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self._apply_dark_theme()
        self._install_exit_shortcut()

        # Timer to update plots
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(200)

    # ---------- Exit Shortcuts ----------
    def _install_exit_shortcut(self):
        quit_button = QPushButton("Exit (Q/Esc)")
        quit_button.clicked.connect(self._exit_app)
        quit_button.setFixedWidth(150)
        quit_button.setStyleSheet(
            "background-color:#b33a3a; color:white; font-weight:bold;"
        )
        self.addToolBarBreak()
        self.addToolBar("Exit").addWidget(quit_button)
        self.addAction(self._make_shortcut("Q", self._exit_app))
        self.addAction(self._make_shortcut("Escape", self._exit_app))

    def _make_shortcut(self, key, func):
        act = QAction(self)
        act.setShortcut(QKeySequence(key))
        act.triggered.connect(func)
        return act

    def _exit_app(self):
        QApplication.quit()

    # ---------- UI Build ----------
    def build_ui(self, signal_values):
        # Classify messages
        classified = {k: [] for k in CLASS_MAP.keys()}
        classified["Misc"] = []

        for _, info in signal_values.items():
            msg = info["message"]
            found = False
            for class_name, keywords in CLASS_MAP.items():
                if any(kw in msg.upper() for kw in keywords):
                    classified[class_name].append(info)
                    found = True
                    break
            if not found:
                classified["Misc"].append(info)

        # Build each tab
        for class_name, signals in classified.items():
            tab_scroll = QScrollArea()
            tab_scroll.setWidgetResizable(True)
            tab_container = QWidget()
            layout = QVBoxLayout(tab_container)

            # Group by message name
            grouped = {}
            for info in signals:
                grouped.setdefault(info["message"], []).append(info)

            for msg, sig_list in grouped.items():
                group = QGroupBox(msg)
                group.setFont(QFont("Arial", 11, QFont.Bold))
                grid = QGridLayout()
                row = 0

                for sig in sorted(sig_list, key=lambda x: x["name"]):
                    key = f"{msg}/{sig['name']}"
                    name_lbl = QLabel(sig["name"])
                    name_lbl.setMinimumWidth(250)
                    value_lbl = QLabel("--")
                    value_lbl.setAlignment(Qt.AlignRight)
                    unit_lbl = QLabel(sig["unit"])
                    unit_lbl.setMinimumWidth(60)

                    # Progress bar
                    bar = None
                    if sig["min"] is not None and sig["max"] is not None:
                        bar = QProgressBar()
                        bar.setMinimum(int(sig["min"]))
                        bar.setMaximum(int(sig["max"]))
                        bar.setTextVisible(False)
                        bar.setMinimumWidth(150)
                        grid.addWidget(bar, row, 1)
                        grid.addWidget(value_lbl, row, 2)
                        grid.addWidget(unit_lbl, row, 3)
                    else:
                        grid.addWidget(value_lbl, row, 1)
                        grid.addWidget(unit_lbl, row, 2)

                    # Optional live plot
                    plot_widget, plot_curve, data_buffer = None, None, None
                    if self.enable_plots:
                        plot_widget = pg.PlotWidget()
                        plot_widget.setFixedHeight(100)
                        plot_widget.showGrid(x=True, y=True, alpha=0.3)
                        plot_curve = plot_widget.plot(pen=pg.mkPen("#00BFFF", width=2))
                        data_buffer = collections.deque(maxlen=100)
                        grid.addWidget(plot_widget, row, 4, 1, 1)

                    grid.addWidget(name_lbl, row, 0)
                    self.signal_widgets[key] = {
                        "value_label": value_lbl,
                        "bar": bar,
                        "choices": sig["choices"],
                        "plot_widget": plot_widget,
                        "plot_curve": plot_curve,
                        "data_buffer": data_buffer,
                    }
                    row += 1

                group.setLayout(grid)
                layout.addWidget(group)

            layout.addStretch()
            tab_scroll.setWidget(tab_container)
            self.tabs.addTab(tab_scroll, class_name)

    # ---------- Signal Update ----------
    def _update_signal(self, key, val, info):
        if key not in self.signal_widgets:
            return
        w = self.signal_widgets[key]
        lbl = w["value_label"]
        if not lbl:
            return

        choices = w["choices"]
        if choices:
            try:
                lbl.setText(str(choices.get(int(val), int(val))))
            except Exception:
                lbl.setText(str(val))
        else:
            lbl.setText(f"{val:.2f}" if isinstance(val, float) else str(val))

        if w["bar"]:
            try:
                w["bar"].setValue(int(val))
            except Exception:
                pass

        if self.enable_plots and w["data_buffer"] is not None:
            try:
                w["data_buffer"].append(float(val))
            except Exception:
                pass

    # ---------- Plot Refresh ----------
    def _refresh_plots(self):
        if not self.enable_plots:
            return
        for w in self.signal_widgets.values():
            if w["plot_curve"] and w["data_buffer"]:
                data = list(w["data_buffer"])
                w["plot_curve"].setData(data)
                if data:
                    ymin, ymax = min(data), max(data)
                    if ymin == ymax:
                        ymin -= 0.5
                        ymax += 0.5
                    w["plot_widget"].setYRange(ymin, ymax)

    # ---------- Dark Theme ----------
    def _apply_dark_theme(self):
        pg.setConfigOption("background", "#1e1e1e")
        pg.setConfigOption("foreground", "#e0e0e0")
        self.setStyleSheet(
            """
            QMainWindow { background-color: #1e1e1e; }
            QWidget { background-color: #1e1e1e; color: #e0e0e0; }
            QGroupBox { border: 1px solid #3d3d3d; border-radius: 5px; padding: 8px; margin-top: 1ex; }
            QLabel { color: #e0e0e0; }
            QProgressBar { border: 1px solid #3d3d3d; background-color: #2d2d2d; }
            QProgressBar::chunk { background-color: #2196F3; }
            QTabBar::tab { background: #2d2d2d; color: #e0e0e0; padding: 10px; }
            QTabBar::tab:selected { background: #2196F3; color: white; }
        """
        )


# ---------- Main ----------
def main():
    rclpy.init()
    dbc_path = "/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc"
    if not os.path.exists(dbc_path):
        print("DBC file not found:", dbc_path)
        sys.exit(1)

    app = QApplication(sys.argv)
    gui = DashboardGUI(enable_plots=True)
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


if __name__ == "__main__":
    main()
