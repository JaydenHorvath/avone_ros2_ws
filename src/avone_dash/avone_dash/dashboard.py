#!/usr/bin/env python3
import sys
import os
import threading
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
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QEvent
from PyQt5.QtGui import QFont, QKeySequence


# ---------- Thread-safe signal bridge ----------
class SignalBridge(QObject):
    update_signal = pyqtSignal(str, object, dict)


# ---------- ROS2 Node ----------
class AVONEDashboard(Node):
    def __init__(self, bridge, dbc_path):
        super().__init__('avone_dashboard')
        self.bridge = bridge

        # Load DBC
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
        """Record DBC structure (group by message)"""
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
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AV.ONE Dashboard (Grouped)")
        self.setGeometry(100, 100, 1600, 1000)
        self.signal_widgets = {}

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

    def _install_exit_shortcut(self):
        """Press Q or Esc to quit cleanly"""
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
        """Group signals by DBC message"""
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
                name_lbl = QLabel(sig['name'])
                name_lbl.setMinimumWidth(300)

                value_lbl = QLabel('--')
                value_lbl.setAlignment(Qt.AlignRight)
                unit_lbl = QLabel(sig['unit'])
                unit_lbl.setMinimumWidth(60)

                bar = None
                if sig['min'] is not None and sig['max'] is not None:
                    bar = QProgressBar()
                    try:
                        bar.setMinimum(int(sig['min']))
                        bar.setMaximum(int(sig['max']))
                    except Exception:
                        pass
                    bar.setTextVisible(False)
                    bar.setMinimumWidth(240)
                    grid.addWidget(bar, row, 1)
                    grid.addWidget(value_lbl, row, 2)
                    grid.addWidget(unit_lbl, row, 3)
                else:
                    grid.addWidget(value_lbl, row, 1)
                    grid.addWidget(unit_lbl, row, 2)

                grid.addWidget(name_lbl, row, 0)
                self.signal_widgets[f"{msg}/{sig['name']}"] = {
                    'value_label': value_lbl,
                    'bar': bar,
                    'choices': sig['choices']
                }
                row += 1

            group.setLayout(grid)
            self.layout.addWidget(group)

        self.layout.addStretch()

    def _update_signal(self, key, val, info):
        if key not in self.signal_widgets:
            return
        w = self.signal_widgets[key]
        lbl = w.get('value_label')
        if not lbl:
            return

        # Show enum choices if exist, otherwise raw ROS value
        choices = w.get('choices')
        if choices:
            try:
                lbl.setText(str(choices.get(int(val), int(val))))
            except Exception:
                lbl.setText(str(val))
        else:
            lbl.setText(f"{val:.2f}" if isinstance(val, float) else str(val))

        bar = w.get('bar')
        if bar:
            try:
                bar.setValue(int(val))
            except Exception:
                pass

    def _apply_dark_theme(self):
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
    gui = DashboardGUI()
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
