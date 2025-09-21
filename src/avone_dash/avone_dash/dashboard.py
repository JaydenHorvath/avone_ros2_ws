# avone_dash/dashboard.py
import threading
import time
import curses
import cantools
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

# std_msgs supported
from std_msgs.msg import Bool, Float32, Float64, Int32, UInt8, UInt16, UInt32

MSG_TYPES = {
    "Bool": Bool,
    "Float32": Float32,
    "Float64": Float64,
    "Int32": Int32,
    "UInt8": UInt8,
    "UInt16": UInt16,
    "UInt32": UInt32,
}


@dataclass
class Item:
    name: str
    topic: str
    type: str
    unit: Optional[str] = None
    choices: Optional[Dict[int, str]] = None
    min_val: Optional[float] = None
    max_val: Optional[float] = None
    value: Any = None
    last_update: float = 0.0

    def as_text(self) -> str:
        """Human-readable value string (with unit and enum decode)."""
        if self.value is None:
            return "—"
        # Enum decode (works even if value arrives as float)
        if self.choices and self.value is not None:
            try:
                return self.choices.get(int(self.value), str(int(self.value)))
            except Exception:
                pass
        # Numeric formatting
        if isinstance(self.value, float):
            s = f"{self.value:.2f}"
        else:
            s = str(self.value)
        return s + (f" {self.unit}" if self.unit else "")

    def percentage(self) -> Optional[float]:
        """Return 0..1 based on min/max (None if not applicable)."""
        if self.value is None:
            return None
        if self.choices is not None:
            return None  # don't bar enums
        if self.min_val is None or self.max_val is None:
            return None
        try:
            v = float(self.value)
        except Exception:
            return None
        lo, hi = float(self.min_val), float(self.max_val)
        if not (hi > lo):
            return None
        # Clamp then normalize
        v = min(max(v, lo), hi)
        return (v - lo) / (hi - lo)


@dataclass
class Group:
    title: str
    items: Dict[str, Item] = field(default_factory=dict)


class AvoneDashboard(Node):
    def __init__(self):
        super().__init__("avone_can_dashboard")
        self.declare_parameters(
            "",
            [
                ("timeout_sec", 0.5),
                ("refresh_rate_hz", 10.0),
                ("show_bars", True),
                ("bar_width", 18),
            ],
        )
        self.timeout_sec: float = float(self.get_parameter("timeout_sec").value)
        self.refresh_rate_hz: float = float(self.get_parameter("refresh_rate_hz").value)
        self.show_bars: bool = bool(self.get_parameter("show_bars").value)
        self.bar_width: int = int(self.get_parameter("bar_width").value)

        # 🔧 Hardcode your DBC file path here (or add a 'dbc_file' param if you prefer)
        dbc_file = "/home/jay/Arduino/libraries/NUCAN/DBC Files/AV1.dbc"

        self.groups: Dict[str, Group] = {}
        self._load_dbc(dbc_file)

        # Launch curses UI
        self._stop_event = threading.Event()
        self.ui_thread = threading.Thread(target=self._curses_main, daemon=True)
        self.ui_thread.start()

    def _load_dbc(self, path: str):
        db = cantools.database.load_file(path)
        all_topics = dict(self.get_topic_names_and_types())

        for msg in db.messages:
            group = Group(title=msg.name)
            for sig in msg.signals:
                topic = f"/av1/{msg.name.lower()}/{sig.name.lower()}"

                # Default type guess
                mtype = "Float32"

                # Override with actual advertised ROS type if available
                if topic in all_topics:
                    ros_type = all_topics[topic][0]  # e.g. "std_msgs/msg/Int32"
                    if ros_type.endswith("Bool"):
                        mtype = "Bool"
                    elif ros_type.endswith("Float64"):
                        mtype = "Float64"
                    elif ros_type.endswith("Int32"):
                        mtype = "Int32"
                    elif ros_type.endswith("UInt8"):
                        mtype = "UInt8"
                    elif ros_type.endswith("UInt16"):
                        mtype = "UInt16"
                    elif ros_type.endswith("UInt32"):
                        mtype = "UInt32"
                    else:
                        mtype = "Float32"  # fallback

                item = Item(
                    name=sig.name,
                    topic=topic,
                    type=mtype,
                    unit=sig.unit,
                    choices=sig.choices,
                    min_val=sig.minimum,
                    max_val=sig.maximum,
                )
                group.items[item.name] = item
                self._subscribe_item(item)

            self.groups[msg.name] = group

        self.get_logger().info(f"Loaded {len(self.groups)} message groups from {path}")

    def _subscribe_item(self, item: Item):
        msg_type = MSG_TYPES.get(item.type)
        if msg_type is None:
            self.get_logger().warn(f"Unsupported type {item.type} for {item.name}")
            return

        def cb(msg):
            item.value = getattr(msg, "data", None)
            item.last_update = time.time()

        self.create_subscription(msg_type, item.topic, cb, 10)
        self.get_logger().info(f"Subscribed to {item.topic} ({item.type})")

    # ---------- UI ----------
    def _curses_main(self):
        curses.wrapper(self._draw_loop)

    def _draw_loop(self, stdscr):
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # good/fresh
        curses.init_pair(2, curses.COLOR_RED, -1)     # fault/off
        curses.init_pair(3, curses.COLOR_YELLOW, -1)  # stale
        curses.init_pair(4, curses.COLOR_CYAN, -1)    # header
        curses.init_pair(5, curses.COLOR_MAGENTA, -1) # group title

        last_refresh = 0.0
        period = 1.0 / max(1e-3, self.refresh_rate_hz)

        while not self._stop_event.is_set():
            now = time.time()
            if now - last_refresh < period:
                time.sleep(0.01)
                continue
            last_refresh = now

            stdscr.erase()
            max_y, max_x = stdscr.getmaxyx()
            title = "AV.ONE – CAN→ROS2 Dashboard (q to quit)"
            stdscr.addstr(0, 0, title[:max_x], curses.A_BOLD | curses.color_pair(4))

            row, col = 2, 0
            col_width = max_x // 2 if max_x >= 120 else max_x

            for gtitle, group in self.groups.items():
                # Move to next column if out of rows
                if row >= max_y - 2:
                    col += col_width
                    row = 2
                    if col >= max_x:
                        break

                header = f"[ {gtitle} ]"
                stdscr.addstr(row, col, header[: max_x - col], curses.A_BOLD | curses.color_pair(5))
                row += 1

                for item in group.items.values():
                    if row >= max_y - 1:
                        break

                    # Value text
                    label = f"{item.name:>20}: "
                    val_txt = item.as_text()
                    base_text = label + val_txt

                    # Freshness color
                    age = now - item.last_update if item.last_update else 999
                    stale = age > self.timeout_sec
                    color = curses.color_pair(3) if stale else curses.color_pair(1)

                    # Optional progress bar
                    text = base_text
                    if self.show_bars and not stale:
                        pct = item.percentage()
                        if pct is not None:
                            # Compute bar space left on this line
                            avail = max_x - col - len(base_text) - 1
                            bar_w = min(max(10, self.bar_width), max(0, avail - 3))  # [####-----]
                            if bar_w > 0:
                                filled = int(round(pct * bar_w))
                                empty = bar_w - filled
                                bar = " [" + ("#" * filled) + ("-" * empty) + "]"
                                text = base_text + bar

                    # Clip to line width
                    text = text[: max_x - col - 1]
                    stdscr.addstr(row, col, text, color)
                    row += 1

                row += 1

            stdscr.refresh()
            stdscr.nodelay(True)
            try:
                if stdscr.getch() in (ord("q"), ord("Q")):
                    self._stop_event.set()
                    break
            except Exception:
                pass

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main():
    rclpy.init()
    node = AvoneDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
