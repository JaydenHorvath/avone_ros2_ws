#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, Int32, UInt8, UInt16, UInt32
import cantools
import curses
import threading
import time
import re
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

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
    value: Any = None
    last_update: float = 0.0

    def as_text(self) -> str:
        if self.value is None:
            return "—"
        if isinstance(self.value, float):
            return f"{self.value:.2f}"
        return str(self.value)


@dataclass
class Group:
    title: str
    items: Dict[str, Item] = field(default_factory=dict)


class AvoneLiveDashboard(Node):
    def __init__(self):
        super().__init__("avone_live_dashboard")

        self.declare_parameters(
            "",
            [
                ("dbc_file", "/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc"),
                ("refresh_rate_hz", 10.0),
                ("timeout_sec", 0.5),
            ],
        )

        self.dbc_file = self.get_parameter("dbc_file").value
        self.refresh_rate = float(self.get_parameter("refresh_rate_hz").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.groups: Dict[str, Group] = {}
        self._stop_event = threading.Event()
        self._subscriptions = set()

        # Load DBC
        self._load_dbc()

        # Periodically rescan for ROS topics
        self.create_timer(1.0, self._rescan_topics)

        # Launch curses UI
        self.ui_thread = threading.Thread(target=self._run_curses, daemon=True)
        self.ui_thread.start()

    # ---------- Setup ----------

    def _load_dbc(self):
        self.db = cantools.database.load_file(self.dbc_file)
        self.get_logger().info(f"Loaded {len(self.db.messages)} messages from {self.dbc_file}")

        for msg in self.db.messages:
            group = Group(title=msg.name)
            for sig in msg.signals:
                topic = f"/av1/{msg.name.lower()}/{sig.name.lower()}"
                group.items[sig.name] = Item(name=sig.name, topic=topic, type="unknown")
            self.groups[msg.name] = group

    # ---------- Topic Matching & Subscriptions ----------

    def _normalize(self, s: str):
        """Simplify name for fuzzy matching."""
        return re.sub(r'[^a-z0-9]', '', s.lower())

    def _rescan_topics(self):
        """Continuously detect live topics and attach subscribers."""
        all_topics = dict(self.get_topic_names_and_types())
        for g in self.groups.values():
            for item in g.items.values():
                if item.topic in self._subscriptions:
                    continue  # already subscribed

                # Try exact match first
                if item.topic in all_topics:
                    ros_type = all_topics[item.topic][0]
                else:
                    # Try fuzzy matching
                    norm_item = self._normalize(item.topic)
                    match = next(
                        (
                            (name, types[0])
                            for name, types in all_topics.items()
                            if self._normalize(name).endswith(norm_item)
                        ),
                        None,
                    )
                    if not match:
                        continue
                    ros_name, ros_type = match
                    item.topic = ros_name  # update to actual

                msg_type_name = ros_type.split("/")[-1]
                msg_type = MSG_TYPES.get(msg_type_name)
                if not msg_type:
                    continue

                self.create_subscription(msg_type, item.topic, self._make_callback(item), 10)
                self._subscriptions.add(item.topic)
                item.type = msg_type_name
                self.get_logger().info(f"Subscribed to {item.topic} ({msg_type_name})")

    def _make_callback(self, item: Item):
        def cb(msg):
            item.value = getattr(msg, "data", None)
            item.last_update = time.time()
        return cb

    # ---------- Curses UI ----------

    def _run_curses(self):
        curses.wrapper(self._draw_loop)

    def _draw_loop(self, stdscr):
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
        curses.init_pair(3, curses.COLOR_RED, -1)
        curses.init_pair(4, curses.COLOR_CYAN, -1)
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)

        last_refresh = 0.0
        scroll_offset = 0
        period = 1.0 / max(1e-3, self.refresh_rate)

        while not self._stop_event.is_set():
            now = time.time()
            if now - last_refresh < period:
                time.sleep(0.01)
                continue
            last_refresh = now

            stdscr.erase()
            max_y, max_x = stdscr.getmaxyx()

            # Layout columns
            if max_x >= 180:
                cols = 3
            elif max_x >= 110:
                cols = 2
            else:
                cols = 1
            col_width = max_x // cols

            stdscr.addstr(0, 0, "AV.ONE – Live ROS2 Dashboard (q to quit)", curses.A_BOLD | curses.color_pair(4))

            lines = []
            for g in self.groups.values():
                lines.append(("group", g.title))
                for item in g.items.values():
                    lines.append(("item", item))

            visible_lines = lines[scroll_offset:scroll_offset + max_y - 3]

            per_col = len(visible_lines) // cols + (len(visible_lines) % cols > 0)

            for c in range(cols):
                start = c * per_col
                end = min(start + per_col, len(visible_lines))
                col_x = c * col_width
                local_row = 2
                for entry_type, content in visible_lines[start:end]:
                    if local_row >= max_y - 2:
                        break
                    if entry_type == "group":
                        stdscr.addstr(local_row, col_x, f"[ {content} ]", curses.A_BOLD | curses.color_pair(5))
                        local_row += 1
                    else:
                        item = content
                        val_txt = item.as_text()
                        label = f"{item.name:>18}: {val_txt}"
                        age = now - item.last_update if item.last_update else 999
                        if age > 1.5:
                            color = curses.color_pair(3)
                        elif age > 0.5:
                            color = curses.color_pair(2)
                        else:
                            color = curses.color_pair(1)
                        stdscr.addstr(local_row, col_x, label[:col_width - 2], color)
                        local_row += 1
                local_row += 1

            stdscr.refresh()
            stdscr.nodelay(True)
            try:
                ch = stdscr.getch()
                if ch in (ord('q'), ord('Q')):
                    self._stop_event.set()
                    break
                elif ch == curses.KEY_DOWN and scroll_offset < len(lines) - (max_y - 3):
                    scroll_offset += 1
                elif ch == curses.KEY_UP and scroll_offset > 0:
                    scroll_offset -= 1
            except Exception:
                pass

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main():
    rclpy.init()
    node = AvoneLiveDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
