#!/usr/bin/env python3
import cantools
import can
import curses
import time
from datetime import datetime

dbc_path = '/home/jay/Arduino/libraries/NUCAN/DBC Files/AV1.dbc'
db = cantools.database.load_file(dbc_path)
messages = {m.frame_id: m for m in db.messages}
msg_names = [m.name for m in db.messages]

# Dictionary to store latest values and timestamps
latest = {name: {'data': {}, 'timestamp': None} for name in msg_names}

def can_reader(latest):
    bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=250000)
    for msg in bus:
        current_time = time.time()
        try:
            m = messages[msg.arbitration_id]
            decoded = m.decode(msg.data)
            latest[m.name] = {
                'data': decoded,
                'timestamp': current_time
            }
        except KeyError:
            # Unknown frame ID
            frame_id = f"0x{msg.arbitration_id:X}"
            if frame_id not in latest:
                latest[frame_id] = {'data': {}, 'timestamp': None}
            latest[frame_id] = {
                'data': {"raw": msg.data.hex()},
                'timestamp': current_time
            }

def format_time_ago(timestamp):
    if timestamp is None:
        return "Never"
    
    elapsed = time.time() - timestamp
    if elapsed < 1:
        return f"{elapsed*1000:.0f}ms ago"
    elif elapsed < 60:
        return f"{elapsed:.1f}s ago"
    elif elapsed < 3600:
        return f"{elapsed/60:.1f}m ago"
    else:
        return f"{elapsed/3600:.1f}h ago"

def dashboard(stdscr, latest, msg_names):
    curses.curs_set(0)
    stdscr.nodelay(True)
    
    while True:
        stdscr.erase()
        stdscr.addstr(0, 0, "Live CAN DBC Decode Dashboard with Timestamps\n(CTRL+C to exit)\n")
        stdscr.addstr(1, 0, f"Current time: {datetime.now().strftime('%H:%M:%S')}\n")
        
        row = 3
        for name in msg_names:
            msg_info = latest.get(name, {'data': {}, 'timestamp': None})
            values = msg_info['data']
            time_str = format_time_ago(msg_info['timestamp'])
            
            if values:
                # Display message name header
                try:
                    stdscr.addstr(row, 0, f"{name}: | {time_str}")
                    row += 1
                except curses.error:
                    break
                
                # Display each signal on its own line
                for signal_name, signal_value in values.items():
                    signal_line = f"  {signal_name:<30}: {signal_value}"
                    try:
                        stdscr.addstr(row, 0, signal_line)
                        row += 1
                    except curses.error:
                        break
                
                # Add blank line between messages for readability
                row += 1
            else:
                # No data received yet
                display_line = f"{name:<25}: No data | {time_str}"
                try:
                    stdscr.addstr(row, 0, display_line)
                    row += 1
                except curses.error:
                    break
        
        # Show any unknown IDs
        unknowns = [k for k in latest.keys() if k not in msg_names]
        if unknowns:
            try:
                stdscr.addstr(row, 0, "--- Unknown Frame IDs ---")
                row += 1
            except curses.error:
                pass
                
            for unk in unknowns:
                msg_info = latest.get(unk, {'data': {}, 'timestamp': None})
                values = msg_info['data']
                time_str = format_time_ago(msg_info['timestamp'])
                
                try:
                    stdscr.addstr(row, 0, f"{unk}: | {time_str}")
                    row += 1
                except curses.error:
                    break
                
                # Display each field on its own line for unknown messages too
                for field_name, field_value in values.items():
                    field_line = f"  {field_name:<30}: {field_value}"
                    try:
                        stdscr.addstr(row, 0, field_line)
                        row += 1
                    except curses.error:
                        break
        
        stdscr.refresh()
        
        # Check for keyboard input
        key = stdscr.getch()
        if key == 3:  # CTRL+C
            break
        
        try:
            curses.napms(100)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    import threading
    t = threading.Thread(target=can_reader, args=(latest,), daemon=True)
    t.start()
    try:
        curses.wrapper(dashboard, latest, msg_names)
    except KeyboardInterrupt:
        print("\nExiting...")
