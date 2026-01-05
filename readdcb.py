#!/usr/bin/env python3
"""
CAN Bus Dashboard - A beautiful terminal UI for monitoring CAN messages
Shows ALL messages with scrolling support
Requires: pip install rich cantools python-can
"""
import cantools
import can
import time
import threading
import sys
from datetime import datetime
from collections import OrderedDict

from rich.console import Console, Group
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.text import Text
from rich import box

# Configuration
DBC_PATH = '/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc'

# Load DBC
db = cantools.database.load_file(DBC_PATH)
messages = {m.frame_id: m for m in db.messages}
msg_names = [m.name for m in db.messages]

# Dictionary to store latest values and timestamps
latest = {name: {'data': {}, 'timestamp': None, 'count': 0} for name in msg_names}
unknown_messages = OrderedDict()
stats = {'total_msgs': 0, 'msgs_per_sec': 0, 'last_count': 0, 'last_time': time.time()}


def format_time_ago(timestamp):
    """Format timestamp as human-readable time ago."""
    if timestamp is None:
        return "[dim]Never[/dim]"
    
    elapsed = time.time() - timestamp
    if elapsed < 0.1:
        return "[green bold]NOW[/green bold]"
    elif elapsed < 1:
        return f"[green]{elapsed*1000:.0f}ms[/green]"
    elif elapsed < 5:
        return f"[yellow]{elapsed:.1f}s[/yellow]"
    elif elapsed < 60:
        return f"[red]{elapsed:.1f}s[/red]"
    elif elapsed < 3600:
        return f"[dim red]{elapsed/60:.1f}m[/dim red]"
    else:
        return f"[dim]{elapsed/3600:.1f}h[/dim]"


def format_value(value):
    """Format signal values with appropriate styling."""
    if isinstance(value, float):
        return f"[cyan]{value:.3f}[/cyan]"
    elif isinstance(value, int):
        return f"[cyan]{value}[/cyan]"
    elif isinstance(value, str):
        return f"[magenta]{value}[/magenta]"
    else:
        return f"[white]{value}[/white]"


def can_reader():
    """Background thread to read CAN messages."""
    global stats
    bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=250000)
    
    for msg in bus:
        current_time = time.time()
        stats['total_msgs'] += 1
        
        try:
            m = messages[msg.arbitration_id]
            if m.name not in latest:
                latest[m.name] = {'data': {}, 'timestamp': None, 'count': 0}
            
            latest[m.name]['data'] = m.decode(msg.data)
            latest[m.name]['timestamp'] = current_time
            latest[m.name]['count'] += 1
            
        except KeyError:
            # Unknown frame ID
            frame_id = f"0x{msg.arbitration_id:X}"
            if frame_id not in unknown_messages:
                unknown_messages[frame_id] = {'data': {}, 'timestamp': None, 'count': 0}
            
            unknown_messages[frame_id]['data'] = {"raw": msg.data.hex().upper()}
            unknown_messages[frame_id]['timestamp'] = current_time
            unknown_messages[frame_id]['count'] += 1


def calculate_rate():
    """Calculate messages per second."""
    global stats
    current_time = time.time()
    elapsed = current_time - stats['last_time']
    
    if elapsed >= 1.0:
        stats['msgs_per_sec'] = (stats['total_msgs'] - stats['last_count']) / elapsed
        stats['last_count'] = stats['total_msgs']
        stats['last_time'] = current_time


def get_freshness_style(timestamp):
    """Get border style based on message freshness."""
    if timestamp is None:
        return "dim white"
    elapsed = time.time() - timestamp
    if elapsed < 1:
        return "bold green"
    elif elapsed < 5:
        return "yellow"
    else:
        return "red"


def generate_dashboard(console_height):
    """Generate the complete dashboard."""
    calculate_rate()
    
    # Create main table that holds everything
    main_table = Table(
        box=box.SIMPLE,
        show_header=False,
        show_edge=False,
        padding=0,
        expand=True,
    )
    main_table.add_column("Content", ratio=1)
    
    # Header
    header = Text()
    header.append("🚗 CAN Bus Dashboard", style="bold white")
    header.append("  │  ", style="dim")
    header.append(f"📡 {stats['msgs_per_sec']:.1f} msg/s", style="green")
    header.append("  │  ", style="dim")
    header.append(f"📊 {stats['total_msgs']} total", style="cyan")
    header.append("  │  ", style="dim")
    header.append(f"📋 {len(msg_names)} messages", style="yellow")
    header.append("  │  ", style="dim")
    header.append(f"🕐 {datetime.now().strftime('%H:%M:%S')}", style="white")
    
    main_table.add_row(Panel(header, box=box.DOUBLE, border_style="blue"))
    main_table.add_row("")
    
    # Create the messages table
    msg_table = Table(
        box=box.ROUNDED,
        show_header=True,
        header_style="bold white on blue",
        expand=True,
        padding=(0, 1),
        row_styles=["", "dim"],
    )
    
    msg_table.add_column("Message", style="bold", width=22, no_wrap=True)
    msg_table.add_column("Signals & Values", ratio=1)
    msg_table.add_column("Last", justify="center", width=10)
    msg_table.add_column("#", justify="right", width=7)
    
    # Add all DBC messages
    for name in msg_names:
        msg_info = latest.get(name, {'data': {}, 'timestamp': None, 'count': 0})
        values = msg_info['data']
        time_str = format_time_ago(msg_info['timestamp'])
        count = msg_info.get('count', 0)
        freshness = get_freshness_style(msg_info['timestamp'])
        
        if values:
            # Format all signals on one line with wrapping
            signals_text = Text()
            for i, (signal_name, signal_value) in enumerate(values.items()):
                if i > 0:
                    signals_text.append("  │  ", style="dim")
                signals_text.append(f"{signal_name}: ", style="white")
                
                # Color code value
                if isinstance(signal_value, float):
                    signals_text.append(f"{signal_value:.2f}", style="cyan bold")
                elif isinstance(signal_value, int):
                    signals_text.append(f"{signal_value}", style="cyan bold")
                elif isinstance(signal_value, str):
                    signals_text.append(f"{signal_value}", style="magenta bold")
                else:
                    signals_text.append(f"{signal_value}", style="white")
            
            msg_table.add_row(
                Text(name, style=freshness),
                signals_text,
                time_str,
                f"[dim]{count}[/dim]",
            )
        else:
            msg_table.add_row(
                Text(name, style="dim"),
                Text("— waiting for data —", style="dim italic"),
                time_str,
                "[dim]0[/dim]",
            )
    
    main_table.add_row(msg_table)
    
    # Unknown messages section
    if unknown_messages:
        main_table.add_row("")
        
        unk_table = Table(
            title="[bold yellow]⚠️  Unknown Frame IDs[/bold yellow]",
            box=box.ROUNDED,
            show_header=True,
            header_style="bold black on yellow",
            expand=True,
            padding=(0, 1),
        )
        
        unk_table.add_column("Frame ID", style="yellow bold", width=12)
        unk_table.add_column("Raw Data (hex)", ratio=1)
        unk_table.add_column("Last", justify="center", width=10)
        unk_table.add_column("#", justify="right", width=7)
        
        for frame_id, msg_info in unknown_messages.items():
            raw_data = msg_info['data'].get('raw', '')
            formatted_raw = ' '.join(raw_data[i:i+2] for i in range(0, len(raw_data), 2))
            time_str = format_time_ago(msg_info['timestamp'])
            count = msg_info.get('count', 0)
            
            unk_table.add_row(
                frame_id,
                Text(formatted_raw, style="yellow"),
                time_str,
                f"[dim]{count}[/dim]",
            )
        
        main_table.add_row(unk_table)
    
    # Footer
    main_table.add_row("")
    footer = Text()
    footer.append("  Ctrl+C", style="bold")
    footer.append(" to exit  │  ", style="dim")
    footer.append("Green", style="green")
    footer.append("=active  ", style="dim")
    footer.append("Yellow", style="yellow")
    footer.append("=stale  ", style="dim")
    footer.append("Red", style="red")
    footer.append("=old", style="dim")
    main_table.add_row(footer)
    
    return main_table


def main():
    """Main entry point."""
    console = Console()
    
    # Start CAN reader thread
    reader_thread = threading.Thread(target=can_reader, daemon=True)
    reader_thread.start()
    
    console.print("[bold green]Starting CAN Dashboard...[/bold green]")
    console.print("[dim]Press Ctrl+C to exit[/dim]\n")
    time.sleep(0.5)
    
    try:
        with Live(
            generate_dashboard(console.height),
            console=console,
            refresh_per_second=10,
            screen=True,
            vertical_overflow="visible",
        ) as live:
            while True:
                live.update(generate_dashboard(console.height))
                time.sleep(0.1)
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Dashboard stopped.[/bold yellow]")
        console.print(f"[dim]Total messages received: {stats['total_msgs']}[/dim]")


if __name__ == "__main__":
    main()
