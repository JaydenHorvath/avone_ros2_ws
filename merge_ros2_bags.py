#!/usr/bin/env python3
# Merge REF bag (GPS timebase) + retimed LiDAR /quanergy/points from another bag.
# New: --extra-shift-sec and --align-by gps_motion_start (auto-detects motion onset from /fix).
import argparse, os, math
from typing import Dict, Tuple, Optional, List
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

LIDAR_TOPIC = "/quanergy/points"
GPS_TOPIC_DEFAULTS = ["/fix", "/gps/fix", "/navsat/fix"]

def _ns(sec: int, nsec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nsec)
def _split_ns(t: int) -> Tuple[int, int]:
    s = t // 1_000_000_000; ns = t % 1_000_000_000; return int(s), int(ns)
def _has_header(msg) -> bool:
    return hasattr(msg, "header") and hasattr(msg.header, "stamp") \
        and hasattr(msg.header.stamp, "sec") and hasattr(msg.header.stamp, "nanosec")

def _open_reader(bag_path: str) -> Tuple[rosbag2_py.SequentialReader, Dict[str,str]]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
    return r, tmap

def _open_writer(bag_path: str, topics_and_types: Dict[str,str]) -> rosbag2_py.SequentialWriter:
    if os.path.exists(bag_path):
        raise RuntimeError(f"Output bag path already exists: {bag_path}")
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    for topic, tname in topics_and_types.items():
        w.create_topic(rosbag2_py.TopicMetadata(name=topic, type=tname, serialization_format="cdr"))
    return w

def _first_bag_msg_time_ns(bag_path: str) -> int:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    if not r.has_next(): raise RuntimeError(f"No messages in bag: {bag_path}")
    _topic,_data,t = r.read_next(); r.reset(); return int(t)

def _first_header_stamp_ns(bag_path: str) -> Optional[int]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic,data,_t = r.read_next()
        msg_type = get_message(tmap[topic])
        msg = deserialize_message(data, msg_type)
        if _has_header(msg):
            return _ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
    return None

# --- GPS motion start detection from /fix ---
# We compute speed from sequential fixes (Haversine) and look for first time speed > threshold
def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    dlat = math.radians(lat2-lat1); dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(a))

def _find_gps_topic(tmap: Dict[str,str], explicit: Optional[str]) -> Optional[str]:
    if explicit and explicit in tmap: return explicit
    for cand in GPS_TOPIC_DEFAULTS:
        if cand in tmap: return cand
    # fallback: pick any NavSatFix
    for name, t in tmap.items():
        if t.endswith("sensor_msgs/msg/NavSatFix"): return name
    return None

def _gps_motion_start_ns(ref_bag: str, gps_topic: Optional[str], speed_thresh: float, hold_count:int) -> Optional[int]:
    r, tmap = _open_reader(ref_bag)
    topic = _find_gps_topic(tmap, gps_topic)
    if not topic: return None
    msg_type = get_message(tmap[topic])
    # stream and compute speed between consecutive fixes
    prev = None
    over_count = 0
    motion_t = None
    while r.has_next():
        tp, data, t = r.read_next()
        if tp != topic: continue
        msg = deserialize_message(data, msg_type)
        lat = msg.latitude; lon = msg.longitude
        if prev is not None:
            dist = _haversine_m(prev["lat"], prev["lon"], lat, lon)
            dt = (t - prev["t"]) / 1e9
            if dt > 0:
                speed = dist / dt
                if speed >= speed_thresh:
                    over_count += 1
                    if over_count >= hold_count:
                        motion_t = t
                        break
                else:
                    over_count = 0
        prev = {"lat": lat, "lon": lon, "t": t}
    return motion_t

def compute_offset_ns(ref_bag: str, lidar_bag: str, align_by: str, manual_ns: Optional[int],
                      gps_topic: Optional[str], speed_thresh: float, hold_count:int) -> int:
    if manual_ns is not None: return manual_ns
    if align_by == "first-message":
        return _first_bag_msg_time_ns(ref_bag) - _first_bag_msg_time_ns(lidar_bag)
    if align_by == "header":
        ref_hdr = _first_header_stamp_ns(ref_bag); lid_hdr = _first_header_stamp_ns(lidar_bag)
        if ref_hdr is None or lid_hdr is None:
            raise RuntimeError("header align requested but no std_msgs/Header found.")
        return ref_hdr - lid_hdr
    if align_by == "gps_motion_start":
        gps_t = _gps_motion_start_ns(ref_bag, gps_topic, speed_thresh, hold_count)
        if gps_t is None:
            raise RuntimeError("Could not detect GPS motion start; try lowering --speed-thresh or specify --gps-topic.")
        # Align LiDAR FIRST /quanergy/points to GPS motion start:
        lid_first = _first_bag_msg_time_ns(lidar_bag)
        return gps_t - lid_first
    raise ValueError(f"Unknown align_by: {align_by}")

def build_topic_map_for_output(ref_types: Dict[str,str], lidar_types: Dict[str,str], replace_topic: bool) -> Dict[str,str]:
    out = dict(ref_types)
    lidar_type = lidar_types.get(LIDAR_TOPIC, None)
    if lidar_type is None:
        raise RuntimeError(f"LiDAR bag does not contain {LIDAR_TOPIC}")
    if LIDAR_TOPIC in out:
        if out[LIDAR_TOPIC] != lidar_type:
            raise RuntimeError(f"Type mismatch on {LIDAR_TOPIC}: ref has {out[LIDAR_TOPIC]}, lidar has {lidar_type}")
        if replace_topic:
            del out[LIDAR_TOPIC]
    out[LIDAR_TOPIC] = lidar_type
    return out

def main():
    p = argparse.ArgumentParser(description=f"Merge reference bag with retimed {LIDAR_TOPIC} from LiDAR bag.")
    p.add_argument("--ref-bag", required=True)
    p.add_argument("--lidar-bag", required=True)
    p.add_argument("--out-bag", required=True)
    p.add_argument("--align-by", default="first-message",
                   choices=["first-message","header","gps_motion_start"])
    p.add_argument("--offset-sec", type=float, default=None,
                   help="Manual offset (seconds) to ADD to LiDAR timestamps (overrides --align-by).")
    p.add_argument("--extra-shift-sec", type=float, default=0.0,
                   help="Extra constant shift (seconds) applied after the chosen alignment method.")
    p.add_argument("--rewrite-header", action="store_true",
                   help=f"Rewrite std_msgs/Header.stamp on {LIDAR_TOPIC} to shifted time.")
    p.add_argument("--replace-topic", action="store_true",
                   help=f"If {LIDAR_TOPIC} exists in ref, drop it and keep only shifted LiDAR.")
    # GPS motion start options
    p.add_argument("--gps-topic", default=None, help="Explicit GPS /fix topic (defaults guessed).")
    p.add_argument("--speed-thresh", type=float, default=0.20, help="m/s to declare motion start.")
    p.add_argument("--hold-count", type=int, default=2, help="consecutive intervals above threshold to confirm motion.")
    args = p.parse_args()

    rclpy.init()
    ref_reader, ref_types = _open_reader(args.ref_bag)
    lidar_reader, lidar_types = _open_reader(args.lidar_bag)
    out_types = build_topic_map_for_output(ref_types, lidar_types, args.replace_topic)

    manual_ns = None if args.offset_sec is None else int(args.offset_sec * 1e9)
    base_offset = compute_offset_ns(args.ref_bag, args.lidar_bag, args.align_by,
                                    manual_ns, args.gps_topic, args.speed_thresh if hasattr(args,'speed_thresh') else args.speed_thresh, args.hold_count)  # typo fallback
    extra_ns = int(args.extra_shift_sec * 1e9)
    offset_ns = base_offset + extra_ns
    s, ns = _split_ns(offset_ns)
    print(f"[INFO] LiDAR offset: {s}.{ns:09d} s (base={base_offset/1e9:+.6f}s, extra={args.extra_shift_sec:+.6f}s)")

    writer = _open_writer(args.out_bag, out_types)

    # Copy reference messages (skip replacing topic if requested)
    skip = LIDAR_TOPIC if args.replace_topic else None
    ref_reader, _ = _open_reader(args.ref_bag)
    cnt_ref = 0
    while ref_reader.has_next():
        topic, data, t = ref_reader.read_next()
        if skip and topic == skip: continue
        writer.write(topic, data, t); cnt_ref += 1
    print(f"[OK] Copied {cnt_ref} msgs from reference bag.")

    # Inject shifted LiDAR
    msg_type = get_message(lidar_types[LIDAR_TOPIC])
    lidar_reader, _ = _open_reader(args.lidar_bag)
    cnt_lid = 0
    while lidar_reader.has_next():
        topic, data, t = lidar_reader.read_next()
        if topic != LIDAR_TOPIC: continue
        new_t = t + offset_ns
        if args.rewrite_header:
            msg = deserialize_message(data, msg_type)
            if _has_header(msg):
                ss, nn = _split_ns(new_t)
                msg.header.stamp.sec = ss; msg.header.stamp.nanosec = nn
                data = serialize_message(msg)
        writer.write(topic, data, new_t); cnt_lid += 1
    print(f"[OK] Injected {cnt_lid} shifted msgs on {LIDAR_TOPIC}.")
    print(f"[DONE] Wrote: {args.out_bag}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
