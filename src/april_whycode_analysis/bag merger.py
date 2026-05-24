#!/usr/bin/env python3
# Fixed concatenation merger for ROS Noetic bag files.
# - Concatenates bags sorted numerically by filename.
# - Shifts timestamps of subsequent bags so playback is continuous (no large time gaps / backward jumps).
# - Also shifts common message header stamps and TF stamps so in-bag timestamps stay consistent.
# - Paths are hard-coded per user request. Logs written to merge.log alongside output bag.

import os
import re
import time
import logging
import shutil
import rosbag
import sys

import rospy  # for Time construction

SOURCE_DIR = "/root/ros1_ws/april_whycode_test_bags"
OUTPUT_BAG = os.path.join(SOURCE_DIR, "merged2.bag")
LOG_FILE = os.path.join(SOURCE_DIR, "merge.log")
BACKUP_SUFFIX = ".bak"
EPS_SEC = 1e-6  # small gap between bags (1 microsecond)

# Configure logging: file + console
logger = logging.getLogger("bag_merger")
logger.setLevel(logging.DEBUG)
fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

fh = logging.FileHandler(LOG_FILE, mode="a")
fh.setLevel(logging.DEBUG)
fh.setFormatter(fmt)
logger.addHandler(fh)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.INFO)
ch.setFormatter(fmt)
logger.addHandler(ch)


def numeric_key(fname):
    base = os.path.basename(fname)
    name = os.path.splitext(base)[0]
    # Prefer a numeric sort when a leading integer exists; always return a consistent tuple
    m = re.match(r"^(\d+)", name)
    if m:
        return (0, int(m.group(1)))
    # non-numeric names sort after numeric ones, case-insensitive
    return (1, name.lower())


def find_bags(directory):
    files = [f for f in os.listdir(directory) if f.endswith(".bag")]
    files.sort(key=numeric_key)
    full_paths = [os.path.join(directory, f) for f in files]
    return full_paths


def backup_existing(path):
    if os.path.exists(path):
        ts = time.strftime("%Y%m%d-%H%M%S")
        bak = path + BACKUP_SUFFIX + "." + ts
        logger.info("Existing output '%s' detected, moving to '%s'", path, bak)
        shutil.move(path, bak)


def _add_nsec_to_stamp(stamp, add_nsec):
    # stamp: object with .secs and .nsecs (rostime)
    total_nsec = int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs) + int(add_nsec)
    stamp.secs = int(total_nsec // 1_000_000_000)
    stamp.nsecs = int(total_nsec % 1_000_000_000)


def _shift_message_stamps(msg, add_nsec):
    # Generic shift for messages that contain header.stamp
    try:
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            _add_nsec_to_stamp(msg.header.stamp, add_nsec)
    except Exception:
        # non-fatal; continue trying other fields
        pass

    # TFMessage: each transform has a header
    try:
        if hasattr(msg, "transforms"):
            for tr in msg.transforms:
                if hasattr(tr, "header") and hasattr(tr.header, "stamp"):
                    _add_nsec_to_stamp(tr.header.stamp, add_nsec)
    except Exception:
        pass

    # Odometry/pose stamped contain nested headers (odometry has header)
    # The generic header handling above will catch many common messages.
    # If other message types require special handling, extend here.


def merge_bags(sources, output):
    total_written = 0
    logger.info("Opening output bag for write: %s", output)
    outbag = rosbag.Bag(output, "w")
    try:
        last_out_time = None  # float seconds of last written message
        for idx, src in enumerate(sources, start=1):
            logger.info("Processing [%d/%d]: %s", idx, len(sources), src)
            try:
                inbag = rosbag.Bag(src, "r")
            except Exception as e:
                logger.error("Failed to open source bag '%s': %s", src, e)
                continue

            try:
                try:
                    src_start = inbag.get_start_time()
                    src_end = inbag.get_end_time()
                except Exception:
                    # fallback: compute from iteration if Bag API not available
                    src_start = None
                    src_end = None

                if src_start is None:
                    # find first/last by scanning (slow fallback)
                    first_t = None
                    last_t = None
                    for _topic, _msg, t in inbag.read_messages():
                        tsec = float(t.secs) + float(t.nsecs) * 1e-9
                        if first_t is None:
                            first_t = tsec
                        last_t = tsec
                    src_start = first_t if first_t is not None else 0.0
                    src_end = last_t if last_t is not None else src_start

                logger.info("  source time range: %.6f -> %.6f (dur %.6fs)", src_start, src_end, src_end - src_start)

                # compute shift so this bag starts immediately after last_out_time (avoid backward/huge gaps)
                if last_out_time is None:
                    shift = 0.0
                else:
                    shift = (last_out_time + EPS_SEC) - src_start
                    if shift < 0:
                        # if shift negative (src starts after last_out_time) set to 0 to preserve chronological order
                        shift = 0.0

                logger.info("  timestamp shift to apply to this bag: %.6f seconds", shift)
                add_nsec = int(round(shift * 1e9))

                count = 0
                # iterate again from start to write messages with shifted times
                for topic, msg, t in inbag.read_messages():
                    orig_t_sec = float(t.secs) + float(t.nsecs) * 1e-9
                    new_t_sec = orig_t_sec + shift
                    total_nsec = int(round(new_t_sec * 1e9))
                    new_secs = int(total_nsec // 1_000_000_000)
                    new_nsecs = int(total_nsec % 1_000_000_000)

                    # shift stamps inside the message so header.stamp etc are consistent with new bag timeline
                    try:
                        _shift_message_stamps(msg, add_nsec)
                    except Exception as e:
                        logger.debug("    warning shifting inner stamps for topic %s: %s", topic, e)

                    outbag.write(topic, msg, rospy.Time(new_secs, new_nsecs))

                    count += 1
                    if count % 5000 == 0:
                        logger.info("    ...written %d messages from %s so far", count, src)

                logger.info("  finished writing %d messages from %s", count, src)
                total_written += count

                # update last_out_time to end of this bag after shift
                last_out_time = (src_end + shift) if src_end is not None else (last_out_time if last_out_time is not None else 0.0)

            except Exception as e:
                logger.exception("Error while reading/writing from '%s': %s", src, e)
            finally:
                inbag.close()
    finally:
        outbag.close()

    logger.info("Merge complete. Total messages written: %d", total_written)
    try:
        size_bytes = os.path.getsize(output)
        logger.info("Output bag size: %.2f MB", size_bytes / (1024.0 * 1024.0))
    except Exception:
        pass
    return total_written


if __name__ == "__main__":
    logger.info("Starting bag concatenation in directory: %s", SOURCE_DIR)

    if not os.path.isdir(SOURCE_DIR):
        logger.error("Source directory does not exist: %s", SOURCE_DIR)
        sys.exit(1)

    sources = find_bags(SOURCE_DIR)
    if not sources:
        logger.error("No .bag files found in %s", SOURCE_DIR)
        sys.exit(1)

    logger.info("Found %d bag(s) to merge (sorted):", len(sources))
    for s in sources:
        logger.info("  %s", s)

    # Backup existing merged output if present
    if os.path.exists(OUTPUT_BAG):
        backup_existing(OUTPUT_BAG)

    try:
        total = merge_bags(sources, OUTPUT_BAG)
        logger.info("SUCCESS: Merged %d bags into '%s' (%d messages)", len(sources), OUTPUT_BAG, total)
    except Exception as e:
        logger.exception("Merge failed: %s", e)
        sys.exit(2)