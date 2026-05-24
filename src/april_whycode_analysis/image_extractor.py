#!/usr/bin/env python3
"""
Bag extractor+sequential-timestamp merger for ROS Noetic.
- Hard-coded source directory and output bag.
- Extracts only the /color/image_raw topic (sensor_msgs/Image).
- Merges all source bags by message timestamp (rosbag message time).
- Reassigns timestamps so messages are strictly sequential (no large gaps): uses the median inter-frame delta
  (ignoring large outliers) and applies that spacing to every image in merged order.
- Writes detailed logs to merge_images.log in the source directory.
"""
import os
import re
import time
import logging
import shutil
import heapq
import sys
import statistics

import rosbag
import rospy

SOURCE_DIR = "/root/ros1_ws/april_whycode_test_bags"
OUTPUT_BAG = os.path.join(SOURCE_DIR, "merged_images_2.bag")
LOG_FILE = os.path.join(SOURCE_DIR, "merge_images.log")
BACKUP_SUFFIX = ".bak"
TOPIC = "/color/image_raw"

# Logging setup
logger = logging.getLogger("image_extractor")
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
    m = re.match(r"^(\d+)", name)
    if m:
        return (0, int(m.group(1)))
    return (1, name.lower())


def find_bags(directory):
    files = [f for f in os.listdir(directory) if f.endswith(".bag")]
    files.sort(key=numeric_key)
    return [os.path.join(directory, f) for f in files]


def backup_existing(path):
    if os.path.exists(path):
        ts = time.strftime("%Y%m%d-%H%M%S")
        bak = path + BACKUP_SUFFIX + "." + ts
        logger.info("Existing output '%s' detected, moving to '%s'", path, bak)
        shutil.move(path, bak)


def _time_to_float(t):
    return float(t.secs) + float(t.nsecs) * 1e-9


def _float_to_ros_time(tfloat):
    total_nsec = int(round(tfloat * 1e9))
    secs = int(total_nsec // 1_000_000_000)
    nsecs = int(total_nsec % 1_000_000_000)
    return rospy.Time(secs, nsecs)


def _collect_sorted_timestamps(sources, topic):
    """K-way merge over source bags to produce a sorted list of message times for the topic."""
    times = []
    open_bags = []
    heap = []
    counter = 0
    try:
        for src in sources:
            try:
                bag = rosbag.Bag(src, "r")
            except Exception as e:
                logger.error("Failed to open source bag '%s' for timestamp collection: %s", src, e)
                continue
            open_bags.append(bag)
            it = bag.read_messages(topics=[topic])
            try:
                tp, msg, t = next(it)
            except StopIteration:
                bag.close()
                open_bags.pop()
                continue
            except Exception as e:
                logger.exception("Error reading first message from %s: %s", src, e)
                bag.close()
                open_bags.pop()
                continue
            heapq.heappush(heap, (_time_to_float(t), counter, src, t, it))
            counter += 1

        while heap:
            t_f, _, src, t, it = heapq.heappop(heap)
            times.append(t_f)
            try:
                _tp, _msg, t2 = next(it)
                heapq.heappush(heap, (_time_to_float(t2), counter, src, t2, it))
                counter += 1
            except StopIteration:
                # iterator exhausted for this bag
                pass
            except Exception as e:
                logger.exception("Error advancing iterator for %s while collecting timestamps: %s", src, e)
    finally:
        for b in open_bags:
            try:
                b.close()
            except Exception:
                pass
    return times


def merge_image_topic_sequential(sources, output, topic):
    """
    Two-pass:
      1) collect sorted timestamps for the topic across all bags (k-way merge).
      2) compute typical inter-frame delta (median of diffs ignoring large outliers).
      3) re-open bags and k-way merge again, writing messages with reassigned sequential timestamps
         starting from the first original timestamp and incrementing by the chosen delta.
    """
    logger.info("Collecting sorted timestamps for topic %s from %d source bags", topic, len(sources))
    times = _collect_sorted_timestamps(sources, topic)
    if not times:
        logger.warning("No timestamps found for topic %s", topic)
        return 0

    # compute diffs and median delta; ignore very large gaps (outliers)
    diffs = [j - i for i, j in zip(times[:-1], times[1:])]
    # consider diffs <= 1s as valid frame intervals for median (tunable)
    valid_diffs = [d for d in diffs if 0.0 < d <= 1.0]
    if valid_diffs:
        median_delta = statistics.median(valid_diffs)
    else:
        # fallback if no valid diffs: use median of all positive diffs or default to 1/30
        pos = [d for d in diffs if d > 0.0]
        median_delta = statistics.median(pos) if pos else (1.0 / 30.0)

    logger.info("Timestamps collected: %d images. median inter-frame delta = %.6f s", len(times), median_delta)

    # Second pass: write messages with sequential timestamps
    logger.info("Opening output bag for write: %s", output)
    outbag = rosbag.Bag(output, "w")
    open_bags = []
    heap = []
    counter = 0
    total_written = 0

    try:
        # initialize iterators
        for src in sources:
            try:
                bag = rosbag.Bag(src, "r")
            except Exception as e:
                logger.error("Failed to open source bag '%s' for writing pass: %s", src, e)
                continue
            open_bags.append(bag)
            it = bag.read_messages(topics=[topic])
            try:
                tp, msg, t = next(it)
            except StopIteration:
                bag.close()
                open_bags.pop()
                continue
            except Exception as e:
                logger.exception("Error reading first message from %s in writing pass: %s", src, e)
                bag.close()
                open_bags.pop()
                continue
            heapq.heappush(heap, (_time_to_float(t), counter, src, tp, msg, t, it))
            counter += 1

        # start new timeline at first original timestamp (keeps absolute baseline)
        first_original = times[0]
        next_time = first_original  # next assigned time (float seconds)

        while heap:
            orig_t, _, src, tp, msg, t, it = heapq.heappop(heap)
            # assign sequential time
            assigned_time = next_time
            ros_t = _float_to_ros_time(assigned_time)
            try:
                outbag.write(tp, msg, ros_t)
            except Exception as e:
                logger.exception("Failed to write message from %s: %s", src, e)
            total_written += 1

            # advance next_time by median_delta
            next_time += median_delta

            # advance iterator for this source and push next
            try:
                tp2, msg2, t2 = next(it)
                heapq.heappush(heap, (_time_to_float(t2), counter, src, tp2, msg2, t2, it))
                counter += 1
            except StopIteration:
                logger.debug("Iterator for %s exhausted (wrote images so far %d)", src, total_written)
            except Exception as e:
                logger.exception("Error advancing iterator for %s in writing pass: %s", src, e)

            if total_written % 500 == 0:
                logger.info("  ...total images written so far: %d", total_written)

    finally:
        for b in open_bags:
            try:
                b.close()
            except Exception:
                pass
        try:
            outbag.close()
        except Exception:
            pass

    logger.info("Sequential merge complete. Total images written: %d", total_written)
    try:
        size_bytes = os.path.getsize(output)
        logger.info("Output bag size: %.2f MB", size_bytes / (1024.0 * 1024.0))
    except Exception:
        pass
    return total_written


if __name__ == "__main__":
    logger.info("Starting image extraction+sequential-merge in directory: %s", SOURCE_DIR)

    if not os.path.isdir(SOURCE_DIR):
        logger.error("Source directory does not exist: %s", SOURCE_DIR)
        sys.exit(1)

    sources = find_bags(SOURCE_DIR)
    if not sources:
        logger.error("No .bag files found in %s", SOURCE_DIR)
        sys.exit(1)

    logger.info("Found %d bag(s) (sorted):", len(sources))
    for s in sources:
        logger.info("  %s", s)

    if os.path.exists(OUTPUT_BAG):
        backup_existing(OUTPUT_BAG)

    try:
        total = merge_image_topic_sequential(sources, OUTPUT_BAG, TOPIC)
        logger.info("SUCCESS: Extracted & sequentially-merged topic '%s' into '%s' (%d images)", TOPIC, OUTPUT_BAG, total)
    except Exception as e:
        logger.exception("Merge failed: %s", e)
        sys.exit(2)