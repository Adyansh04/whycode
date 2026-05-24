#!/usr/bin/env python3
import rosbag
from rospy import Time
import copy

def compact_timeline(input_bag_path, output_bag_path, gap_threshold_sec=10.0):
    """
    Remove gaps AND compact timeline to create continuous playback.
    - Collects all messages, splits into recording segments when gaps > gap_threshold_sec.
    - Writes segments back-to-back with a small gap (0.1s) between them.
    - Adjusts message header.stamp when present.
    """
    print("Analyzing and compacting timeline...")

    # Collect all messages with timestamps
    all_messages = []
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            all_messages.append((t.to_sec(), topic, msg, t))

    # Sort by timestamp
    all_messages.sort(key=lambda x: x[0])

    if not all_messages:
        print("No messages found!")
        return

    # Identify recording segments
    segments = []
    current_segment = [all_messages[0]]

    for i in range(1, len(all_messages)):
        prev_ts = all_messages[i - 1][0]
        cur_ts = all_messages[i][0]
        gap = cur_ts - prev_ts

        if gap > gap_threshold_sec:
            # End current segment, start new one
            segments.append(current_segment)
            current_segment = [all_messages[i]]
        else:
            current_segment.append(all_messages[i])

    # Add final segment
    segments.append(current_segment)

    # Print segment summaries
    for idx, seg in enumerate(segments, start=1):
        start_ts = seg[0][0]
        end_ts = seg[-1][0]
        print(f"Segment {idx}: {len(seg)} messages, {start_ts:.3f} -> {end_ts:.3f}")

    # Write compacted timeline
    messages_written = 0
    current_time_offset = 0.0
    gap_between_segments = 0.1

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for segment_idx, segment in enumerate(segments):
            if segment_idx == 0:
                # First segment keeps original timestamps
                time_shift = 0.0
                new_segment_start = segment[0][0]
            else:
                # Subsequent segments: place immediately after current_time_offset
                new_segment_start = current_time_offset
                time_shift = new_segment_start - segment[0][0]

            print(f"Processing segment {segment_idx + 1}: shifting start {segment[0][0]:.3f} -> {new_segment_start:.3f}")

            for orig_timestamp, topic, msg, orig_t in segment:
                # Calculate new timestamp
                new_timestamp = orig_timestamp + time_shift
                new_t = Time.from_sec(new_timestamp)

                # Update message header if it has one (avoid mutating original msg object)
                try:
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                        m = copy.deepcopy(msg)
                        m.header.stamp = new_t
                        outbag.write(topic, m, new_t)
                    else:
                        outbag.write(topic, msg, new_t)
                except Exception as e:
                    print(f"Warning: failed to write/modify message on topic {topic}: {e}")
                    # attempt to write original message timestamp-only
                    try:
                        outbag.write(topic, msg, new_t)
                    except Exception:
                        pass

                messages_written += 1

            # Update offset for next segment: last written timestamp + small gap
            last_new_ts = segment[-1][0] + time_shift
            current_time_offset = last_new_ts + gap_between_segments

    print(f"\nCompacted timeline:")
    print(f"  Original segments: {len(segments)}")
    print(f"  Messages written: {messages_written}")
    print(f"  Output bag: {output_bag_path}")


if __name__ == "__main__":
    input_bag = "/root/ros1_ws/april_whycode_test_bags/merged_topics.bag"
    output_bag = "/root/ros1_ws/april_whycode_test_bags/compacted_topics.bag"

    compact_timeline(input_bag, output_bag, gap_threshold_sec=10.0)
    