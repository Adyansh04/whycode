#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseArray
import numpy as np
from collections import defaultdict, deque
import time

class WhyCodeIDVisualizer:
    def __init__(self):
        rospy.init_node('whycode_id_visualizer', anonymous=True)
        
        # Configuration
        self.history_duration = 100.0  # seconds (100 seconds history)
        self.active_threshold = 3.0    # seconds (active if detected in last 3 seconds)
        self.min_id_value = -1  # Minimum WhyCode ID (invalid/undetected)
        self.max_id_value = 10  # Maximum WhyCode ID
        
        # Data storage
        self.detection_history = defaultdict(lambda: deque())  # ID -> timestamps
        self.detection_counts = defaultdict(int)  # ID -> total count
        self.last_detection_time = defaultdict(float)  # ID -> last seen time
        self.detection_gaps = defaultdict(lambda: deque())  # ID -> time gaps between detections
        
        # Visualization setup
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1, figsize=(12, 12))
        self.fig.suptitle('WhyCode ID Detection Analysis', fontsize=16)
        
        # Subscribe to pose array topic
        self.pose_sub = rospy.Subscriber('/whycon/poses', PoseArray, self.pose_callback)
        
        # Get subscriber rate for refresh rate
        self.update_interval = 100  # 10Hz refresh rate (adjust as needed)
        
        # Start visualization
        self.ani = animation.FuncAnimation(self.fig, self.update_plots, interval=self.update_interval, blit=False)
        plt.tight_layout()
        plt.show()
    
    def pose_callback(self, msg):
        current_time = time.time()
        
        # Extract ALL IDs from z-coordinates (including -1 for invalid)
        detected_ids = []
        for pose in msg.poses:
            id_val = int(pose.position.z)
            # Include all IDs in the valid range (-1 to 10)
            if self.min_id_value <= id_val <= self.max_id_value:
                detected_ids.append(id_val)
        
        for id_val in detected_ids:
            # Calculate time gap if this ID was detected before
            if id_val in self.last_detection_time:
                gap = current_time - self.last_detection_time[id_val]
                self.detection_gaps[id_val].append(gap)
                
                # Keep only recent gaps (last 50 gaps for analysis)
                if len(self.detection_gaps[id_val]) > 50:
                    self.detection_gaps[id_val].popleft()
            
            # Update detection records
            self.detection_history[id_val].append(current_time)
            self.detection_counts[id_val] += 1
            self.last_detection_time[id_val] = current_time
            
            # Clean old history (keep 100 seconds)
            while (self.detection_history[id_val] and 
                   current_time - self.detection_history[id_val][0] > self.history_duration):
                self.detection_history[id_val].popleft()
    
    def update_plots(self, frame):
        current_time = time.time()
        
        # Clear all axes
        self.ax1.clear()
        self.ax2.clear() 
        self.ax3.clear()
        self.ax4.clear()
        
        # Plot 1: Detection Timeline (last 100 seconds)
        self.plot_timeline(current_time)
        
        # Plot 2: Detection Counts per ID (bar chart)
        self.plot_detection_counts()
        
        # Plot 3: Currently Active IDs (detected in last 3 seconds)
        self.plot_active_ids(current_time)
        
        # Plot 4: Time Gaps Between Detections
        self.plot_time_gaps()
        
        plt.tight_layout()
    
    def plot_timeline(self, current_time):
        """Timeline showing when each ID was detected over last 100 seconds"""
        self.ax1.set_title('Detection Timeline (Last 100 seconds)')
        self.ax1.set_xlabel('Time (seconds ago)')
        self.ax1.set_ylabel('WhyCode ID')
        
        # Create color map for all possible IDs (-1 to 10)
        id_range = self.max_id_value - self.min_id_value + 1
        colors = plt.cm.tab10(np.linspace(0, 1, id_range))
        
        for id_val, timestamps in self.detection_history.items():
            if timestamps:
                time_offsets = [current_time - t for t in timestamps]
                y_positions = [id_val] * len(time_offsets)
                
                # Map ID to color index (shift by min_id_value to handle negative IDs)
                color_idx = (id_val - self.min_id_value) % len(colors)
                
                # Special handling for ID -1 (invalid detections)
                if id_val == -1:
                    line_style = 'x--'  # Different style for invalid IDs
                    alpha = 0.5
                    label = 'ID -1 (Invalid)'
                else:
                    line_style = 'o-'
                    alpha = 0.7
                    label = f'ID {id_val}'
                
                # Use lines instead of just scatter points for better visibility
                if len(time_offsets) > 1:
                    self.ax1.plot(time_offsets, y_positions, line_style, 
                                 color=colors[color_idx], 
                                 alpha=alpha, markersize=3, linewidth=1, 
                                 label=label)
                else:
                    self.ax1.scatter(time_offsets, y_positions, 
                                   c=[colors[color_idx]], 
                                   alpha=alpha, s=30, label=label,
                                   marker='x' if id_val == -1 else 'o')
        
        self.ax1.set_xlim(0, self.history_duration)
        self.ax1.set_ylim(self.min_id_value - 0.5, self.max_id_value + 0.5)
        if self.detection_history:
            self.ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.invert_xaxis()  # Most recent on the left
    
    def plot_detection_counts(self):
        """Bar chart showing total detection counts per ID"""
        self.ax2.set_title('Total Detection Counts per ID')
        self.ax2.set_xlabel('WhyCode ID')
        self.ax2.set_ylabel('Total Detections')
        
        if self.detection_counts:
            ids = list(self.detection_counts.keys())
            counts = list(self.detection_counts.values())
            
            # Color coding with special handling for ID -1
            colors = []
            for id_val, count in zip(ids, counts):
                if id_val == -1:
                    colors.append('red')  # Always red for invalid IDs
                elif count > 100:
                    colors.append('green')
                elif count > 50:
                    colors.append('orange')
                else:
                    colors.append('yellow')
            
            bars = self.ax2.bar(ids, counts, color=colors, alpha=0.7)
            
            # Add count values on bars
            for bar, count in zip(bars, counts):
                height = bar.get_height()
                self.ax2.text(bar.get_x() + bar.get_width()/2., height + max(counts)*0.01,
                            f'{count}', ha='center', va='bottom', fontsize=9)
            
            self.ax2.set_ylim(0, max(counts) * 1.1)
            self.ax2.set_xlim(self.min_id_value - 0.5, self.max_id_value + 0.5)
        
        self.ax2.grid(True, alpha=0.3)
    
    def plot_active_ids(self, current_time):
        """Show currently active IDs (detected in last 3 seconds)"""
        self.ax3.set_title('Currently Active IDs (Detected in Last 3 Seconds)')
        self.ax3.set_xlabel('WhyCode ID')
        self.ax3.set_ylabel('Seconds Since Last Detection')
        
        active_ids = []
        time_since_last = []
        
        for id_val, last_time in self.last_detection_time.items():
            gap = current_time - last_time
            if gap <= self.active_threshold:  # Active if detected in last 3 seconds
                active_ids.append(id_val)
                time_since_last.append(gap)
        
        if active_ids:
            # Color coding with special handling for ID -1
            colors = []
            for id_val, gap in zip(active_ids, time_since_last):
                if id_val == -1:
                    colors.append('red')  # Red for invalid IDs
                elif gap < 1.0:
                    colors.append('green')  # Very recent
                else:
                    colors.append('yellow')  # Recent
            
            bars = self.ax3.bar(active_ids, time_since_last, color=colors, alpha=0.7)
            
            # Add time values on bars
            for bar, gap in zip(bars, time_since_last):
                height = bar.get_height()
                self.ax3.text(bar.get_x() + bar.get_width()/2., height + 0.05,
                            f'{gap:.1f}s', ha='center', va='bottom', fontsize=9)
            
            self.ax3.set_ylim(0, self.active_threshold * 1.1)
            self.ax3.set_xlim(self.min_id_value - 0.5, self.max_id_value + 0.5)
            
            # Count valid vs invalid active IDs
            valid_active = len([id_val for id_val in active_ids if id_val > 0])
            invalid_active = len([id_val for id_val in active_ids if id_val == -1])
            
            # Add status text
            status_text = f'Active IDs: {len(active_ids)} (Valid: {valid_active}, Invalid: {invalid_active})'
            self.ax3.text(0.02, 0.95, status_text, 
                         transform=self.ax3.transAxes, fontsize=10, 
                         bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"))
        else:
            # No active IDs
            self.ax3.text(0.5, 0.5, 'No Active IDs', transform=self.ax3.transAxes, 
                         ha='center', va='center', fontsize=14,
                         bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral"))
            self.ax3.set_ylim(0, 1)
            self.ax3.set_xlim(self.min_id_value - 0.5, self.max_id_value + 0.5)
        
        self.ax3.grid(True, alpha=0.3)
    
    def plot_time_gaps(self):
        """Show time gaps between consecutive detections for each ID"""
        self.ax4.set_title('Time Gaps Between Consecutive Detections')
        self.ax4.set_xlabel('WhyCode ID')
        self.ax4.set_ylabel('Average Gap (seconds)')
        
        ids = []
        avg_gaps = []
        colors_list = []
        
        for id_val, gaps in self.detection_gaps.items():
            if gaps and len(gaps) > 1:  # Need at least 2 detections to calculate gaps
                avg_gap = np.mean(list(gaps))
                ids.append(id_val)
                avg_gaps.append(avg_gap)
                
                # Color coding with special handling for ID -1
                if id_val == -1:
                    colors_list.append('red')      # Red for invalid IDs
                elif avg_gap < 0.5:
                    colors_list.append('green')    # Very consistent
                elif avg_gap < 1.0:
                    colors_list.append('yellow')   # Consistent
                elif avg_gap < 2.0:
                    colors_list.append('orange')   # Moderate
                else:
                    colors_list.append('red')      # Inconsistent
        
        if ids:
            bars = self.ax4.bar(ids, avg_gaps, color=colors_list, alpha=0.7)
            
            # Add gap values on bars
            for bar, gap in zip(bars, avg_gaps):
                height = bar.get_height()
                self.ax4.text(bar.get_x() + bar.get_width()/2., height + max(avg_gaps)*0.02,
                            f'{gap:.2f}s', ha='center', va='bottom', fontsize=9)
            
            self.ax4.set_ylim(0, max(avg_gaps) * 1.15)
            self.ax4.set_xlim(self.min_id_value - 0.5, self.max_id_value + 0.5)
        
        self.ax4.grid(True, alpha=0.3)

if __name__ == '__main__':
    try:
        visualizer = WhyCodeIDVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass