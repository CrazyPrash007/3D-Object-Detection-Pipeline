#!/usr/bin/env python3

import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from concurrent.futures import ThreadPoolExecutor

# Global variable to track active markers
active_marker_ids = set()

def create_marker(distance, angle, marker_id, frame_id="os_sensor"):
    markers = MarkerArray()
    
    # Create a ROS Marker for the detected obstacle
    box_marker = Marker()
    box_marker.header.frame_id = frame_id
    box_marker.header.stamp = rospy.Time.now()
    box_marker.ns = "detection"
    box_marker.id = marker_id
    box_marker.type = Marker.SPHERE
    box_marker.action = Marker.ADD

    # Calculate position based on distance and angle
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)
    z = 0  # Since it's a 2D LiDAR, z-coordinate is 0

    # Set marker scale and position
    box_marker.scale.x = 0.2  # Size of the sphere marker
    box_marker.scale.y = 0.2
    box_marker.scale.z = 0.2
    box_marker.pose.position.x = x
    box_marker.pose.position.y = y
    box_marker.pose.position.z = z

    # Set marker color (red for detected obstacles)
    box_marker.color.a = 0.5  # Transparency
    box_marker.color.r = 1.0
    box_marker.color.g = 0.0
    box_marker.color.b = 0.0

    markers.markers.append(box_marker)
    
    # Create a text marker to display the distance
    text_marker = Marker()
    text_marker.header.frame_id = frame_id
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = "detection"
    text_marker.id = marker_id + 1000  # Ensure a unique ID for text marker
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD

    text_marker.pose.position.x = x
    text_marker.pose.position.y = y
    text_marker.pose.position.z = z + 0.5  # Position above the obstacle marker

    text_marker.scale.z = 0.5  # Font size
    text_marker.color.a = 1.0
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0

    text_marker.text = f"{distance:.2f} m"  # Distance text

    markers.markers.append(text_marker)

    return markers

def laser_scan_callback(msg):
    global active_marker_ids

    # Emergency brake flag, set to True by default
    emergency_brake = True
    
    current_marker_ids = set()
    markers = MarkerArray()

    # Loop over all the laser scan points
    for i, (range_value, angle) in enumerate(zip(msg.ranges, np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges)))):
        # If the point is valid (non-infinite distance) and within the 3-meter range
        if range_value > 0.0 and range_value <= 3.0:
            marker_id = i
            current_marker_ids.add(marker_id)

            # Create markers for the detected obstacle
            marker = create_marker(range_value, angle, marker_id)
            markers.markers.extend(marker.markers)

            # Emergency brake condition: obstacle within 1 meters
            if range_value <= 1.0:
                emergency_brake = False  # Set to False if an object is within 1 meters

    # Remove markers that are no longer active
    markers_to_remove = MarkerArray()
    for marker_id in active_marker_ids - current_marker_ids:
        remove_marker = Marker()
        remove_marker.header.frame_id = "os_sensor"
        remove_marker.header.stamp = rospy.Time.now()
        remove_marker.ns = "detection"
        remove_marker.id = marker_id
        remove_marker.type = Marker.SPHERE
        remove_marker.action = Marker.DELETE

        remove_text_marker = Marker()
        remove_text_marker.header.frame_id = "os_sensor"
        remove_text_marker.header.stamp = rospy.Time.now()
        remove_text_marker.ns = "detection"
        remove_text_marker.id = marker_id + 1000  # Text marker ID
        remove_text_marker.type = Marker.TEXT_VIEW_FACING
        remove_text_marker.action = Marker.DELETE

        markers_to_remove.markers.append(remove_marker)
        markers_to_remove.markers.append(remove_text_marker)

    # Publish the markers
    marker_pub.publish(markers_to_remove)
    marker_pub.publish(markers)

    # Publish emergency braking status
    braking_pub.publish(Bool(data=emergency_brake))

    # Update active marker IDs
    active_marker_ids = current_marker_ids

if __name__ == "__main__":
    rospy.init_node('detection_node')

    # Publishers for markers and emergency braking status
    marker_pub = rospy.Publisher('/detection/markers', MarkerArray, queue_size=10)
    braking_pub = rospy.Publisher('/braking', Bool, queue_size=10)

    # Subscriber for the 2D laser scan data
    rospy.Subscriber('/spur/laser/scan', LaserScan, laser_scan_callback, queue_size=10)

    rospy.spin()

