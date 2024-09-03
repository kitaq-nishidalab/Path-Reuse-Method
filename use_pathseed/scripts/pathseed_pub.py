#!/usr/bin/env python3

import rospy
from pathseeds_encoder.msg import PathSeed
import numpy as np
import os

def read_data_from_file(file_path):
    """Read data from a file and convert it to a NumPy array."""
    data = []
    if not os.path.isfile(file_path):
        rospy.logerr("File not found: '%s'", file_path)
        return np.array(data)  # Return empty array if file not found

    try:
        with open(file_path, 'r') as file:
            for line in file:
                line_data = line.strip().rstrip(',').rstrip(';').split(',')
                for value in line_data:
                    value = value.strip()
                    if value:
                        try:
                            data.append(float(value))
                        except ValueError:
                            rospy.logwarn("Could not convert value to float: '%s'", value)
    except IOError as e:
        rospy.logerr("Failed to read file: '%s'. Error: %s", file_path, e)
    return np.array(data)

def publisher():
    """Continuously publish PathSeed messages."""
    pub = rospy.Publisher('move_group/path_seed', PathSeed, queue_size=10)
    rospy.init_node('matrix_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    count = 0

    # Initialize file path and read initial data
    filename = rospy.get_param('/pathseed_file', '/home/nishidalab/train_ws/src/pathseeds_encoder/pathseeds/pathseed1.txt')
    data = read_data_from_file(filename)
    num_cols = 6
    if len(data) % num_cols != 0:
        rospy.logerr("Initial data length is not divisible by the number of columns")
        return
    num_rows = len(data) // num_cols

    rospy.loginfo("Publishing PathSeed message continuously...")
    while not rospy.is_shutdown():
        # Check if the file path has changed
        new_filename = rospy.get_param('/pathseed_file', filename)
        print(new_filename)
        if new_filename != filename:
            filename = new_filename
            data = read_data_from_file(filename)
            num_cols = 6
            if len(data) % num_cols != 0:
                rospy.logerr("Data length is not divisible by the number of columns after file change")
                continue  # Skip the current iteration if data is invalid
            num_rows = len(data) // num_cols
        
        rospy.loginfo("Publishing PathSeed message #%d", count)
        matrix_msg = PathSeed()
        matrix_msg.rows = num_rows
        matrix_msg.cols = num_cols
        matrix_msg.data = data.tolist()

        rospy.loginfo("  Rows: %d", matrix_msg.rows)
        rospy.loginfo("  Cols: %d", matrix_msg.cols)
        rospy.loginfo("  First row of data: %s", str(data[:num_cols]))
        rospy.loginfo("  Total number of data elements: %d", len(data))
        count += 1

        pub.publish(matrix_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
