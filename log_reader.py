#!/usr/bin/env python3
"""A simple script to convert ROS2 log timestamps to readable format."""

import datetime
import sys


def convert_timestamp_to_readable(timestamp):
    """Convert a UNIX timestamp to a readable datetime string."""
    dt_object = datetime.datetime.fromtimestamp(timestamp)
    return dt_object.strftime("%Y-%m-%d %H:%M:%S.%f")


def process_log_file(log_file_path):
    """Process the ROS2 log file, converting timestamps to readable format."""
    with open(log_file_path, "r") as file:
        for line in file:
            # Split the line to extract the timestamp
            parts = line.split(" ")
            if len(parts) > 0:
                try:
                    timestamp = float(parts[0])
                    readable_time = convert_timestamp_to_readable(timestamp)
                    # Replace the original timestamp with the readable time
                    parts[0] = readable_time
                    # Print the modified line without the newline character
                    print(" ".join(parts).strip())
                # Value error & Broken pipe error handling
                except ValueError:
                    # If the first part is not a timestamp, print the line as is
                    print(line.strip())
                except BrokenPipeError:
                    # If the pipe is broken, exit gracefully
                    return


if __name__ == "__main__":
    # 从命令行参数读取日志文件路径

    if len(sys.argv) < 2:
        print("Usage: python log_reader.py <log_file_path>")
        sys.exit(1)

    log_file_path = sys.argv[1]

    process_log_file(log_file_path)
