import os
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# === Paths ===
bag_path = os.path.expanduser("~/10x/rosbags/depth")
output_folder = os.path.expanduser("~/10x/depth_images")
os.makedirs(output_folder, exist_ok=True)

# === Setup ===
bridge = CvBridge()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# === Get topic info ===
topic_types = reader.get_all_topics_and_types()
topic_name = topic_types[0].name
topic_type = topic_types[0].type
msg_type = get_message(topic_type)

# === Extract messages ===
count = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    msg = deserialize_message(data, msg_type)
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    filename = os.path.join(output_folder, f"depth_{count:03d}.png")
    cv2.imwrite(filename, cv_image)
    print(f"Saved {filename}")
    count += 1

print(f"\nAll done! Extracted {count} depth frames to '{output_folder}/'")
