#!/usr/bin/env python3

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import sys

def republish_bag_with_suffix(input_bag, output_bag, suffix="_2"):
    """
    Copies all topics from a ROS2 bag to a new bag with topics renamed with a suffix
    
    Args:
        input_bag: Path to input rosbag2 directory
        output_bag: Path to output rosbag2 directory
        suffix: Suffix to append to topic names (default: "_2")
    """
    
    try:
        # Get storage options and converter options
        storage_options_read = rosbag2_py.StorageOptions(uri=input_bag, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        # Open input bag
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options_read, converter_options)
        
        # Get storage options for output bag
        storage_options_write = rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3')
        
        # Open output bag for writing
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options_write, converter_options)
        
        # Get all topics from input bag
        topics_info = reader.get_all_topics_and_types()
        print(f"Found {len(topics_info)} topics in bag:")
        
        topic_mapping = {}
        for topic_info in topics_info:
            old_topic = topic_info.name
            new_topic = old_topic + suffix
            topic_mapping[old_topic] = (new_topic, topic_info.type)
            print(f"  {old_topic} -> {new_topic}")
        
        # Create topic metadata for output bag
        for old_topic, (new_topic, msg_type) in topic_mapping.items():
            topic_info = rosbag2_py.TopicMetadata(
                name=new_topic,
                type=msg_type,
                serialization_format='cdr'
            )
            writer.create_topic(topic_info)
        
        print("\nCopying messages...")
        message_count = 0
        
        # Read and write messages with new topic names
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            if topic in topic_mapping:
                new_topic, _ = topic_mapping[topic]
                writer.write(new_topic, data, timestamp)
                message_count += 1
                
                if message_count % 1000 == 0:
                    print(f"Copied {message_count} messages...")
        
        print(f"\nFinished! Copied {message_count} total messages to {output_bag}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python republish_bag_ros2.py <input_bag> <output_bag> [suffix]")
        print("Example: python republish_bag_ros2.py ./smsp_walk_2 ./smsp_walk_2_renamed _2")
        sys.exit(1)
    
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    suffix = sys.argv[3] if len(sys.argv) > 3 else "_2"
    
    republish_bag_with_suffix(input_bag, output_bag, suffix)