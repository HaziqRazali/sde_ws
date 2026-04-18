import argparse
from pathlib import Path
import numpy as np
import cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

def inspect_bag(bag_path):
    print(f"Inspecting bag: {bag_path}")
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        # Find image topics
        image_topics = []
        for conn in reader.connections:
            if 'image' in conn.topic.lower() or 'camera' in conn.topic.lower():
                if conn.topic not in image_topics:
                    image_topics.append(conn.topic)
        
        print(f"Found image-related topics: {image_topics}")

        seen_topics = set()
        for conn, ts, rawdata in reader.messages(connections=[c for c in reader.connections if c.topic in image_topics]):
            if conn.topic in seen_topics:
                continue
            
            print(f"\n--- Topic: {conn.topic} ---")
            print(f"Type: {conn.msgtype}")
            
            try:
                msg = reader.deserialize(rawdata, conn.msgtype)
                
                # Check for CompressedImage
                if hasattr(msg, 'format'):
                    print(f"Format String: '{msg.format}'")
                    data_len = len(msg.data)
                    print(f"Data Length: {data_len} bytes")
                    
                    # Heuristic check for PNG/JPEG headers
                    # msg.data is uint8 numpy array in rosbags
                    if data_len > 4:
                        # Convert to bytes for hex check
                        header_bytes = msg.data[:4].tobytes()
                        print(f"Header (hex): {header_bytes.hex()}")
                        if header_bytes.hex().startswith('ffd8'):
                            print("Likely JPEG")
                        elif header_bytes.hex().startswith('89504e47'):
                            print("Likely PNG")
                        else:
                            print("Unknown header")

                    # Decoding Attempts
                    np_arr = msg.data
                    
                    # 1. Standard decode
                    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                    if img is not None:
                        print(f"Standard Decode Success: Shape={img.shape}, Dtype={img.dtype}")
                        if len(img.shape) == 3:
                            print(f"Channels: {img.shape[2]}")
                        elif len(img.shape) == 2:
                            print("Channels: 1 (Grayscale)")
                            print(f"Min/Max values: {img.min()}/{img.max()}")
                    else:
                        print("Standard Decode Failed")

                    # 2. Skip header decode (Common for compressedDepth)
                    # compressedDepth has a 12-byte header: 
                    # [format_enum(4)][depth_quantization(4)][depth_max(4)]... then raw data
                    if "compressedDepth" in conn.topic or "compressedDepth" in msg.format:
                        print("Attempting to skip 12-byte header...")
                        if data_len > 12:
                            np_arr_skip = msg.data[12:]
                            img_skip = cv2.imdecode(np_arr_skip, cv2.IMREAD_UNCHANGED)
                            if img_skip is not None:
                                print(f"Header Skip Decode Success: Shape={img_skip.shape}, Dtype={img_skip.dtype}")
                                print(f"Min: {img_skip.min()}, Max: {img_skip.max()}")
                            else:
                                print("Header Skip Decode Failed")
                
                # Check for Raw Image
                elif hasattr(msg, 'encoding'):
                    print(f"Encoding: {msg.encoding}")
                    print(f"Width: {msg.width}, Height: {msg.height}")
                    print(f"Step: {msg.step}")
            
            except Exception as e:
                print(f"Error inspecting message: {e}")
            
            seen_topics.add(conn.topic)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inspect ROS bag image topics")
    parser.add_argument("bag_path", help="Path to the ROS bag directory")
    args = parser.parse_args()
    
    inspect_bag(args.bag_path)
