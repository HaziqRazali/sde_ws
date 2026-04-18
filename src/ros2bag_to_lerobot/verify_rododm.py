import robodm
import numpy as np
import os
import sys

# Path to your generated VLA file
VLA_PATH = "./robodm_output/booster_stream.vla"

print(f"🔍 Inspecting {VLA_PATH}...\n")

if not os.path.exists(VLA_PATH):
    print(f"❌ File not found: {VLA_PATH}")
    sys.exit(1)

try:
    # 1. Open Trajectory in Read Mode
    traj = robodm.Trajectory(path=VLA_PATH, mode="r")
    
    # 2. Get Metadata via Backend (Memory Safe)
    print("--- Stream Metadata (extracted from backend) ---")
    streams = traj.backend.get_streams()
    
    print(f"{'Feature Name':<40} | {'Encoding':<15} | {'Type Hint'}")
    print("-" * 80)
    for s in streams:
        print(f"{s.feature_name:<40} | {s.encoding:<15} | {s.feature_type}")
        
    print("-" * 80)
    print(f"Total Streams: {len(streams)}")

    # 3. Load FIRST 20 FRAMES to verify data integrity (Memory Safe)
    print("\n--- Inspecting First 20 Frames (Data Integrity Check) ---")
    
    try:
        # CHANGED: slice(0, 20) loads the first 20 frames
        frame_data = traj.load(data_slice=slice(0, 20))
        
        print(f"{'Key':<40} | {'Shape':<20} | {'Dtype'}")
        print("-" * 80)
        
        all_good = True
        for key, val in frame_data.items():
            # Handle cases where data might be a list or numpy array
            if hasattr(val, 'shape'):
                shape_str = str(val.shape)
                dtype_str = str(val.dtype)
                # Check if we actually got 20 frames
                if val.shape[0] != 20:
                    shape_str += " ⚠️ LENGTH MISMATCH"
                    all_good = False
            elif isinstance(val, list):
                shape_str = f"list(len={len(val)})"
                dtype_str = type(val[0]).__name__ if val else "empty"
                if len(val) != 20:
                    shape_str += " ⚠️ LENGTH MISMATCH"
                    all_good = False
            else:
                shape_str = "scalar"
                dtype_str = type(val).__name__
                
            print(f"{key:<40} | {shape_str:<20} | {dtype_str}")
            
        print("-" * 80)
        if all_good:
            print("✅ First 20 frames loaded successfully. Synchronization looks correct.")
        else:
            print("⚠️  WARNING: Some arrays do not have exactly 20 frames.")
        
    except Exception as e:
        print(f"❌ Failed to load data slice: {e}")

    traj.close()

except Exception as e:
    print(f"❌ Critical Error opening file: {e}")
    import traceback
    traceback.print_exc()