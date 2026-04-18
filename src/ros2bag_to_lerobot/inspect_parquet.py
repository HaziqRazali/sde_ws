import pandas as pd
from pathlib import Path

# Update this path to your generated parquet file
parquet_path = Path(
    "lerobot_dataset/_booster_t1_bag_ep4_2026-02-03_18-03-33/data/chunk-000/file-000.parquet"
)

if not parquet_path.exists():
    print(f"Error: File not found at {parquet_path}")
    exit(1)

print(f"--- Inspecting Parquet: {parquet_path} ---")

# Load data
df = pd.read_parquet(parquet_path)

# 1. Print Shape and Columns
print(f"\n[Shape]: {df.shape} (rows, columns)")
print("\n[Columns]:")
for col in df.columns:
    dtype = df[col].dtype
    # For object columns, check if they are arrays (common in LeRobot)
    sample = df[col].iloc[0]
    if isinstance(sample, (list, pd.Series, pd.Index)):
        print(f"  - {col}: Array/List (Length: {len(sample)})")
    else:
        print(f"  - {col}: {dtype}")

# 2. Show Head
print("\n[First 5 Rows]:")
print(df.head())

# 3. Check for specific LeRobot features
if "observation.state" in df.columns:
    print("\n[Observation State Sample]:")
    print(df["observation.state"].iloc[0])

if "action" in df.columns:
    print("\n[Action Sample]:")
    print(df["action"].iloc[0])

# 4. Check for is_done
if "is_done" in df.columns:
    print("\n[Is Done Status]:")
    # Show counts of True vs False
    done_counts = df["is_done"].value_counts()
    print(f"Counts:\n{done_counts}")
    
    # Show last few rows to see if it actually flips to 1.0 at the end
    print("\n[Last 5 'is_done' values]:")
    print(df["is_done"].tail().tolist())
    
    # Specifically check if the last frame is marked as done
    is_last_done = df["is_done"].iloc[-1]
    print(f"\nLast Frame Done? {bool(is_last_done)}")

