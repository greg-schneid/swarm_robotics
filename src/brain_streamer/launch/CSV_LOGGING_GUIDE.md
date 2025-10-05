# CSV Logging Feature for Brain Streamer

## Overview

The brain streamer now supports saving raw EEG data to CSV files. This feature is controlled by launch parameters and is disabled by default.

## Usage

### Basic Usage (CSV logging disabled)
```bash
ros2 launch brain_streamer brain_system.launch.py
```

### Enable CSV Logging
```bash
ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true
```

### Enable CSV Logging with Custom Output Directory
```bash
ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true csv_output_dir:=/path/to/output
```

## CSV File Format

### File Naming
Files are automatically named with timestamps: `brain_data_YYYYMMDD_HHMMSS.csv`

Example: `brain_data_20251005_143022.csv`

### CSV Structure
```csv
timestamp,TP9,AF7,AF8,TP10,AUX
1728139822.123,800.5,750.2,760.8,810.3,0.0
1728139822.127,802.1,751.0,761.5,811.2,0.0
...
```

**Columns:**
- `timestamp`: LSL timestamp (Unix time with microsecond precision)
- `TP9`: Left ear electrode (μV)
- `AF7`: Front-left electrode (μV)
- `AF8`: Front-right electrode (μV)
- `TP10`: Right ear electrode (μV)
- `AUX`: Auxiliary channel (μV)

## Implementation Details

### ROS2 Parameters
- `save_to_csv` (bool): Enable/disable CSV logging (default: `false`)
- `csv_output_dir` (string): Directory for CSV files (default: `/workspaces/swarm_robotics/data`)

### Features
- **Thread-safe writing**: Uses locks to ensure data integrity when writing from the streaming thread
- **Line buffering**: Data is flushed to disk after each line for safety
- **Automatic directory creation**: Output directory is created if it doesn't exist
- **Graceful shutdown**: CSV file is properly closed when the node stops
- **Error handling**: Logging errors don't crash the node

### Data Rate
The CSV logging captures data at the full Muse sampling rate (~256 Hz), resulting in approximately:
- **256 samples/second**
- **~15,360 samples/minute**
- **~921,600 samples/hour**

File sizes will be approximately **50-100 MB per hour** depending on precision.

## Storage Considerations

The default output directory `/workspaces/swarm_robotics/data` has a `.gitignore` file configured to exclude CSV files from version control.

Make sure you have sufficient disk space before long recording sessions!

## Example: Post-Processing

You can load and analyze the CSV data using Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load the data
df = pd.read_csv('/workspaces/swarm_robotics/data/brain_data_20251005_143022.csv')

# Plot electrode data
plt.figure(figsize=(12, 6))
for channel in ['TP9', 'AF7', 'AF8', 'TP10']:
    plt.plot(df['timestamp'], df[channel], label=channel, alpha=0.7)
plt.xlabel('Timestamp')
plt.ylabel('Voltage (μV)')
plt.legend()
plt.title('Raw EEG Data')
plt.show()
```
