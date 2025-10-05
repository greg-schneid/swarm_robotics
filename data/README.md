# Brain Data CSV Logs

This directory contains CSV files with raw brain data captured from the Muse EEG headset.

## File Format

Each CSV file is named with a timestamp: `brain_data_YYYYMMDD_HHMMSS.csv`

### Columns

- `timestamp`: LSL timestamp when the sample was captured
- `TP9`: Left ear electrode reading (μV)
- `AF7`: Front-left electrode reading (μV)
- `AF8`: Front-right electrode reading (μV)
- `TP10`: Right ear electrode reading (μV)
- `AUX`: Auxiliary channel reading (μV)

## Enabling CSV Logging

CSV logging is disabled by default. To enable it when launching:

```bash
# Enable CSV logging with default output directory
ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true

# Enable CSV logging with custom output directory
ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true csv_output_dir:=/path/to/output
```

## Sample Rate

Data is sampled at approximately 256 Hz (samples per second).
