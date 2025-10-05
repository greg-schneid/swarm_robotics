# Brain System Launch File

This launch file starts the complete brain interface system for processing Muse EEG headset data.

## System Components

### 1. Brain Streamer (`brain_stream`)
- **Package**: `brain_streamer`
- **Function**: Connects to Muse EEG headset via Lab Streaming Layer (LSL)
- **Publishes**: 
  - `/brain/raw` (BrainData) - Raw EEG samples at 256 Hz
  - `/brain/raw_average` (Float32MultiArray) - 5-second averaged data

### 2. Brain Processor (`brain_processor`)
- **Package**: `brain_processor`
- **Function**: Processes EEG data to detect mental states
- **Subscribes**: `/brain/raw` (BrainData)
- **Publishes**: 
  - `/brain/action` (BrainActionMsg) - Mental state actions (STOP, IDLE, FORWARD, BOOST)
  - `/brain/metrics` (Float32MultiArray) - [beta/alpha ratio, alpha power, beta power]

## Usage

### Launch Both Nodes
```bash
# Source your workspace
source /workspaces/swarm_robotics/install/setup.bash

# Launch the brain system
ros2 launch brain_streamer brain_system.launch.py
```

### Launch Individual Nodes
If you need to launch nodes separately:

```bash
# Terminal 1: Brain Streamer
ros2 run brain_streamer brain_stream

# Terminal 2: Brain Processor
ros2 run brain_processor brain_processor
```

## Configuration Parameters

### Brain Streamer Parameters
- `process_frequency`: Processing frequency in Hz (default: 5)
- `mains_hz`: Mains frequency for notch filter - 60 Hz (US) or 50 Hz (EU/Asia)
- `summary_period`: Time period for summary statistics in seconds (default: 5.0)

### Brain Processor Parameters
- `sample_rate`: EEG sampling rate in Hz (default: 256.0)
- `buffer_sec`: Rolling buffer size in seconds for processing (default: 4.0)
- `mains_hz`: Mains frequency for notch filter (default: 60.0)

## Prerequisites

1. **Muse Headset** must be connected and streaming via `muselsl`
2. **Lab Streaming Layer (LSL)** installed
3. **ROS2 packages** built:
   ```bash
   colcon build --packages-select swarm_messages brain_streamer brain_processor
   ```

## Data Flow

```
Muse Headset (via Bluetooth)
    ↓
muselsl stream (LSL)
    ↓
brain_stream node → /brain/raw (BrainData)
    ↓
brain_processor node → /brain/action (BrainActionMsg)
                    → /brain/metrics (Float32MultiArray)
```

## Mental States

The brain processor classifies mental states based on Beta/Alpha ratios:
- **STOP** (Relaxed): Low Beta/Alpha ratio - eyes closed or very relaxed
- **IDLE**: Moderate ratio - normal resting state
- **FORWARD** (Focused): High ratio - active concentration
- **BOOST**: Very high ratio - intense focus

## Troubleshooting

### No LSL Stream Found
```bash
# Start muselsl manually
muselsl stream --backend bleak
```

### Check Running Nodes
```bash
ros2 node list
```

### Monitor Topics
```bash
# View raw brain data
ros2 topic echo /brain/raw

# View actions
ros2 topic echo /brain/action

# View metrics
ros2 topic echo /brain/metrics
```

### Check Node Info
```bash
ros2 node info /brain_stream
ros2 node info /brain_processor
```
