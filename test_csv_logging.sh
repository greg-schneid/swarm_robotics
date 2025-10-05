#!/bin/bash

# Test script for CSV logging
echo "======================================"
echo "Testing CSV Logging for Brain Streamer"
echo "======================================"
echo ""

# Source the ROS2 workspace
source /workspaces/swarm_robotics/install/setup.bash

# Clear old CSV files (optional)
echo "Removing old test CSV files..."
rm -f /workspaces/swarm_robotics/data/brain_data_*.csv

echo ""
echo "Starting brain_streamer with CSV logging enabled..."
echo "Will run for 5 seconds, then stop with Ctrl+C"
echo ""
echo "Watch for these log messages:"
echo "  - 'CSV logging parameter:' - shows what parameter value was received"
echo "  - 'CSV logging enabled. Saving to:' - confirms CSV file was created"
echo "  - 'CSV file closed successfully.' - confirms proper shutdown"
echo ""

# Run the launch file with timeout
timeout 5 ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true 2>&1 | tee /tmp/brain_test_output.log

echo ""
echo "======================================"
echo "Test completed. Checking results..."
echo "======================================"
echo ""

# Check for CSV files
echo "CSV files created:"
ls -lh /workspaces/swarm_robotics/data/brain_data_*.csv 2>/dev/null | tail -5

echo ""
echo "Checking most recent CSV file content:"
LATEST_CSV=$(ls -t /workspaces/swarm_robotics/data/brain_data_*.csv 2>/dev/null | head -1)
if [ -f "$LATEST_CSV" ]; then
    echo "File: $LATEST_CSV"
    echo "Size: $(stat -f%z "$LATEST_CSV" 2>/dev/null || stat -c%s "$LATEST_CSV") bytes"
    echo "Line count: $(wc -l < "$LATEST_CSV")"
    echo ""
    echo "First 5 lines:"
    head -5 "$LATEST_CSV"
    echo ""
    echo "Last 5 lines:"
    tail -5 "$LATEST_CSV"
else
    echo "No CSV file found!"
fi

echo ""
echo "======================================"
echo "Log excerpts (saved to /tmp/brain_test_output.log):"
echo "======================================"
grep -i "csv\|save_to" /tmp/brain_test_output.log || echo "No CSV-related logs found"
