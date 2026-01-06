#!/usr/bin/env bash
# Quick script to kill all DepthAI camera processes that might be holding the device

echo "Finding DepthAI camera processes..."
PIDS=$(pgrep -f "depthai_camera_test_script.py" 2>/dev/null)

if [ -z "$PIDS" ]; then
    echo "No DepthAI camera processes found."
    exit 0
fi

echo "Found processes: $PIDS"
echo "Killing processes..."

for PID in $PIDS; do
    echo "  Killing PID $PID..."
    kill -15 "$PID" 2>/dev/null || true
done

sleep 2

# Check if any are still running and force kill
REMAINING=$(pgrep -f "depthai_camera_test_script.py" 2>/dev/null)
if [ -n "$REMAINING" ]; then
    echo "Some processes didn't exit, force killing..."
    for PID in $REMAINING; do
        echo "  Force killing PID $PID..."
        kill -9 "$PID" 2>/dev/null || true
    done
    sleep 1
fi

# Final check
FINAL=$(pgrep -f "depthai_camera_test_script.py" 2>/dev/null)
if [ -z "$FINAL" ]; then
    echo "✓ All DepthAI camera processes killed."
else
    echo "⚠ Warning: Some processes may still be running: $FINAL"
fi

