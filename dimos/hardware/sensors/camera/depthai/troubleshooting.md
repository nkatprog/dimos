# DepthAI/OAK-D Pro Troubleshooting Guide

## Common Issues and Solutions

### Issue: Device Crashes During Operation

**Symptoms:**
- Initial "Couldn't open stream" error
- Partial frames received, then communication errors
- "Device likely crashed but did not reboot in time"
- Resource deadlock errors

**Causes:**
1. Device needs recovery time after a crash
2. USB power/bandwidth issues
3. Device overheating
4. Multiple processes trying to access device
5. USB cable/port problems

**Solutions:**

1. **Wait for Device Recovery**
   - After a crash, wait 5-10 seconds before restarting
   - The updated script now includes automatic recovery checks

2. **Check for Other Processes**
   ```bash
   lsof | grep -i depthai
   ps aux | grep depthai
   ```
   Kill any other processes using the device

3. **Unplug and Replug**
   - Physically disconnect the USB cable
   - Wait 5 seconds
   - Reconnect to a USB 3.0 port (not USB 2.0)

4. **Use Direct USB Port**
   - Avoid USB hubs
   - Use a USB 3.0 port directly on the motherboard
   - Try a different USB port

5. **Check USB Power**
   - Ensure USB port provides adequate power
   - Some laptops have low-power USB ports
   - Try a powered USB hub if needed

6. **Reduce Load**
   - Lower FPS: `--fps 15` instead of 30
   - Lower resolution: `--rgb-resolution 720p` instead of 1080p
   - Disable depth if not needed: `--no-show-depth`

### Issue: "Couldn't open stream" on First Run

**Solution:**
- The script now checks device availability before starting
- If device is in error state, it will wait for recovery
- Make sure udev rules are properly installed (see setup script)

### Issue: Works After Restart But Not Initially

**Solution:**
- Device needs time to fully boot and stabilize
- The updated script adds a 0.5 second delay after device check
- If issues persist, increase the delay in the script

### Issue: Qt/Wayland Plugin Warning

**Symptoms:**
```
qt.qpa.plugin: Could not find the Qt platform plugin "wayland"
```

**Solution:**
- This is a harmless warning from OpenCV
- It will fall back to X11 automatically
- To suppress, set environment variable:
  ```bash
  export QT_QPA_PLATFORM=xcb
  ```

### Best Practices

1. **Always Clean Exit**
   - Use 'q' or ESC to quit (don't Ctrl+C if possible)
   - The script now has better cleanup on exit

2. **Wait Between Runs**
   - If you need to restart, wait 3-5 seconds
   - This gives the device time to fully release resources

3. **Monitor Device Temperature**
   - OAK-D Pro can overheat during extended use
   - If crashes become frequent, let device cool down

4. **Check USB Connection**
   - Use high-quality USB 3.0 cable
   - Ensure cable is fully seated
   - Avoid long cables (>2m)

5. **System Resources**
   - Close other applications using USB devices
   - Ensure adequate system memory
   - Check `dmesg` for USB errors:
     ```bash
     dmesg | tail -20 | grep -i usb
     ```

## Updated Script Features

The `depthai_camera_test_script.py` has been updated with:

1. **Device Availability Check** - Checks if device is ready before starting
2. **Automatic Recovery** - Waits for device to recover if in bad state
3. **Better Error Handling** - Catches and handles device crashes gracefully
4. **Improved Cleanup** - Ensures device is properly released on exit
5. **Depth Processing Safety** - Handles errors during depth frame processing

## Running the Script

```bash
# Basic usage
python depthai_camera_test_script.py

# With reduced load (more stable)
python depthai_camera_test_script.py --fps 15 --rgb-resolution 720p

# RGB only (no depth processing)
python depthai_camera_test_script.py --no-show-depth

# With IR enabled (OAK-D Pro only)
python depthai_camera_test_script.py --enable-ir --ir-dot-ma 200 --ir-flood-ma 200
```

## If Problems Persist

1. Check device firmware version
2. Update DepthAI library: `pip install --upgrade depthai`
3. Try with sudo (diagnostic only): `sudo python depthai_camera_test_script.py`
4. Check system logs: `journalctl -k | grep -i usb`
5. Contact Luxonis support with device logs


