# Manual USB Permissions Setup for OAK-D Pro on Ubuntu 24.04

This guide provides step-by-step commands to manually configure USB permissions for your OAK-D Pro camera on Ubuntu 24.04.

## Step 1: Check Current Device Status

First, verify your camera is detected:

```bash
lsusb | grep -i "03e7\|luxonis\|movidius"
```

You should see something like:
```
Bus 003 Device 030: ID 03e7:2485 Intel Movidius MyriadX
```

## Step 2: Check Your User Groups

Check if you're in the `plugdev` group:

```bash
groups
```

Look for `plugdev` in the output. If it's not there, you'll need to add yourself (Step 3).

## Step 3: Add Yourself to plugdev Group (if needed)

If you're NOT in the `plugdev` group, add yourself:

```bash
sudo usermod -a -G plugdev $USER
```

**Important:** After this command, you MUST log out and log back in (or restart) for the group change to take effect.

## Step 4: Remove Old Udev Rules (if they exist)

Remove any existing DepthAI udev rules to start fresh:

```bash
sudo rm -f /etc/udev/rules.d/80-depthai.rules
sudo rm -f /etc/udev/rules.d/80-movidius.rules
```

## Step 5: Create New Udev Rules File

Create a simple, working udev rules file for Ubuntu 24.04:

```bash
sudo tee /etc/udev/rules.d/80-depthai.rules > /dev/null << 'EOF'
# Luxonis / DepthAI (OAK) USB permissions for Ubuntu 24.04
# Unbooted (bootloader): 03e7:2485  "Movidius MyriadX"
# Booted (app running):  03e7:f63b  "Luxonis Device"

# Rule for unbooted device
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2485", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# Rule for booted device
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# Vendor-wide fallback rule (covers all 03e7 devices)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev", TAG+="uaccess"
EOF
```

## Step 6: Verify the Udev Rules File

Check that the file was created correctly:

```bash
cat /etc/udev/rules.d/80-depthai.rules
```

You should see the three rules above.

## Step 7: Set Correct Permissions on Udev Rules File

Ensure the file has the correct permissions:

```bash
sudo chmod 644 /etc/udev/rules.d/80-depthai.rules
```

## Step 8: Reload Udev Rules

Tell the system to reload the udev rules:

```bash
sudo udevadm control --reload-rules
```

## Step 9: Trigger Udev to Apply Rules

Apply the rules to currently connected devices:

```bash
sudo udevadm trigger
```

## Step 10: Unplug and Replug Your Camera

**IMPORTANT:** Physically unplug your OAK-D Pro camera from the USB port, wait 2-3 seconds, then plug it back in.

## Step 11: Verify Device Permissions

After replugging, check the device permissions:

```bash
lsusb | grep 03e7
```

Get the bus and device number (e.g., Bus 003 Device 030), then check permissions:

```bash
ls -la /dev/bus/usb/003/030
```

(Replace 003/030 with your actual bus/device numbers from lsusb)

You should see permissions like: `crw-rw-rw-` or `crw-rw----` with group `plugdev`

## Step 12: Test with DepthAI

Try running your device check script:

```bash
cd /home/jalaj/dimos/dimos/hardware/sensors/camera/depthai
source /home/jalaj/dimos/dimos/.venv-depthai/bin/activate
python depthai_device_check.py --no-show --frames 5
```

## Troubleshooting

### If Step 12 still fails:

1. **Verify you're in plugdev group:**
   ```bash
   groups | grep plugdev
   ```
   If not shown, you need to log out and log back in after Step 3.

2. **Check if device changed USB address:**
   ```bash
   lsusb | grep 03e7
   ```
   The device number may have changed after replugging.

3. **Try with sudo (diagnostic only):**
   ```bash
   sudo /home/jalaj/dimos/dimos/.venv-depthai/bin/python depthai_device_check.py --no-show --frames 5
   ```
   If this works, it confirms a permissions issue.

4. **Check udev rules are being applied:**
   ```bash
   udevadm info --query=all --name=/dev/bus/usb/003/030 | grep -E "MODE|GROUP|TAG"
   ```
   (Replace with your actual device path)

5. **Try a different USB 3.0 port** - Some ports may have different permissions.

## Alternative: Luxonis Official Simple Rule

If the above doesn't work, try the simplest rule recommended by Luxonis:

```bash
sudo tee /etc/udev/rules.d/80-depthai.rules > /dev/null << 'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the device.

## Notes

- Ubuntu 24.04 uses newer systemd/udev which may handle permissions differently than Ubuntu 20.04
- The `TAG+="uaccess"` is important for systemd-logind to grant access
- Group-based permissions (`GROUP="plugdev"`) provide an additional layer of access control
- Always unplug/replug after changing udev rules

