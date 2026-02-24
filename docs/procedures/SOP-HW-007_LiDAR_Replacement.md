# SOP: Livox Mid-360 LiDAR Replacement

**Document ID:** SOP-HW-007
**Version:** 1.0
**Category:** Hardware
**Estimated Duration:** 45 minutes

## Prerequisites
- Replacement Livox Mid-360 unit (P/N: LIV-MID360-01)
- 3m Ethernet cable (P/N: CBL-ETH-3M)
- Mounting bracket (P/N: MNT-LIDAR-360)
- Laptop with Livox Viewer 2 installed

## Safety Notes
- Power off the robot before disconnecting the LiDAR
- Wear ESD wrist strap when handling the sensor
- Do not look directly into the laser aperture
- Class 1 laser - eye safe under normal operation

## Tools Required
- Phillips screwdriver (PH2)
- 3mm hex key
- ESD wrist strap
- Multimeter (for voltage verification)
- Cable ties

## PPE Required
- Safety glasses
- ESD wrist strap

## Steps

### Step 1: Power Down Robot
**Type:** Instruction | **Critical:** Yes
Initiate a safe shutdown of the robot. Wait for all motors to de-energize and status LEDs to turn off. Disconnect the main power supply.

### Step 2: Document Failed Unit
**Type:** Photo | **Critical:** No
Take photos of the failed LiDAR unit showing:
- Current mounting position and cable routing
- LED status (if any)
- Any visible damage
- Serial number label

### Step 3: Run Pre-Removal Diagnostic
**Type:** Script | **Critical:** No
**Embedded Script:** LiDAR Network Diagnostic
Run the diagnostic script to capture the current state before removal. This documents the failure for RMA.

```bash
# Ping LiDAR
ping -c 3 192.168.1.181

# Check if LiDAR node is running
ros2 node list | grep livox

# Check point cloud topic
ros2 topic hz /livox/lidar --window 5
```

### Step 4: Disconnect Cables
**Type:** Instruction | **Critical:** Yes
1. Disconnect the Ethernet cable from the LiDAR
2. Disconnect the power cable (9-27V DC)
3. Label cables for reconnection
4. Verify zero voltage on power connector with multimeter

### Step 5: Remove Failed Unit
**Type:** Instruction | **Critical:** No
1. Remove the 4x M3 mounting screws using 3mm hex key
2. Carefully lift the LiDAR unit from the mounting bracket
3. Place in anti-static bag for RMA
4. Inspect mounting bracket for damage

### Step 6: Install New Unit
**Type:** Instruction | **Critical:** Yes
1. Verify new unit serial number matches work order
2. Align LiDAR unit on mounting bracket (arrow points forward)
3. Install 4x M3 mounting screws - torque to 0.5 Nm
4. Verify unit is secure with no play

### Step 7: Reconnect Cables
**Type:** Instruction | **Critical:** Yes
1. Connect Ethernet cable to LiDAR port
2. Connect power cable (verify 12V before connecting)
3. Route cables with cable ties matching original routing
4. Verify no cable strain on connectors

### Step 8: Power On and Verify LED
**Type:** Instruction | **Critical:** Yes
Power on the robot. The LiDAR LED should show:
- **Solid Red** → Powering on (5-10 seconds)
- **Blinking Green** → Normal operation, awaiting connection
- **Solid Green** → Connected and streaming data

If LED shows solid red for >30 seconds, check power connections.

### Step 9: Configure Network
**Type:** Script | **Critical:** Yes
**Embedded Script:** LiDAR Network Configuration
Configure the new LiDAR unit's IP address to match the robot's network:

```bash
# Verify LiDAR is on default IP
ping -c 3 192.168.1.1

# Update MID360 config with correct IP
cat /opt/livox/config/MID360_config.json

# Restart Livox driver
sudo systemctl restart livox_ros2_driver
```

### Step 10: Run Post-Install Diagnostic
**Type:** Script | **Critical:** Yes
**Embedded Script:** LiDAR Verification Script
Verify the new unit is working correctly:

```bash
# Verify LiDAR responds on configured IP
ping -c 3 192.168.1.181

# Check ROS2 node is running
ros2 node list | grep livox

# Verify point cloud data
ros2 topic hz /livox/lidar --window 10

# Check point cloud quality (should be >10k points/frame)
ros2 topic echo /livox/lidar --once | head -5

# Verify IMU data
ros2 topic hz /livox/imu --window 5
```

### Step 11: Quality Check - Point Cloud
**Type:** Inspection | **Critical:** Yes
Open Livox Viewer 2 or RViz and verify:
- 360° horizontal coverage with no gaps
- Point density >10,000 points per frame
- No anomalous readings or noise
- Range accuracy within ±2cm at known distances

**Pass/Fail criteria:** Full 360° coverage, >10k points/frame, no gaps

### Step 12: Bag Failed Unit for RMA
**Type:** Verification | **Critical:** No
1. Place failed unit in anti-static bag
2. Attach RMA label with:
   - Serial number
   - Failure description
   - Work order number
   - Date of removal
3. Leave bagged unit at site for pickup

## Acceptance Criteria
- New LiDAR unit powered on with solid green LED
- Point cloud data streaming at expected rate (200k pts/s)
- 360° coverage verified with no gaps
- All cables properly routed and secured
- Failed unit bagged and labeled for RMA
