# SOP: 2D LiDAR Replacement

**Document ID:** SOP-HW-007
**Version:** 2.0
**Category:** Hardware
**Estimated Duration:** 30 minutes

## Prerequisites
- Replacement 2D LiDAR unit
- Mounting hardware
- SSH access to robot

## Safety Notes
- Power off the robot before disconnecting the LiDAR
- Wear ESD wrist strap when handling the sensor
- Class 1 laser — eye safe under normal operation

## Tools Required
- Phillips screwdriver (PH2)
- Hex key set
- ESD wrist strap
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
- Any visible damage
- Serial number label

### Step 3: Run Pre-Removal Diagnostic
**Type:** Script | **Critical:** No
**Embedded Script:** LiDAR Diagnostic
Run the diagnostic script to capture the current state before removal:

```bash
# Check if LiDAR node is running
ros2 node list | grep two_d_lidar

# Check scan topic
ros2 topic hz /bcr_bot/scan --window 5
```

### Step 4: Disconnect Cables
**Type:** Instruction | **Critical:** Yes
1. Disconnect the data cable from the LiDAR
2. Disconnect the power cable
3. Label cables for reconnection

### Step 5: Remove Failed Unit
**Type:** Instruction | **Critical:** No
1. Remove the mounting screws
2. Carefully lift the LiDAR unit from the mounting bracket
3. Place in anti-static bag for RMA
4. Inspect mounting bracket for damage

### Step 6: Install New Unit
**Type:** Instruction | **Critical:** Yes
1. Verify new unit matches the replacement specification
2. Align LiDAR unit on mounting bracket (ensure forward-facing orientation)
3. Install mounting screws — hand-tight plus 1/4 turn
4. Verify unit is secure with no play

### Step 7: Reconnect Cables
**Type:** Instruction | **Critical:** Yes
1. Connect data cable to LiDAR port
2. Connect power cable
3. Route cables with cable ties matching original routing
4. Verify no cable strain on connectors

### Step 8: Power On and Start LiDAR
**Type:** Script | **Critical:** Yes
Power on the robot and start the LiDAR node:

```bash
# Start the 2D LiDAR node
bash ~/scripts/start_lidar.sh

# Verify scan topic is active
ros2 topic list | grep /bcr_bot/scan
```

### Step 9: Run Post-Install Diagnostic
**Type:** Script | **Critical:** Yes
**Embedded Script:** LiDAR Verification
Verify the new unit is working correctly:

```bash
# Check ROS2 node is running
ros2 node list | grep two_d_lidar

# Verify scan data rate
ros2 topic hz /bcr_bot/scan --window 10

# Check a single scan message
ros2 topic echo /bcr_bot/scan --once | head -10
```

### Step 10: Quality Check — Scan Coverage
**Type:** Inspection | **Critical:** Yes
Verify in RViz or Foxglove:
- 360-degree horizontal coverage with no gaps
- Range readings within 0.55m – 16m
- Consistent scan rate (~30 Hz)
- No anomalous readings or noise

**Pass/Fail criteria:** Full 360-degree coverage, stable rate, no gaps

### Step 11: Bag Failed Unit for RMA
**Type:** Verification | **Critical:** No
1. Place failed unit in anti-static bag
2. Attach RMA label with:
   - Serial number
   - Failure description
   - Work order number
   - Date of removal
3. Leave bagged unit at site for pickup

## Acceptance Criteria
- New LiDAR unit mounted and connected
- `/bcr_bot/scan` topic publishing at ~30 Hz
- 360-degree coverage verified with no gaps
- All cables properly routed and secured
- Failed unit bagged and labeled for RMA
