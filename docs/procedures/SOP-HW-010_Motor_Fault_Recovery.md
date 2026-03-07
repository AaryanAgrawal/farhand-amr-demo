# SOP-HW-010: Motor Fault Recovery

**Robot:** BCR-001 (Differential Drive AMR)
**Category:** Corrective Maintenance
**Estimated Time:** 20-30 minutes
**Safety Level:** HIGH — E-stop must be verified before any work

## Prerequisites

- PPE: Safety glasses, steel-toe boots
- Access to BCR-001 via SSH
- Physical access to robot if motor inspection required
- drive_diagnostic.sh script available

## Procedure

### Step 1: Verify E-Stop (SAFETY)

**WARNING: Do NOT approach the robot until E-stop is confirmed engaged.**

1. Visually confirm the E-stop button is pressed (red mushroom button on top panel)
2. Verify no wheel movement by observation for 10 seconds
3. If robot is in a high-traffic area, place safety cones around a 2m perimeter

### Step 2: Read Fault Code

SSH into BCR-001 and check the drive log:

```bash
tail -20 ~/logs/drive.log
```

Common fault codes:
| Code | Meaning | Severity |
|------|---------|----------|
| 0x01 | Encoder communication error | Medium |
| 0x02 | Motor overtemperature | High |
| 0x03 | Overcurrent (left or right motor) | High |
| 0x04 | Undervoltage to motor controller | Medium |
| 0x05 | Hall sensor fault | High |

### Step 3: Run Drive Diagnostic

```bash
~/scripts/drive_diagnostic.sh
```

Check output for:
- Node status (should show drive_node: RUNNING or NOT FOUND)
- Odometry rate (should be ~50Hz when healthy)
- Joint state effort values (should be non-zero when motors are powered)

### Step 4: Check Encoder Alignment

```bash
ros2 topic echo /bcr_bot/joint_states --once
```

Compare `position` values for left and right wheels. If drift exceeds 0.5 rad from expected, encoder may need recalibration.

### Step 5: Clear Fault and Restart

Kill and restart the drive node:

```bash
pkill -f drive_node.py
sleep 2
cd /home/ubuntu && python3 nodes/drive_node.py &
```

Wait 5 seconds, then verify:

```bash
tail -5 ~/logs/drive.log
```

Look for "Ready. Max velocity: 0.26 m/s" — this confirms the fault cleared.

### Step 6: Test Movement

Send a low-speed test velocity command:

```bash
ros2 topic pub /bcr_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
```

Observe:
- Wheels should rotate slowly forward
- Odometry should update (`ros2 topic echo /bcr_bot/odom --once`)
- No new fault codes in drive log

### Step 7: Verify Odometry Accuracy

```bash
timeout 5 ros2 topic hz /bcr_bot/odom --window 5
```

Expected: 49-51 Hz. If significantly lower, motor controller may have partial fault.

### Step 8: Return to Service

1. Release E-stop (twist clockwise to disengage)
2. Run full sensor health check: `~/scripts/sensor_health_check.sh`
3. Log the incident and resolution in the work order
4. If fault recurs within 24 hours, escalate to Level 2 maintenance

## Escalation Criteria

Escalate immediately if:
- Fault code 0x05 (Hall sensor) — requires physical motor replacement
- Fault recurs 3+ times in 24 hours
- Visible damage to motor, wiring, or encoder
- Burning smell from motor housing
- Robot was carrying a payload when fault occurred (pallet may be unsecured)
