# SOP-PM-002: Weekly Full System Audit

**Robot:** BCR-001 (Differential Drive AMR)
**Category:** Preventive Maintenance
**Estimated Time:** 30-45 minutes
**Safety Level:** LOW
**Frequency:** Weekly, Monday AM shift

## Purpose

Comprehensive system verification to catch degradation trends before they cause failures. Covers every subsystem individually with quantitative measurements.

## Procedure

### Step 1: Full Sensor Health Baseline

```bash
~/scripts/sensor_health_check.sh
```

Record all rates. Compare against last week's readings for degradation trends.

### Step 2: Depth Camera Verification

```bash
~/scripts/camera_diagnostic.sh
```

Check:
- Rate: 29-31Hz (nominal 30Hz)
- Resolution: 640x480
- No zero-frame warnings in sensor_health.log

### Step 3: Stereo Camera Sync Check

From camera_diagnostic.sh output, verify:
- Left rate: 9-11Hz
- Right rate: 9-11Hz
- Rate difference: < 2Hz
- If desync detected, note and schedule calibration

### Step 4: IMU Calibration Check

```bash
ros2 topic echo /bcr_bot/imu --once
```

Check:
- angular_velocity values near 0 when stationary (|value| < 0.05)
- linear_acceleration z ≈ 9.81 m/s² (gravity)
- Review sensor_health.log for drift warnings

### Step 5: Battery Capacity Test

```bash
~/scripts/battery_report.sh
```

Record:
- Current voltage (should be >11.5V for healthy battery)
- Check battery log for charge/discharge cycle count
- If voltage drop from full charge to current is >1.5V in 8 hours, battery may be aging

### Step 6: Drive Encoder Check

```bash
~/scripts/drive_diagnostic.sh
```

Check joint_states for:
- Position values: note cumulative drift between L and R
- Velocity: should be 0 when robot is stationary
- Effort: should be non-zero (motors receiving power)

If encoder drift > 0.5 rad from expected, flag for recalibration.

### Step 7: Gripper Cycle Test

```bash
ros2 topic echo /bcr_bot/payload/gripper_state
```

Observe for 30 seconds:
- State should be stable ("open" or "closed")
- If "moving" persists for >5s, flag gripper fault
- Check weight sensor: should read 0 when unloaded, positive when loaded

### Step 8: Navigation System Test

If LiDAR is available:
```bash
~/scripts/start_lidar.sh
sleep 5
ros2 topic echo /bcr_bot/nav_status --once
```

If nav_status shows "navigating" or "idle" (not "emergency_stop"), navigation stack is healthy.

If LiDAR is not available, note in report and skip.

### Step 9: Log File Review

```bash
echo "=== Drive Errors ==="
grep -c "ERROR\|CRITICAL" ~/logs/drive.log
echo "=== Sensor Warnings ==="
grep -c "WARNING\|ERROR" ~/logs/sensor_health.log
echo "=== Battery Warnings ==="
grep -c "WARNING\|CRITICAL" ~/logs/battery.log
echo "=== Nav Issues ==="
grep -c "WARNING\|EMERGENCY" ~/logs/navigation.log
echo "=== Payload Issues ==="
grep -c "ERROR" ~/logs/payload.log
```

Flag any new errors since last weekly audit.

### Step 10: Generate Summary Report

Create a summary with:
- Date and time of audit
- All sensor rates (current vs expected)
- Battery health (voltage, percentage, cycle trend)
- Encoder drift (cumulative since last calibration)
- Error/warning counts from logs
- Overall assessment: PASS / MARGINAL / FAIL
- Recommended actions (if any)

Submit as work order completion with all measurements attached.
