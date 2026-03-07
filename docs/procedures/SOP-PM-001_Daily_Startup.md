# SOP-PM-001: Daily Startup Check

**Robot:** BCR-001 (Differential Drive AMR)
**Category:** Preventive Maintenance
**Estimated Time:** 5-10 minutes
**Safety Level:** LOW
**Frequency:** Daily, start of shift

## Procedure

### Step 1: Verify SSH Connectivity

```bash
ssh ubuntu@<robot_ip> -p 2222
```

If connection fails:
- Check robot power (front panel LED should be solid green)
- Check network connection (Tailscale or local WiFi)
- Try ping: `ping <robot_ip>`

### Step 2: Run Sensor Health Check

```bash
~/scripts/sensor_health_check.sh
```

All sensors should show "OK" status:
- Depth Camera: ~30Hz
- Stereo Left/Right: ~10Hz each
- IMU: ~5Hz
- Battery: ~1Hz
- Drive/Odom: ~50Hz
- Navigation: ~1Hz
- Payload: ~1Hz
- Gripper: ~2Hz

**LiDAR will show OFF unless explicitly started.** This is normal for default configuration.

### Step 3: Check Battery Level

```bash
~/scripts/battery_report.sh
```

| Battery Level | Action |
|---------------|--------|
| > 50% | OK — proceed with operations |
| 20-50% | Monitor — plan charging during next break |
| 10-20% | Charge soon — schedule dock within 1 hour |
| < 10% | Charge immediately — do not assign delivery tasks |

### Step 4: Check Drive System

```bash
~/scripts/drive_diagnostic.sh
```

Verify:
- drive_node is RUNNING
- Odometry rate is ~50Hz
- No fault codes in drive log

### Step 5: Log Results

Record daily check results in the work order system:
- All sensors: PASS / FAIL (list failures)
- Battery: XX%
- Drive: PASS / FAIL
- Overall: READY / DEGRADED / NOT OPERATIONAL

If any sensor shows FAIL or battery is below 10%, create a corrective work order before assigning tasks to the robot.
