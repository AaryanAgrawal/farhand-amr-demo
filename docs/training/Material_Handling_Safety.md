# Material Handling Safety Guide

**Robot:** BCR-001 (Differential Drive AMR with Pallet Gripper)
**Compliance:** OSHA 1910.178, ANSI/RIA R15.08
**Last Updated:** 2026-03-06

## 1. Emergency Stop Procedures

### E-Stop Locations
- **Primary:** Red mushroom button on top panel of robot
- **Secondary:** Wall-mounted E-stop at each dock station (DS-01 through DS-08)
- **Software:** Via SSH: `pkill -f "python3 nodes/"` (stops all nodes)

### E-Stop Activation
1. Press the red mushroom button firmly — it locks in the pressed position
2. Robot will immediately:
   - Cut power to drive motors
   - Hold gripper in current position (does not open)
   - Continue publishing sensor data (for diagnostics)
3. Verify: No wheel movement for 10 seconds before approaching

### E-Stop Release
1. Twist the mushroom button clockwise — it pops up
2. Robot does NOT auto-resume — nodes must be restarted manually
3. Run `~/scripts/clear_faults.sh` to restart all systems
4. Verify with `~/scripts/sensor_health_check.sh` before resuming operations

## 2. Gripper Safety

### Pinch Hazard
The gripper mechanism can exert up to **500N of closing force**. This is enough to cause severe injury.

**Rules:**
- NEVER reach into the gripper opening while robot is powered
- NEVER attempt to manually pry the gripper open while powered
- If gripper is stuck, engage E-stop FIRST, then investigate
- Keep hands clear of gripper rails during any operation

### Pallet Security
- Maximum pallet weight: **30kg**
- Gripper is designed for standard EUR pallets (800x1200mm) and half-pallets (800x600mm)
- Overweight pallets may slip during transport — verify weight before pickup
- If weight sensor reads -1.0kg, the sensor is disconnected — treat pallet as unsecured

### Unsecured Pallet Protocol
If gripper fault occurs while carrying a pallet:
1. Establish 3m exclusion zone
2. Engage E-stop
3. Place physical supports under pallet before any investigation
4. Do NOT attempt to remove pallet while robot is powered

## 3. Clearance Zones

| Zone | Distance | Purpose |
|------|----------|---------|
| Operational zone | 1m | Normal operation — stay clear of path |
| Maintenance zone | 2m | During scheduled maintenance |
| Fault investigation zone | 3m | During fault recovery — cones required |
| Pedestrian crossing | 5m | When robot is navigating through crossings |

### High-Traffic Areas
- BCR-001 navigates Aisles 3-8 and Dock Stations 1-8
- Pedestrian crossings at Aisle 4/5 intersection and Dock 4/5 boundary
- Robot should slow to 0.1 m/s in crossing zones (configured in nav parameters)
- If robot stops in a crossing, redirect foot traffic before investigating

## 4. PPE Requirements

| Task | Safety Glasses | Steel-Toe Boots | Gloves | Hard Hat |
|------|---------------|-----------------|--------|----------|
| Daily check (SSH only) | No | No | No | No |
| Physical inspection | Yes | Yes | No | No |
| Motor/drive work | Yes | Yes | No | No |
| Gripper work | Yes | Yes | Cut-resistant | No |
| Battery work | Yes | Yes | Insulated | No |
| Overhead work | Yes | Yes | No | Yes |

## 5. Battery Safety

- Battery type: 12V 20Ah LiFePO4
- Do NOT short-circuit terminals
- Do NOT expose to water or extreme heat (>60°C)
- If battery is swollen or leaking, do NOT handle — evacuate area and call safety team
- Always disconnect negative terminal first, reconnect positive terminal first
- Dispose through approved recycling program (see SOP-HW-012)

## 6. Incident Reporting

**ALL incidents must be reported, including near-misses.**

Report immediately:
- Any contact between robot and person
- Pallet drop or shift during transport
- E-stop activation (log reason)
- Equipment damage (robot or warehouse infrastructure)
- Battery thermal event

Report within shift:
- Sensor faults that could affect safety (LiDAR, cameras)
- Navigation errors (wrong path, unexpected stops)
- Gripper anomalies (slow close, weight mismatch)

Use the Farhand platform to create a corrective work order with:
- Time and location of incident
- Description of what happened
- Photos if applicable
- Immediate actions taken
- Witnesses (if any)
