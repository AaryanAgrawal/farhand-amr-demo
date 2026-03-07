# SOP-MH-001: Gripper Fault Recovery

**Robot:** BCR-001 (Differential Drive AMR with Pallet Gripper)
**Category:** Corrective Maintenance — Material Handling
**Estimated Time:** 20-30 minutes
**Safety Level:** HIGH — gripper pinch hazard, unsecured payload risk

## Prerequisites

- PPE: Safety glasses, steel-toe boots, cut-resistant gloves
- Physical access to robot and gripper mechanism
- SSH access to BCR-001

## SAFETY WARNING

**NEVER reach into the gripper mechanism while the robot is powered on.** The gripper can close with 500N of force without warning. Always engage the E-stop and verify the gripper is de-energized before physical inspection.

If a pallet is partially gripped (stuck in "moving" state), it may be unsecured and could fall. Establish a 3m exclusion zone around the robot before approaching.

## Procedure

### Step 1: Safety Lockout

1. Engage E-stop on the robot
2. Place lockout tag on E-stop: "DO NOT OPERATE — Gripper Fault Investigation"
3. If pallet is partially gripped, place supports under the pallet before proceeding
4. Establish 3m exclusion zone with safety cones

### Step 2: Check Payload Node Status

SSH into BCR-001:

```bash
ros2 node list | grep payload
ros2 topic echo /bcr_bot/payload/gripper_state --once
ros2 topic echo /bcr_bot/payload/status --once
```

Expected fault indicators:
- gripper_state: "moving" (stuck)
- status: weight_kg = -1.0 (sensor disconnected)
- status: loaded = false (despite pallet present)

### Step 3: Check Weight Sensor

```bash
# Read the last payload log entries
tail -20 ~/logs/payload.log
```

Look for:
- "Weight sensor reading -1.0kg" — sensor cable disconnected or sensor failure
- "Gripper stuck in MOVING state" — actuator jammed or control board fault

### Step 4: Inspect Gripper Mechanism (Physical)

**E-stop must be engaged. Verify visually that gripper is not moving.**

1. Check gripper rails for debris or foreign objects
2. Check gripper actuator cable connection (3-pin Molex connector)
3. Check weight sensor cable (2-pin JST connector at gripper base)
4. Check for hydraulic/pneumatic line damage (if applicable)

### Step 5: Test Open/Close Cycle

After physical inspection, if no damage found:

1. Release E-stop
2. Restart payload node:
   ```bash
   pkill -f payload_node.py
   sleep 2
   cd /home/ubuntu && python3 nodes/payload_node.py &
   ```
3. Monitor gripper state:
   ```bash
   ros2 topic echo /bcr_bot/payload/gripper_state
   ```
4. Wait 10 seconds. State should cycle: open → moving → closed (or remain "open" if no pallet)

### Step 6: Verify Pallet Handling

If gripper is functional:
1. Check weight sensor reads correctly:
   ```bash
   ros2 topic echo /bcr_bot/payload/status --once
   ```
2. Weight should be positive if pallet is loaded (typical: 15-30kg)
3. Pallet ID should be populated

### Step 7: Document Findings

Record in work order:
- Fault type (mechanical jam, sensor disconnect, control board, other)
- Root cause if identified
- Parts replaced (if any)
- Gripper test result (pass/fail)
- Weight sensor test result (pass/fail)

## Escalation Criteria

Escalate immediately if:
- Gripper actuator physically damaged (bent rails, cracked housing)
- Weight sensor reads -1.0 after reconnecting cable
- Gripper cycles but does not reach full closed position
- Hydraulic/pneumatic leak detected
- Pallet was dropped or shifted during fault
