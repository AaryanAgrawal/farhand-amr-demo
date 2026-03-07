# SOP-HW-012: Battery Replacement

**Robot:** BCR-001 (Differential Drive AMR)
**Category:** Corrective Maintenance
**Estimated Time:** 15-20 minutes
**Safety Level:** MEDIUM — electrical hazard

## Prerequisites

- PPE: Insulated gloves, safety glasses
- Replacement battery: 12V 20Ah LiFePO4 (part# BAT-LFP-1220)
- Torque wrench (5 Nm for terminal bolts)
- Multimeter for voltage verification

## Procedure

### Step 1: Power Down

1. Engage E-stop
2. SSH into robot and run clean shutdown:
   ```bash
   pkill -f "python3 nodes/"
   ```
3. Wait 10 seconds for all nodes to stop
4. Power off main switch (rear panel, bottom left)

### Step 2: Disconnect Old Battery

1. Open battery compartment (4x M4 screws on bottom panel)
2. Disconnect negative terminal FIRST (black wire)
3. Disconnect positive terminal (red wire)
4. Remove battery from compartment (weight: ~2.5kg)

### Step 3: Inspect Terminals

1. Check terminal connectors for corrosion (green/white buildup)
2. Check wire insulation for cracks or damage
3. If corrosion found: clean with baking soda solution, dry thoroughly
4. Check compartment for moisture or debris

### Step 4: Install New Battery

1. Place new battery in compartment (label facing up)
2. Connect positive terminal FIRST (red wire)
3. Torque to 5 Nm
4. Connect negative terminal (black wire)
5. Torque to 5 Nm
6. Verify no loose connections by gentle tug test

### Step 5: Power On and Verify

1. Close battery compartment
2. Power on main switch
3. Release E-stop
4. Wait for boot sequence (~30 seconds)
5. SSH in and check battery state:
   ```bash
   ~/scripts/battery_report.sh
   ```

Expected readings for new battery:
- Voltage: 12.4-12.8V
- Percentage: 95-100%
- Health: GOOD
- Status: DISCHARGING (or NOT_CHARGING if idle)

### Step 6: Run Health Check

```bash
~/scripts/sensor_health_check.sh
```

All sensors should show OK. If any sensor fails to start, check power distribution board connections.

## Old Battery Disposal

- Label old battery with date removed and reason
- Store in battery recycling bin (Building C, Room 102)
- Do NOT dispose in regular waste
- Log battery serial number in maintenance system

## Troubleshooting

| Symptom | Cause | Action |
|---------|-------|--------|
| Voltage reads 0V after install | Reversed polarity | Disconnect immediately, swap terminals |
| Voltage reads <11V on new battery | Defective battery | Replace with another unit |
| Robot won't boot | Main fuse blown | Check 20A fuse on power board |
| Sensors don't start | Power distribution fault | Check 5V/12V rails on distribution board |
