#!/bin/bash
# BCR-001 Battery Status Report
source /opt/ros/humble/setup.bash

echo ""
echo "=== BATTERY REPORT ==="
echo ""

data=$(timeout 3 ros2 topic echo /bcr_bot/battery_state --once 2>/dev/null)
if [ -z "$data" ]; then
    echo "ERROR: Cannot read battery state topic"
    exit 1
fi

echo "Current state:"
echo "$data" | grep -E "voltage|current|percentage|power_supply_status|power_supply_health" | sed 's/^/   /'
echo ""

pct=$(echo "$data" | grep "percentage" | awk '{print $2}')
if [ -n "$pct" ]; then
    level=$(echo "$pct" | awk '{if ($1 > 0.2) print "OK"; else if ($1 > 0.1) print "LOW - charge soon"; else print "CRITICAL - charge immediately"}')
    echo "Assessment: $level"
fi
echo ""

echo "Battery log (last 5 entries):"
if [ -f ~/logs/battery.log ]; then
    tail -5 ~/logs/battery.log | sed 's/^/   /'
else
    echo "   No battery log found"
fi
echo ""
