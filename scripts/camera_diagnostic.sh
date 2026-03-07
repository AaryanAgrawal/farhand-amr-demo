#!/bin/bash
# BCR-001 Camera System Diagnostic
source /opt/ros/humble/setup.bash

echo ""
echo "=== CAMERA DIAGNOSTIC ==="
echo ""

echo "1. Depth camera rate:"
rate=$(timeout 6 ros2 topic hz /bcr_bot/camera/image_raw --window 3 2>/dev/null | grep "average" | head -1)
[ -n "$rate" ] && echo "   $rate" || echo "   No data (camera may be offline)"
echo ""

echo "2. Stereo left rate:"
rate=$(timeout 6 ros2 topic hz /bcr_bot/stereo/left/image_raw --window 3 2>/dev/null | grep "average" | head -1)
[ -n "$rate" ] && echo "   $rate" || echo "   No data"
echo ""

echo "3. Stereo right rate:"
rate=$(timeout 6 ros2 topic hz /bcr_bot/stereo/right/image_raw --window 3 2>/dev/null | grep "average" | head -1)
[ -n "$rate" ] && echo "   $rate" || echo "   No data"
echo ""

echo "4. Depth camera resolution:"
info=$(timeout 2 ros2 topic echo /bcr_bot/camera/camera_info --once 2>/dev/null)
if [ -n "$info" ]; then
    w=$(echo "$info" | grep "width:" | awk '{print $2}')
    h=$(echo "$info" | grep "height:" | awk '{print $2}')
    echo "   ${w}x${h}"
else
    echo "   Cannot read camera info"
fi
echo ""

echo "5. Stereo sync check:"
l_rate=$(timeout 6 ros2 topic hz /bcr_bot/stereo/left/image_raw --window 3 2>/dev/null | grep "average" | head -1 | awk '{printf "%.1f", $3}')
r_rate=$(timeout 6 ros2 topic hz /bcr_bot/stereo/right/image_raw --window 3 2>/dev/null | grep "average" | head -1 | awk '{printf "%.1f", $3}')
if [ -n "$l_rate" ] && [ -n "$r_rate" ]; then
    echo "   Left: ${l_rate}Hz  Right: ${r_rate}Hz"
    diff=$(echo "$l_rate $r_rate" | awk '{d=$1-$2; if(d<0) d=-d; print d}')
    ok=$(echo "$diff" | awk '{if ($1 < 2.0) print "YES"; else print "NO"}')
    echo "   Sync OK: $ok (rate difference: ${diff}Hz)"
else
    echo "   Cannot check sync — one or both cameras offline"
fi
echo ""

echo "6. Sensor health log (camera entries):"
if [ -f ~/logs/sensor_health.log ]; then
    grep -i "camera\|stereo\|depth" ~/logs/sensor_health.log | tail -5 | sed 's/^/   /'
else
    echo "   No sensor health log found"
fi
echo ""
