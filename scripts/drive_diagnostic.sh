#!/bin/bash
# BCR-001 Drive Motor Diagnostic
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null

echo ""
echo "=== DRIVE DIAGNOSTIC ==="
echo ""

echo "1. Node status:"
ros2 node list 2>/dev/null | grep -q drive && echo "   drive_node: RUNNING" || echo "   drive_node: NOT FOUND"
echo ""

echo "2. Odometry rate:"
rate=$(timeout 4 ros2 topic hz /bcr_bot/odom --window 3 2>/dev/null | grep "average" | head -1)
if [ -n "$rate" ]; then
    echo "   $rate"
else
    echo "   No odometry data received"
fi
echo ""

echo "3. Last velocity command:"
vel=$(timeout 2 ros2 topic echo /bcr_bot/cmd_vel --once 2>/dev/null)
if [ -n "$vel" ]; then
    echo "$vel" | head -8 | sed 's/^/   /'
else
    echo "   No velocity commands received"
fi
echo ""

echo "4. Joint states (latest):"
joints=$(timeout 2 ros2 topic echo /bcr_bot/joint_states --once 2>/dev/null)
if [ -n "$joints" ]; then
    echo "$joints" | head -12 | sed 's/^/   /'
else
    echo "   No joint state data"
fi
echo ""

echo "5. Drive log (last 10 lines):"
if [ -f ~/logs/drive.log ]; then
    tail -10 ~/logs/drive.log | sed 's/^/   /'
else
    echo "   No drive log found"
fi
echo ""
