# SOP-NW-001: WiFi Reconnection Procedure

## Purpose

Restore wireless network connectivity on a robot that has lost its WiFi connection to the site network.

## Applies To

All BCR-series robots with onboard WiFi adapters (wlan0 interface).

## Prerequisites

- Physical access to the robot
- Site WiFi SSID and password (available in site configuration)
- Laptop or tablet with SSH access to robot (via Ethernet fallback)

## Safety Notes

- Ensure robot is stationary and e-stopped before performing network changes.
- Do not power cycle the robot unless all other steps fail.

## Procedure

### Step 1: Check Signal Strength

Connect to the robot via Ethernet fallback cable and run:

    iwconfig wlan0

Verify the signal level. If signal level is below -80 dBm, reposition the robot closer to the access point before continuing.

### Step 2: Verify Interface Status

Check if the wireless interface is up:

    ip link show wlan0

If the interface shows DOWN, bring it up:

    sudo ip link set wlan0 up

### Step 3: Restart Network Service

Restart the networking service to force reassociation:

    sudo systemctl restart NetworkManager

Wait 10 seconds for the connection to re-establish.

### Step 4: Verify IP Address

Confirm the robot obtained an IP address on the site network:

    ip addr show wlan0

The interface should show an inet address in the expected subnet (e.g., 192.168.1.x).

### Step 5: Ping Controller

Verify end-to-end connectivity by pinging the fleet controller:

    ping -c 3 192.168.1.1

All 3 packets should return with <10ms latency on a local network.

### Step 6: Verify ROS2 Communication

Confirm ROS2 topics are flowing across the network:

    ros2 topic list

The robot should publish /odom, /scan, /cmd_vel, and other standard topics.

## Estimated Duration

5-10 minutes

## Escalation

If WiFi cannot be restored after completing all steps, escalate to HQ with the output of iwconfig and ip addr for further analysis.
