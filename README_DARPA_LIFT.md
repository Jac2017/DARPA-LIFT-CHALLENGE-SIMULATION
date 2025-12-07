# DARPA LIFT Challenge Mission Map

This package contains a complete simulation environment for the DARPA LIFT Challenge mission using PX4, Gazebo Harmonic, and ROS2 Humble.

## Mission Overview

The DARPA LIFT Challenge requires a drone to:
- **Weight limit**: Under 55 lbs (24.95 kg) empty
- **Payload**: Minimum 110 lbs (49.9 kg) 
- **Flight distance**: 5 nautical miles total
  - 4 nautical miles WITH payload
  - 1 nautical mile WITHOUT payload  
- **Altitude**: 350 feet (106.68m) ± 50 feet
- **Time limit**: Under 30 minutes
- **Landing**: Within 10-foot (3.05m) radius circle

## Files Included

### 1. `darpa_lift_challenge.sdf` - Gazebo World File
This is the 3D simulation world that contains:
- **Takeoff/Landing Zone**: Yellow circle at origin (0, 0, 0) with 10 ft radius
- **Waypoint 1**: 1 nautical mile north - green marker at 350 ft altitude
- **Waypoint 2**: 2 nautical miles north - green marker at 350 ft altitude
- **Waypoint 3**: 3 nautical miles north - green marker at 350 ft altitude  
- **Payload Dropoff Zone**: 4 nautical miles north - large red target circle on ground
- **Waypoint 4**: 5 nautical miles north - blue marker at 350 ft altitude (NO payload)
- **Altitude Reference Planes**: 
  - Red plane at 300 ft (minimum altitude)
  - Green plane at 350 ft (target altitude)
  - Red plane at 400 ft (maximum altitude)

### 2. `darpa_lift_mission.py` - ROS2 Mission Planner
This is a Python script that autonomously flies the complete mission. It includes:
- Extensive comments explaining every section of code
- State machine for mission control
- Automatic waypoint navigation
- Payload release simulation
- Altitude monitoring
- Mission timing
- Error handling

## Installation Instructions

### Step 1: Copy the World File to PX4

Copy the world file into your PX4 installation:

```bash
# Navigate to your simulation directory
cd ~/DARPA-LIFT-CHALLENGE-SIMULATION

# Copy the world file to PX4 worlds directory
cp darpa_lift_challenge.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### Step 2: Install the ROS2 Mission Node

```bash
# Navigate to your ROS2 workspace
cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2/src

# Create a new package for the mission (if not already existing)
# If my_offboard_ctrl already exists, just copy the file there
cp darpa_lift_mission.py my_offboard_ctrl/

# Make the script executable
chmod +x my_offboard_ctrl/darpa_lift_mission.py

# Build the workspace
cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2
colcon build

# Source the workspace
source install/local_setup.bash
```

### Step 3: Update package.xml (if needed)

If you created a new package, make sure `package.xml` includes:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>px4_msgs</exec_depend>
```

And in `setup.py`, add the script to the entry points:

```python
entry_points={
    'console_scripts': [
        'darpa_lift_mission = my_offboard_ctrl.darpa_lift_mission:main',
    ],
},
```

## Running the Mission

### Terminal 1: Start Micro XRCE-DDS Agent
This allows ROS2 to communicate with PX4.

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2: Launch PX4 with DARPA LIFT World
This starts the PX4 flight controller and Gazebo simulation.

```bash
# Launch PX4 with the DARPA LIFT Challenge world
PX4_SYS_AUTOSTART=4010 \
PX4_SIM_MODEL=gz_x500_mono_cam \
PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" \
PX4_GZ_WORLD=darpa_lift_challenge \
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

**Note**: The first time you run this, it may take a moment to load the world.

### Terminal 3: Launch QGroundControl (Optional)
QGroundControl provides a ground station interface to monitor the mission.

```bash
# Just open QGroundControl - it will auto-connect
./QGroundControl.AppImage
```

### Terminal 4: Run the Mission Planner

```bash
# Navigate to workspace
cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2

# Source the workspace  
source install/local_setup.bash

# Run the mission!
ros2 run my_offboard_ctrl darpa_lift_mission
```

## What You'll See

When you run the mission, you should see:

1. **Console Output**: The mission planner will log its progress:
   ```
   [INFO] Starting DARPA LIFT Challenge Mission!
   [INFO] Vehicle ARMED - Beginning takeoff
   [INFO] Reached cruise altitude: 106.68 meters
   [INFO] Proceeding to Waypoint 1 (1 nm)
   [INFO] Distance to Waypoint 1: 1500.0m
   ...
   [INFO] PAYLOAD RELEASED!
   ...
   [INFO] TOUCHDOWN! Mission Complete!
   [INFO] Total mission time: 12m 34s
   ```

2. **Gazebo View**: You'll see the drone:
   - Take off vertically from the yellow landing zone
   - Fly north through the green waypoint markers
   - Descend at the red payload dropoff zone
   - Climb back up and continue to the blue waypoint
   - Return to the yellow landing zone

3. **QGroundControl** (if running): Will show:
   - Real-time position on map
   - Altitude graph
   - Speed and battery status
   - Mission progress

## Understanding the Code

### Mission States
The mission uses a state machine with these states:
- `IDLE`: Waiting to start
- `ARMING`: Arming the motors
- `TAKING_OFF`: Climbing to 350 ft
- `WAYPOINT_1` through `WAYPOINT_4`: Flying to waypoints
- `DROPPING_PAYLOAD`: Descending and releasing payload
- `RETURNING_HOME`: Flying back to start
- `LANDING`: Final descent
- `COMPLETED`: Mission finished

### Key Coordinate System (NED)
PX4 uses the NED (North-East-Down) coordinate system:
- **North**: +X direction
- **East**: +Y direction  
- **Down**: +Z direction (negative Z = altitude)

So a position of `(1852, 0, -106.68)` means:
- 1852 meters north
- 0 meters east
- 106.68 meters UP (negative down)

### Waypoint Navigation
The mission checks if the drone is within `position_tolerance` (5 meters) of each waypoint:

```python
distance = sqrt(dx² + dy² + dz²)
if distance < position_tolerance:
    # Waypoint reached!
```

### Payload Release
Currently simulated with a log message. To add real payload release:

```python
def simulate_payload_release(self):
    # Add your servo/mechanism control here
    # For example, publish to a servo topic:
    # self.servo_publisher.publish(release_command)
    self.payload_released = True
```

## Customization

### Change Altitude
Edit the target altitude in `darpa_lift_mission.py`:

```python
self.target_altitude = -106.68  # Change this value (negative = up)
```

### Change Waypoint Positions
Edit the waypoints list:

```python
self.waypoints = [
    {'name': 'Waypoint 1', 'x': 1852.0, 'y': 0.0, 'z': -106.68},
    # Add, remove, or modify waypoints here
]
```

### Adjust Tolerances
Make waypoint detection more/less strict:

```python
self.position_tolerance = 5.0  # Increase for looser, decrease for tighter
```

### Add Different Flight Pattern
Instead of straight north, you could fly a rectangular or circular pattern by changing waypoint coordinates.

## Troubleshooting

### Problem: "ERROR [gz_bridge] Service call timed out"
**Solution**: This is normal on first launch. Just try again:
```bash
# Press Ctrl+C and re-run the command
PX4_GZ_WORLD=darpa_lift_challenge ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

### Problem: Drone doesn't arm
**Solution**: 
1. Check that Micro XRCE-DDS Agent is running in Terminal 1
2. Ensure PX4 is fully started (wait for "INFO [commander] Ready for takeoff")
3. Verify the mission node is receiving telemetry

### Problem: Drone flies away or crashes
**Solution**: 
1. Check that coordinate system is NED (negative Z = up)
2. Verify waypoint coordinates match the world
3. Make sure offboard mode engaged successfully

### Problem: Can't see the world in Gazebo
**Solution**: 
1. Verify the world file was copied correctly to `~/PX4-Autopilot/Tools/simulation/gz/worlds/`
2. Check that the world name matches: `PX4_GZ_WORLD=darpa_lift_challenge`
3. Look for error messages in the PX4 terminal

## Mission Requirements Checklist

Use this to verify your setup meets DARPA LIFT Challenge rules:

- [ ] Drone weighs less than 55 lbs (modify model as needed)
- [ ] Payload is at least 110 lbs (simulated in mission)
- [ ] Flight path is 5 nautical miles total
- [ ] 4 nm WITH payload, 1 nm WITHOUT payload
- [ ] Altitude maintained at 350 ft ± 50 ft
- [ ] Payload released in controlled manner (touch ground first)
- [ ] Landing within 10 ft radius circle
- [ ] Mission completes in under 30 minutes
- [ ] VTOL takeoff and landing (no runway)

## Next Steps for Real Implementation

To use this with a real drone, you'll need to:

1. **Add Payload Attachment Mechanism**
   - Physical mechanism to hold Olympic barbell plates
   - Weight sensor to verify payload

2. **Add Payload Release Mechanism**  
   - Servo or solenoid to release payload
   - Ensure controlled descent before release

3. **Add GPS Waypoint Navigation**
   - Convert simulation coordinates to GPS coordinates
   - Use PX4's mission planning system

4. **Add Safety Features**
   - Geofencing to keep drone in bounds
   - Return-to-home on low battery
   - Emergency landing procedures

5. **Test with Increasing Payload**
   - Start with light payload
   - Gradually increase to 110+ lbs
   - Tune PID controllers for heavy payload

## Resources

- **PX4 Documentation**: https://docs.px4.io/
- **Gazebo Documentation**: https://gazebosim.org/docs
- **ROS2 Humble Documentation**: https://docs.ros.org/en/humble/
- **DARPA LIFT Challenge**: https://www.darpa.mil/research/challenges/lift
- **DARPA LIFT Rules**: https://www.darpa.mil/research/challenges/lift/rules

## License

This code is provided as-is for educational and competition purposes.

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review PX4 and ROS2 documentation
3. Post in PX4 forums: https://discuss.px4.io/

---

**Good luck with the DARPA LIFT Challenge!** 🚁
