# DARPA LIFT Challenge - Quick Reference Card

## Essential Mission Parameters

| Parameter | Value | Metric |
|-----------|-------|--------|
| Aircraft weight limit | < 55 lbs | 24.95 kg |
| Minimum payload | 110 lbs | 49.9 kg |
| Total flight distance | 5 nm | 9,260 m |
| Distance WITH payload | 4 nm | 7,408 m |
| Distance WITHOUT payload | 1 nm | 1,852 m |
| Cruise altitude | 350 ft ± 50 ft | 106.68 m ± 15.24 m |
| Landing circle radius | 10 ft | 3.05 m |
| Time limit | < 30 minutes | 1,800 seconds |

## Quick Start Commands

### Terminal 1: DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2: Launch Simulation
```bash
# Using the helper script
~/launch_darpa_lift.sh

# OR manually
PX4_SYS_AUTOSTART=4010 \
PX4_SIM_MODEL=gz_x500_mono_cam \
PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" \
PX4_GZ_WORLD=darpa_lift_challenge \
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

### Terminal 3: Run Mission
```bash
# Using the helper script  
~/run_darpa_mission.sh

# OR manually
cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2
source install/local_setup.bash
ros2 run my_offboard_ctrl darpa_lift_mission
```

## Mission Waypoints (NED Coordinates)

| Name | North (X) | East (Y) | Down (Z) | Altitude |
|------|-----------|----------|----------|----------|
| Start/Landing | 0 m | 0 m | 0 m | 0 ft |
| Takeoff Point | 0 m | 0 m | -106.68 m | 350 ft |
| Waypoint 1 | 1,852 m | 0 m | -106.68 m | 350 ft |
| Waypoint 2 | 3,704 m | 0 m | -106.68 m | 350 ft |
| Waypoint 3 | 5,556 m | 0 m | -106.68 m | 350 ft |
| Payload Dropoff | 7,408 m | 0 m | 0 m | 0 ft |
| Waypoint 4 | 9,260 m | 0 m | -106.68 m | 350 ft |

## Mission Sequence

```
1. IDLE → Wait for telemetry
2. ARMING → Arm motors and engage offboard mode
3. TAKING_OFF → Climb to 350 ft
4. WAYPOINT_1 → Fly to 1 nm (WITH payload)
5. WAYPOINT_2 → Fly to 2 nm (WITH payload)
6. WAYPOINT_3 → Fly to 3 nm (WITH payload)
7. FLYING_TO_DROPOFF → Fly to 4 nm (WITH payload)
8. DROPPING_PAYLOAD → Descend, release, and climb
9. WAYPOINT_4 → Fly to 5 nm (WITHOUT payload)
10. RETURNING_HOME → Fly back to start
11. LANDING → Land in 10 ft circle
12. COMPLETED → Mission done!
```

## Common ROS2 Commands

### Check if mission node is running
```bash
ros2 node list | grep darpa
```

### Monitor mission output
```bash
ros2 topic echo /fmu/out/vehicle_local_position
```

### Check vehicle status
```bash
ros2 topic echo /fmu/out/vehicle_status
```

### List all PX4 topics
```bash
ros2 topic list | grep fmu
```

### Rebuild workspace after changes
```bash
cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2
colcon build --packages-select my_offboard_ctrl
source install/local_setup.bash
```

## Troubleshooting Quick Fixes

### Problem: "Service call timed out"
```bash
# Just restart PX4
# Press Ctrl+C and run the launch command again
```

### Problem: Drone doesn't arm
```bash
# Check all terminals are running:
# 1. MicroXRCEAgent
# 2. PX4 (should show "Ready for takeoff")
# 3. Mission node

# Check topics are publishing:
ros2 topic list | grep fmu
```

### Problem: Drone flies erratically
```bash
# Check coordinates are negative for altitude
# Z = -106.68 means UP 106.68 meters
# Z = +106.68 means DOWN 106.68 meters (wrong!)
```

### Problem: Can't see the world
```bash
# Verify world file is in correct location:
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds/darpa_lift_challenge.sdf

# Check world name matches:
echo $PX4_GZ_WORLD  # Should be "darpa_lift_challenge"
```

### Problem: Mission stops unexpectedly
```bash
# Check logs for errors:
ros2 run my_offboard_ctrl darpa_lift_mission 2>&1 | tee mission_log.txt

# Check PX4 console for errors
```

## File Locations

| File | Location |
|------|----------|
| World SDF | `~/PX4-Autopilot/Tools/simulation/gz/worlds/darpa_lift_challenge.sdf` |
| Mission Python | `~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2/src/my_offboard_ctrl/darpa_lift_mission.py` |
| Launch Script | `~/launch_darpa_lift.sh` |
| Mission Runner | `~/run_darpa_mission.sh` |
| README | `README_DARPA_LIFT.md` |
| This Card | `QUICK_REFERENCE.md` |

## Keyboard Shortcuts in Gazebo

| Key | Action |
|-----|--------|
| Mouse drag | Rotate view |
| Scroll wheel | Zoom in/out |
| Shift + mouse | Pan view |
| Ctrl + R | Reset view |
| Spacebar | Pause/Resume simulation |

## Important Notes

⚠️ **Altitude in NED Frame**
- DOWN is POSITIVE Z
- UP is NEGATIVE Z
- `-106.68` means 106.68 meters UP
- `0` means ground level

⚠️ **Payload Requirements**
- Minimum 110 lbs (49.9 kg)
- Must use Olympic barbell plates
- Must be co-located (not distributed)
- Must touch ground before release

⚠️ **Landing Requirements**
- Must land within 10 ft (3.05 m) radius
- Must be controlled (no damage)
- Landing zone center is at (0, 0, 0)

⚠️ **Time Limit**
- Must complete in under 30 minutes
- Timer starts at payload liftoff
- Timer stops at landing with motors cut

## Scoring

```
Score = Payload Weight / Aircraft Weight

Target: 4:1 ratio (4x payload-to-weight)

Tiebreakers:
1. Highest payload weight
2. Fastest time
```

## Visual Markers in Simulation

| Color | Meaning |
|-------|---------|
| 🟡 Yellow | Takeoff/Landing Zone (10 ft radius) |
| 🟢 Green | Waypoints 1-3 (WITH payload) |
| 🔴 Red | Payload Dropoff Zone |
| 🔵 Blue | Waypoint 4 (WITHOUT payload) |
| 🟢 Green plane | Target altitude (350 ft) |
| 🔴 Red planes | Min/max altitude boundaries |

## Key Safety Rules

✓ VTOL only (no runway or launch aids)
✓ Visual Line of Sight (VLOS) required
✓ Maintain 350 ft ± 50 ft altitude
✓ Payload must touch ground before release
✓ Land within 10 ft radius
✓ No damage requiring repairs
✓ Follow all FAA regulations

## Next Steps

1. ✅ Install files using `setup_darpa_lift.sh`
2. ✅ Test basic simulation launch
3. ✅ Run mission in simulation
4. 📝 Analyze flight data
5. 📝 Tune parameters for your aircraft
6. 📝 Add payload attachment mechanism
7. 📝 Test with increasing payload weights
8. 📝 Practice landing accuracy
9. 📝 Optimize for speed
10. 🚁 Ready for competition!

## Resources

- **Main README**: `README_DARPA_LIFT.md`
- **Mission Diagram**: `MISSION_DIAGRAM.txt`
- **PX4 Docs**: https://docs.px4.io/
- **DARPA LIFT**: https://www.darpa.mil/research/challenges/lift
- **Support Forum**: https://discuss.px4.io/

---

**Quick tip**: Keep this file open in a terminal while running the simulation for quick reference!

```bash
less QUICK_REFERENCE.md
# Press 'q' to quit, arrow keys to navigate
```
