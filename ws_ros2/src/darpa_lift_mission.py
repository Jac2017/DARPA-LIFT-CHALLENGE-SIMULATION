#!/usr/bin/env python3
"""
DARPA LIFT Challenge Mission Planner

This ROS2 node autonomously flies the DARPA LIFT Challenge mission:
- Takes off vertically to 350 ft (106.68m)
- Flies through 4 waypoints WITH payload
- Drops payload at 4nm point
- Flies final 1nm WITHOUT payload  
- Returns to landing zone
- Lands within 10 ft radius circle

Mission requirements:
- Total distance: 5 nautical miles
- Altitude: 350 ft ± 50 ft (106.68m ± 15.24m)
- Time limit: Under 30 minutes
- Payload: Minimum 110 lbs (must be attached before takeoff)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import PX4 message types for offboard control
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

import math
from enum import Enum


class MissionState(Enum):
    """
    Enumeration of all possible states during the DARPA LIFT mission.
    This helps us track where we are in the mission sequence.
    """
    IDLE = 0              # Waiting to start
    ARMING = 1            # Arming the drone
    TAKING_OFF = 2        # Vertical takeoff to cruise altitude
    WAYPOINT_1 = 3        # Flying to 1nm waypoint (WITH payload)
    WAYPOINT_2 = 4        # Flying to 2nm waypoint (WITH payload)
    WAYPOINT_3 = 5        # Flying to 3nm waypoint (WITH payload)
    FLYING_TO_DROPOFF = 6 # Flying to 4nm dropoff point (WITH payload)
    DROPPING_PAYLOAD = 7  # Descending and releasing payload
    ASCENDING = 8         # Climbing back to cruise altitude
    WAYPOINT_4 = 9        # Flying to 5nm waypoint (WITHOUT payload)
    RETURNING_HOME = 10   # Flying back to landing zone
    LANDING = 11          # Vertical landing
    COMPLETED = 12        # Mission complete


class DARPALiftMission(Node):
    """
    ROS2 Node that controls the drone to complete the DARPA LIFT Challenge mission.
    """
    
    def __init__(self):
        super().__init__('darpa_lift_mission')
        
        # Configure QoS profile for PX4 communication
        # PX4 uses "best effort" reliability, so we match that
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ===== PUBLISHERS =====
        # These publishers send commands to PX4
        
        # Offboard control mode - tells PX4 what type of control we want
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile
        )
        
        # Trajectory setpoint - sends position/velocity commands
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )
        
        # Vehicle command - sends high-level commands (arm, takeoff, land, etc.)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )
        
        # ===== SUBSCRIBERS =====
        # These subscribers receive telemetry from PX4
        
        # Current position of the vehicle
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, 
            qos_profile
        )
        
        # Vehicle status (armed state, flight mode, etc.)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile
        )
        
        # ===== MISSION PARAMETERS =====
        # These define the DARPA LIFT Challenge course
        
        # Mission waypoints in NED coordinates (North, East, Down)
        # Down is negative, so -106.68 means UP 106.68 meters
        self.waypoints = [
            {'name': 'Takeoff', 'x': 0.0, 'y': 0.0, 'z': -106.68},           # Start
            {'name': 'Waypoint 1', 'x': 1852.0, 'y': 0.0, 'z': -106.68},     # 1 nm
            {'name': 'Waypoint 2', 'x': 3704.0, 'y': 0.0, 'z': -106.68},     # 2 nm
            {'name': 'Waypoint 3', 'x': 5556.0, 'y': 0.0, 'z': -106.68},     # 3 nm
            {'name': 'Payload Dropoff', 'x': 7408.0, 'y': 0.0, 'z': 0.0},    # 4 nm (ground level)
            {'name': 'Waypoint 4', 'x': 9260.0, 'y': 0.0, 'z': -106.68},     # 5 nm
            {'name': 'Landing', 'x': 0.0, 'y': 0.0, 'z': 0.0}                # Return home
        ]
        
        # Altitude limits (in meters, negative because NED frame)
        self.target_altitude = -106.68  # 350 feet
        self.min_altitude = -121.92     # 400 feet (max altitude limit)
        self.max_altitude = -91.44      # 300 feet (min altitude limit)
        
        # Position tolerance (how close we need to be to consider waypoint reached)
        self.position_tolerance = 5.0  # meters
        
        # Current mission state
        self.mission_state = MissionState.IDLE
        self.current_waypoint_index = 0
        
        # Vehicle telemetry (updated by callbacks)
        self.vehicle_position = None
        self.vehicle_status = None
        
        # Offboard mode counter
        # PX4 requires continuous offboard control mode messages
        self.offboard_setpoint_counter = 0
        
        # Mission timing
        self.mission_start_time = None
        self.payload_released = False
        
        # ===== MAIN CONTROL TIMER =====
        # This timer runs at 10 Hz and is the main control loop
        timer_period = 0.1  # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('DARPA LIFT Challenge Mission Node Initialized')
        self.get_logger().info('Waiting for vehicle telemetry...')
    
    # ===== CALLBACK FUNCTIONS =====
    # These are called automatically when messages arrive
    
    def vehicle_local_position_callback(self, msg):
        """
        Called when we receive new position data from PX4.
        Stores the current position for use in control logic.
        """
        self.vehicle_position = msg
    
    def vehicle_status_callback(self, msg):
        """
        Called when we receive vehicle status from PX4.
        Stores information like armed state and flight mode.
        """
        self.vehicle_status = msg
    
    # ===== CONTROL FUNCTIONS =====
    
    def publish_offboard_control_mode(self):
        """
        Publish offboard control mode.
        This tells PX4 that we want to control position.
        Must be sent continuously at >2Hz.
        """
        msg = OffboardControlMode()
        msg.position = True  # We want to control position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_trajectory_setpoint(self, x, y, z):
        """
        Publish a trajectory setpoint (target position).
        
        Args:
            x: North position in meters (NED frame)
            y: East position in meters (NED frame)  
            z: Down position in meters (NED frame, negative = up)
        """
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # Face north
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """
        Publish a vehicle command (arm, disarm, takeoff, land, etc.).
        
        Args:
            command: The command ID (from VehicleCommand constants)
            param1: First parameter
            param2: Second parameter
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def arm(self):
        """
        Arm the vehicle (enable motors).
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0  # 1.0 = arm
        )
        self.get_logger().info('Arming command sent')
    
    def disarm(self):
        """
        Disarm the vehicle (disable motors).
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=0.0  # 0.0 = disarm
        )
        self.get_logger().info('Disarming command sent')
    
    def engage_offboard_mode(self):
        """
        Switch to offboard control mode.
        This allows our software to control the drone.
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0,  # Custom mode
            param2=6.0   # Offboard mode
        )
        self.get_logger().info('Offboard mode command sent')
    
    def land(self):
        """
        Command the vehicle to land.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing command sent')
    
    def distance_to_waypoint(self, waypoint):
        """
        Calculate 3D distance to a waypoint.
        
        Args:
            waypoint: Dictionary with 'x', 'y', 'z' keys
            
        Returns:
            Distance in meters
        """
        if self.vehicle_position is None:
            return float('inf')  # Return infinity if no position data
        
        dx = waypoint['x'] - self.vehicle_position.x
        dy = waypoint['y'] - self.vehicle_position.y
        dz = waypoint['z'] - self.vehicle_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def simulate_payload_release(self):
        """
        Simulate releasing the payload.
        In real implementation, this would trigger a servo or mechanism.
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('PAYLOAD RELEASED!')
        self.get_logger().info('=' * 60)
        self.payload_released = True
    
    # ===== MAIN CONTROL LOOP =====
    
    def timer_callback(self):
        """
        Main control loop - runs at 10 Hz.
        This is the "brain" of the mission that decides what to do.
        """
        
        # Always publish offboard control mode (required for offboard control)
        self.publish_offboard_control_mode()
        
        # Check if we have position data yet
        if self.vehicle_position is None or self.vehicle_status is None:
            return  # Wait for telemetry
        
        # Increment offboard setpoint counter
        self.offboard_setpoint_counter += 1
        
        # ===== STATE MACHINE =====
        # This controls the mission flow
        
        if self.mission_state == MissionState.IDLE:
            # Initial state - prepare for mission
            if self.offboard_setpoint_counter >= 10:  # Wait 1 second for stability
                self.get_logger().info('Starting DARPA LIFT Challenge Mission!')
                self.mission_state = MissionState.ARMING
                self.mission_start_time = self.get_clock().now()
        
        elif self.mission_state == MissionState.ARMING:
            # Arm the vehicle and engage offboard mode
            self.engage_offboard_mode()
            self.arm()
            
            # Check if armed
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('Vehicle ARMED - Beginning takeoff')
                self.mission_state = MissionState.TAKING_OFF
        
        elif self.mission_state == MissionState.TAKING_OFF:
            # Vertical takeoff to cruise altitude
            wp = self.waypoints[0]  # Takeoff waypoint
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            # Check if we've reached cruise altitude
            if abs(self.vehicle_position.z - wp['z']) < self.position_tolerance:
                self.get_logger().info(f'Reached cruise altitude: {abs(wp["z"])} meters')
                self.get_logger().info('Proceeding to Waypoint 1 (1 nm)')
                self.current_waypoint_index = 1
                self.mission_state = MissionState.WAYPOINT_1
        
        elif self.mission_state == MissionState.WAYPOINT_1:
            # Fly to 1 nm waypoint
            wp = self.waypoints[1]
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            # Log progress every 5 seconds
            if self.offboard_setpoint_counter % 50 == 0:
                distance = self.distance_to_waypoint(wp)
                self.get_logger().info(f'Distance to {wp["name"]}: {distance:.1f}m')
            
            # Check if waypoint reached
            if self.distance_to_waypoint(wp) < self.position_tolerance:
                self.get_logger().info(f'Reached {wp["name"]}!')
                self.current_waypoint_index = 2
                self.mission_state = MissionState.WAYPOINT_2
        
        elif self.mission_state == MissionState.WAYPOINT_2:
            # Fly to 2 nm waypoint
            wp = self.waypoints[2]
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            if self.offboard_setpoint_counter % 50 == 0:
                distance = self.distance_to_waypoint(wp)
                self.get_logger().info(f'Distance to {wp["name"]}: {distance:.1f}m')
            
            if self.distance_to_waypoint(wp) < self.position_tolerance:
                self.get_logger().info(f'Reached {wp["name"]}!')
                self.current_waypoint_index = 3
                self.mission_state = MissionState.WAYPOINT_3
        
        elif self.mission_state == MissionState.WAYPOINT_3:
            # Fly to 3 nm waypoint
            wp = self.waypoints[3]
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            if self.offboard_setpoint_counter % 50 == 0:
                distance = self.distance_to_waypoint(wp)
                self.get_logger().info(f'Distance to {wp["name"]}: {distance:.1f}m')
            
            if self.distance_to_waypoint(wp) < self.position_tolerance:
                self.get_logger().info(f'Reached {wp["name"]}!')
                self.get_logger().info('Approaching PAYLOAD DROPOFF ZONE (4 nm)')
                self.current_waypoint_index = 4
                self.mission_state = MissionState.FLYING_TO_DROPOFF
        
        elif self.mission_state == MissionState.FLYING_TO_DROPOFF:
            # Fly to payload dropoff point
            wp = self.waypoints[4]
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            if self.offboard_setpoint_counter % 50 == 0:
                distance = self.distance_to_waypoint(wp)
                self.get_logger().info(f'Distance to DROPOFF ZONE: {distance:.1f}m')
            
            # Check if we've reached dropoff zone
            if self.distance_to_waypoint(wp) < self.position_tolerance:
                self.get_logger().info('Arrived at PAYLOAD DROPOFF ZONE')
                self.get_logger().info('Descending to release payload...')
                self.mission_state = MissionState.DROPPING_PAYLOAD
        
        elif self.mission_state == MissionState.DROPPING_PAYLOAD:
            # We're at ground level at the dropoff zone
            wp = self.waypoints[4]
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            # Release payload when stationary at ground
            if not self.payload_released:
                self.simulate_payload_release()
                self.get_logger().info('Payload released - climbing back to altitude')
                self.mission_state = MissionState.ASCENDING
        
        elif self.mission_state == MissionState.ASCENDING:
            # Climb back to cruise altitude at dropoff location
            # Target altitude at same x,y as dropoff
            x = self.waypoints[4]['x']
            y = self.waypoints[4]['y']
            z = self.target_altitude
            
            self.publish_trajectory_setpoint(x, y, z)
            
            # Check if back at cruise altitude
            if abs(self.vehicle_position.z - z) < self.position_tolerance:
                self.get_logger().info('Back at cruise altitude')
                self.get_logger().info('Proceeding to Waypoint 4 (5 nm) WITHOUT payload')
                self.current_waypoint_index = 5
                self.mission_state = MissionState.WAYPOINT_4
        
        elif self.mission_state == MissionState.WAYPOINT_4:
            # Fly to 5 nm waypoint WITHOUT payload
            wp = self.waypoints[5]
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            if self.offboard_setpoint_counter % 50 == 0:
                distance = self.distance_to_waypoint(wp)
                self.get_logger().info(f'Distance to {wp["name"]}: {distance:.1f}m')
            
            if self.distance_to_waypoint(wp) < self.position_tolerance:
                self.get_logger().info(f'Reached {wp["name"]}!')
                self.get_logger().info('Turning back - returning to landing zone')
                self.current_waypoint_index = 6
                self.mission_state = MissionState.RETURNING_HOME
        
        elif self.mission_state == MissionState.RETURNING_HOME:
            # Return to landing zone (still at altitude)
            x = 0.0
            y = 0.0
            z = self.target_altitude
            
            self.publish_trajectory_setpoint(x, y, z)
            
            # Calculate horizontal distance
            horizontal_distance = math.sqrt(
                self.vehicle_position.x**2 + self.vehicle_position.y**2
            )
            
            if self.offboard_setpoint_counter % 50 == 0:
                self.get_logger().info(f'Distance to landing zone: {horizontal_distance:.1f}m')
            
            # Check if above landing zone
            if horizontal_distance < self.position_tolerance:
                self.get_logger().info('Above landing zone - beginning descent')
                self.mission_state = MissionState.LANDING
        
        elif self.mission_state == MissionState.LANDING:
            # Descend to landing zone
            wp = self.waypoints[6]  # Landing waypoint
            self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
            
            # Check if landed (very close to ground)
            if abs(self.vehicle_position.z) < 0.5:  # Within 0.5m of ground
                self.get_logger().info('=' * 60)
                self.get_logger().info('TOUCHDOWN! Mission Complete!')
                
                # Calculate mission time
                mission_time = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
                minutes = int(mission_time // 60)
                seconds = int(mission_time % 60)
                
                self.get_logger().info(f'Total mission time: {minutes}m {seconds}s')
                self.get_logger().info('=' * 60)
                
                self.disarm()
                self.mission_state = MissionState.COMPLETED
        
        elif self.mission_state == MissionState.COMPLETED:
            # Mission finished - just maintain position
            self.get_logger().info('Mission completed successfully!', once=True)


def main(args=None):
    """
    Main function - entry point for the ROS2 node.
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the mission node
    darpa_lift_mission = DARPALiftMission()
    
    # Keep the node running
    rclpy.spin(darpa_lift_mission)
    
    # Clean up
    darpa_lift_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
