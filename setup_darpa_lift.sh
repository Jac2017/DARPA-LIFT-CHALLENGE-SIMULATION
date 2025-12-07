#!/bin/bash

################################################################################
# DARPA LIFT Challenge Setup Script
# 
# This script automates the installation of the mission map and files for
# the DARPA LIFT Challenge simulation.
#
# What it does:
# 1. Checks if required directories exist
# 2. Copies the world file to PX4
# 3. Sets up the ROS2 mission node
# 4. Builds the ROS2 workspace
# 5. Verifies installation
################################################################################

# Colors for output (makes it easier to read)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to print section headers
print_header() {
    echo ""
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}"
}

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Define key paths
PX4_DIR="$HOME/PX4-Autopilot"
ROS2_WS_DIR="$HOME/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2"
WORLD_FILE="$SCRIPT_DIR/darpa_lift_challenge.sdf"
MISSION_FILE="$SCRIPT_DIR/darpa_lift_mission.py"

################################################################################
# START OF INSTALLATION
################################################################################

print_header "DARPA LIFT Challenge Setup"
echo "This script will set up your DARPA LIFT Challenge simulation environment."
echo ""

################################################################################
# STEP 1: Check Prerequisites
################################################################################

print_header "Step 1: Checking Prerequisites"

# Check if PX4 directory exists
if [ ! -d "$PX4_DIR" ]; then
    print_error "PX4-Autopilot directory not found at: $PX4_DIR"
    print_info "Please run the installation script first or check your PX4 installation."
    exit 1
fi
print_success "Found PX4-Autopilot at: $PX4_DIR"

# Check if ROS2 workspace exists
if [ ! -d "$ROS2_WS_DIR" ]; then
    print_error "ROS2 workspace not found at: $ROS2_WS_DIR"
    print_info "Please check your workspace location."
    exit 1
fi
print_success "Found ROS2 workspace at: $ROS2_WS_DIR"

# Check if world file exists
if [ ! -f "$WORLD_FILE" ]; then
    print_error "World file not found at: $WORLD_FILE"
    print_info "Please ensure darpa_lift_challenge.sdf is in the same directory as this script."
    exit 1
fi
print_success "Found world file: $WORLD_FILE"

# Check if mission file exists
if [ ! -f "$MISSION_FILE" ]; then
    print_error "Mission file not found at: $MISSION_FILE"
    print_info "Please ensure darpa_lift_mission.py is in the same directory as this script."
    exit 1
fi
print_success "Found mission file: $MISSION_FILE"

################################################################################
# STEP 2: Install World File
################################################################################

print_header "Step 2: Installing World File"

# Create worlds directory if it doesn't exist
WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
if [ ! -d "$WORLDS_DIR" ]; then
    print_warning "Worlds directory doesn't exist. Creating it..."
    mkdir -p "$WORLDS_DIR"
fi

# Copy world file
print_info "Copying world file to PX4..."
cp "$WORLD_FILE" "$WORLDS_DIR/"

# Verify copy was successful
if [ -f "$WORLDS_DIR/darpa_lift_challenge.sdf" ]; then
    print_success "World file installed successfully!"
else
    print_error "Failed to copy world file."
    exit 1
fi

################################################################################
# STEP 3: Install Mission Node
################################################################################

print_header "Step 3: Installing Mission Node"

# Check if my_offboard_ctrl package exists, if not create it
PKG_DIR="$ROS2_WS_DIR/src/my_offboard_ctrl"

if [ ! -d "$PKG_DIR" ]; then
    print_warning "Package 'my_offboard_ctrl' doesn't exist. Creating it..."
    
    # Navigate to src directory
    cd "$ROS2_WS_DIR/src" || exit 1
    
    # Create ROS2 package
    ros2 pkg create --build-type ament_python my_offboard_ctrl --dependencies rclpy px4_msgs
    
    if [ $? -eq 0 ]; then
        print_success "Package created successfully!"
    else
        print_error "Failed to create package."
        exit 1
    fi
fi

# Copy mission file to package
print_info "Copying mission file to package..."
cp "$MISSION_FILE" "$PKG_DIR/"

# Make script executable
chmod +x "$PKG_DIR/darpa_lift_mission.py"

# Update setup.py to include the new script
SETUP_PY="$PKG_DIR/setup.py"

# Check if entry point already exists
if grep -q "darpa_lift_mission" "$SETUP_PY"; then
    print_info "Entry point already exists in setup.py"
else
    print_info "Adding entry point to setup.py..."
    
    # This is a simple approach - in production you might want more robust parsing
    # We'll add it before the closing bracket of entry_points
    sed -i "/entry_points={/a\\        'console_scripts': [\n            'darpa_lift_mission = my_offboard_ctrl.darpa_lift_mission:main',\n        ]," "$SETUP_PY"
    
    print_success "Updated setup.py"
fi

################################################################################
# STEP 4: Build ROS2 Workspace
################################################################################

print_header "Step 4: Building ROS2 Workspace"

cd "$ROS2_WS_DIR" || exit 1

print_info "Building workspace... (this may take a moment)"

# Build only the package we need
colcon build --packages-select my_offboard_ctrl

if [ $? -eq 0 ]; then
    print_success "Workspace built successfully!"
else
    print_error "Build failed. Please check the error messages above."
    exit 1
fi

################################################################################
# STEP 5: Create Launch Helper Script
################################################################################

print_header "Step 5: Creating Launch Helper Scripts"

# Create a convenient launch script
LAUNCH_SCRIPT="$HOME/launch_darpa_lift.sh"

cat > "$LAUNCH_SCRIPT" << 'EOF'
#!/bin/bash

# DARPA LIFT Challenge Quick Launch Script
# This script launches PX4 with the DARPA LIFT world

echo "Launching DARPA LIFT Challenge Simulation..."
echo ""
echo "After this starts, in separate terminals run:"
echo "  Terminal 1: MicroXRCEAgent udp4 -p 8888"
echo "  Terminal 2: This script (already running)"
echo "  Terminal 3: cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2 && source install/local_setup.bash && ros2 run my_offboard_ctrl darpa_lift_mission"
echo ""

# Launch PX4 with DARPA LIFT world
PX4_SYS_AUTOSTART=4010 \
PX4_SIM_MODEL=gz_x500_mono_cam \
PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" \
PX4_GZ_WORLD=darpa_lift_challenge \
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
EOF

chmod +x "$LAUNCH_SCRIPT"
print_success "Created launch script: $LAUNCH_SCRIPT"

# Create mission launch script
MISSION_LAUNCH="$HOME/run_darpa_mission.sh"

cat > "$MISSION_LAUNCH" << EOF
#!/bin/bash

# DARPA LIFT Challenge Mission Runner
# This script runs the mission planner

cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2
source install/local_setup.bash
ros2 run my_offboard_ctrl darpa_lift_mission
EOF

chmod +x "$MISSION_LAUNCH"
print_success "Created mission script: $MISSION_LAUNCH"

################################################################################
# STEP 6: Verification
################################################################################

print_header "Step 6: Verification"

print_info "Checking installation..."

# Check world file
if [ -f "$WORLDS_DIR/darpa_lift_challenge.sdf" ]; then
    print_success "✓ World file installed"
else
    print_error "✗ World file missing"
fi

# Check mission file
if [ -f "$PKG_DIR/darpa_lift_mission.py" ]; then
    print_success "✓ Mission file installed"
else
    print_error "✗ Mission file missing"
fi

# Check if built
if [ -d "$ROS2_WS_DIR/install/my_offboard_ctrl" ]; then
    print_success "✓ ROS2 package built"
else
    print_error "✗ ROS2 package not built"
fi

################################################################################
# FINAL INSTRUCTIONS
################################################################################

print_header "Installation Complete!"

echo ""
echo "Your DARPA LIFT Challenge simulation is ready!"
echo ""
echo "To run the simulation:"
echo ""
echo "  ${GREEN}Terminal 1 - Start the DDS Agent:${NC}"
echo "    MicroXRCEAgent udp4 -p 8888"
echo ""
echo "  ${GREEN}Terminal 2 - Launch PX4 and Gazebo:${NC}"
echo "    $LAUNCH_SCRIPT"
echo "  ${YELLOW}OR manually:${NC}"
echo "    PX4_GZ_WORLD=darpa_lift_challenge ~/PX4-Autopilot/build/px4_sitl_default/bin/px4"
echo ""
echo "  ${GREEN}Terminal 3 - Run the mission:${NC}"
echo "    $MISSION_LAUNCH"
echo "  ${YELLOW}OR manually:${NC}"
echo "    cd ~/DARPA-LIFT-CHALLENGE-SIMULATION/ws_ros2"
echo "    source install/local_setup.bash"
echo "    ros2 run my_offboard_ctrl darpa_lift_mission"
echo ""
echo "  ${GREEN}Optional - Launch QGroundControl:${NC}"
echo "    ./QGroundControl.AppImage"
echo ""
echo "${BLUE}For more information, see README_DARPA_LIFT.md${NC}"
echo ""

print_success "Setup complete! Happy flying! 🚁"
