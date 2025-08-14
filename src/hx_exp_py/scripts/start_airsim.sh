#!/bin/bash

# HX AirSim Environment Startup Script

echo "Starting HX AirSim Environment..."

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
AIRSIM_DIR="$PROJECT_DIR/airsim_env"

# Copy settings to home directory (AirSim looks for it there)
echo "Preparing AirSim settings..."
mkdir -p ~/Documents/AirSim

# Backup existing settings.json if it exists
if [ -f ~/Documents/AirSim/settings.json ]; then
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    BACKUP_FILE="~/Documents/AirSim/settings.json.backup_${TIMESTAMP}"
    echo "Backing up existing settings.json to ${BACKUP_FILE}"
    cp ~/Documents/AirSim/settings.json "$BACKUP_FILE"
fi

echo "Copying new AirSim settings..."
cp "$AIRSIM_DIR/settings.json" ~/Documents/AirSim/

# Set display if needed (for headless systems)
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# Change to AirSim directory
cd "$AIRSIM_DIR/LinuxNoEditor"

# Make sure the executable is executable
chmod +x Blocks.sh
chmod +x Blocks/Binaries/Linux/Blocks

echo "Launching AirSim Blocks Environment..."
echo "AirSim will start in 3 seconds..."
sleep 3

# Launch AirSim
./Blocks.sh -ResX=1280 -ResY=720 -windowed

echo "AirSim has exited."