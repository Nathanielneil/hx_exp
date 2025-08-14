#!/usr/bin/env python3

"""
Simple AirSim Test Script (Standalone)
This script tests AirSim connection without ROS dependencies
"""

import airsim
import time
import sys

def test_airsim_connection():
    """Test basic AirSim connection and vehicle control"""
    
    print("=" * 50)
    print("Simple AirSim Connection Test")
    print("=" * 50)
    
    try:
        # Connect to AirSim
        print("Connecting to AirSim...")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✓ Successfully connected to AirSim!")
        
        # Enable API control
        print("Enabling API control...")
        client.enableApiControl(True)
        client.armDisarm(True)
        print("✓ API control enabled and vehicle armed!")
        
        # Get current state
        state = client.getMultirotorState()
        print(f"✓ Vehicle position: {state.kinematics_estimated.position}")
        print(f"✓ Vehicle ready: {state.ready}")
        
        # Test basic takeoff
        print("\nTesting takeoff...")
        takeoff_result = client.takeoffAsync(timeout_sec=5)
        takeoff_result.join()
        print("✓ Takeoff completed!")
        
        # Hover for a few seconds
        print("Hovering for 3 seconds...")
        time.sleep(3)
        
        # Get position after takeoff
        state = client.getMultirotorState()
        print(f"✓ Position after takeoff: {state.kinematics_estimated.position}")
        
        # Land
        print("Landing...")
        land_result = client.landAsync(timeout_sec=5)
        land_result.join()
        print("✓ Landing completed!")
        
        # Disable API control
        client.armDisarm(False)
        client.enableApiControl(False)
        print("✓ API control disabled")
        
        print("\n" + "=" * 50)
        print("✓ All tests passed! AirSim is working correctly.")
        print("✓ You can now proceed with ROS integration.")
        print("=" * 50)
        
        return True
        
    except Exception as e:
        print(f"✗ Error: {str(e)}")
        print("\nPossible issues:")
        print("1. AirSim is not running")
        print("2. AirSim settings.json is not configured correctly")
        print("3. Python airsim package is not installed")
        print("\nTo fix:")
        print("1. Start AirSim: ./scripts/start_airsim.sh")
        print("2. Install airsim: pip install airsim")
        return False

if __name__ == "__main__":
    success = test_airsim_connection()
    sys.exit(0 if success else 1)