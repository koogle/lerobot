#!/usr/bin/env python3
"""
Simple test script for LeRobot SO101 follower arm movements
Tests each joint through a range of motions
"""

import sys
import os
# Add the src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import time
import numpy as np
import json
from pathlib import Path
from lerobot.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus

def create_sine_trajectory(start_pos, amplitude, frequency, duration, dt=0.01):
    """Generate smooth sine wave trajectory for testing"""
    t = np.arange(0, duration, dt)
    trajectory = start_pos + amplitude * np.sin(2 * np.pi * frequency * t)
    return trajectory

def test_individual_motors(motors_bus, test_duration=3):
    """Test each motor individually with small movements"""
    print("\n=== Testing Individual Motors ===")
    
    # Read initial positions
    initial_positions = motors_bus.sync_read("Present_Position")
    print(f"Initial positions: {initial_positions}")
    
    motor_names = list(motors_bus.motors.keys())
    
    for motor_name in motor_names:
        print(f"\nTesting {motor_name}...")
        
        # Wider range of motion (15-30 degrees)
        amplitude = 20.0  # in degrees for wider motion testing
        
        # Generate smooth trajectory
        start_pos = initial_positions[motor_name]
        trajectory = create_sine_trajectory(start_pos, amplitude, 0.5, test_duration)
        
        # Execute trajectory
        start_time = time.time()
        for position in trajectory:
            # Write position for single motor (values are already normalized)
            motors_bus.sync_write("Goal_Position", {motor_name: float(position)})
            
            # Read current position and load
            current_pos = motors_bus.read("Present_Position", motor_name)
            current_load = motors_bus.read("Present_Load", motor_name)
            
            print(f"\r{motor_name} - Pos: {current_pos:7.2f}, Load: {current_load:7.2f}", end="")
            
            time.sleep(0.01)
        
        # Return to initial position
        motors_bus.sync_write("Goal_Position", {motor_name: initial_positions[motor_name]})
        time.sleep(1)
        print(f"\n{motor_name} test complete")

def test_coordinated_motion(motors_bus, test_duration=5):
    """Test coordinated multi-motor movements"""
    print("\n=== Testing Coordinated Motion ===")
    
    # Read initial positions
    initial_positions = motors_bus.sync_read("Present_Position")
    
    # Create different frequencies for each motor for interesting pattern
    frequencies = {
        "shoulder_pan": 0.3,
        "shoulder_lift": 0.4,
        "elbow_flex": 0.5,
        "wrist_flex": 0.6,
        "wrist_roll": 0.8,
        "gripper": 0.2
    }
    
    # Wider amplitudes for coordinated motion (SO101 ranges in degrees)
    amplitudes = {
        "shoulder_pan": 25.0,
        "shoulder_lift": 20.0,
        "elbow_flex": 20.0,
        "wrist_flex": 15.0,
        "wrist_roll": 30.0,
        "gripper": 5.0  # Keep gripper small even if not testing separately
    }
    
    print("Executing coordinated motion pattern...")
    start_time = time.time()
    dt = 0.02  # 50Hz update rate
    
    while (time.time() - start_time) < test_duration:
        t = time.time() - start_time
        
        # Calculate target positions for all motors
        target_positions = {}
        for motor_name, initial_pos in initial_positions.items():
            amplitude = amplitudes.get(motor_name, 100)
            frequency = frequencies.get(motor_name, 0.5)
            target_positions[motor_name] = float(
                initial_pos + amplitude * np.sin(2 * np.pi * frequency * t)
            )
        
        # Write all positions at once
        motors_bus.sync_write("Goal_Position", target_positions)
        
        # Read feedback
        current_positions = motors_bus.sync_read("Present_Position")
        
        # Display status
        status = " | ".join([f"{name[:2]}: {pos:6.1f}" for name, pos in current_positions.items()])
        print(f"\r{status}", end="")
        
        time.sleep(dt)
    
    print("\nReturning to initial positions...")
    motors_bus.sync_write("Goal_Position", initial_positions)
    time.sleep(2)
    print("Coordinated motion test complete")

def test_gripper(motors_bus):
    """Test gripper open/close movements"""
    print("\n=== Testing Gripper ===")
    
    initial_pos = motors_bus.read("Present_Position", "gripper")
    print(f"Initial gripper position: {initial_pos}")
    
    # SO101 gripper range (0-100% for normalized gripper)
    open_position = min(initial_pos + 30.0, 100.0)
    closed_position = max(initial_pos - 30.0, 0.0)
    
    for i in range(3):
        print(f"\nGripper cycle {i+1}/3")
        
        # Open gripper
        print("Opening gripper...")
        motors_bus.sync_write("Goal_Position", {"gripper": open_position})
        time.sleep(1.5)
        
        # Close gripper
        print("Closing gripper...")
        motors_bus.sync_write("Goal_Position", {"gripper": closed_position})
        time.sleep(1.5)
    
    # Return to initial
    motors_bus.sync_write("Goal_Position", {"gripper": initial_pos})
    time.sleep(1)
    print("Gripper test complete")

def robot_dance(motors_bus, dance_duration=10):
    """Make the robot do a fun dance!"""
    print("\n=== Robot Dance Time! 🎵 ===")
    
    # Read initial positions
    initial_positions = motors_bus.sync_read("Present_Position")
    
    print("Starting dance routine...")
    start_time = time.time()
    dt = 0.02  # 50Hz update rate
    
    while (time.time() - start_time) < dance_duration:
        t = time.time() - start_time
        
        # Create fun dance movements with different patterns
        target_positions = {}
        
        # Wave motion for shoulder pan (side to side)
        target_positions["shoulder_pan"] = initial_positions["shoulder_pan"] + \
            20.0 * np.sin(2 * np.pi * 0.5 * t)
        
        # Nodding motion for shoulder lift
        target_positions["shoulder_lift"] = initial_positions["shoulder_lift"] + \
            15.0 * np.sin(2 * np.pi * 0.8 * t + np.pi/4)
        
        # Elbow pumping
        target_positions["elbow_flex"] = initial_positions["elbow_flex"] + \
            15.0 * np.sin(2 * np.pi * 1.0 * t)
        
        # Wrist wave
        target_positions["wrist_flex"] = initial_positions["wrist_flex"] + \
            10.0 * np.sin(2 * np.pi * 1.5 * t + np.pi/2)
        
        # Wrist roll disco move
        target_positions["wrist_roll"] = initial_positions["wrist_roll"] + \
            25.0 * np.sin(2 * np.pi * 0.3 * t)
        
        # Gripper open/close rhythm
        target_positions["gripper"] = initial_positions["gripper"] + \
            5.0 * (1 + np.sin(2 * np.pi * 2.0 * t))
        
        # Add some beat drops every 2 seconds
        if int(t) % 2 == 0 and (t - int(t)) < 0.1:
            # Quick shoulder drop
            target_positions["shoulder_lift"] -= 5.0
        
        # Write all positions at once
        motors_bus.sync_write("Goal_Position", target_positions)
        
        # Display dance progress
        dance_bar = "🎵" + "=" * int((t / dance_duration) * 20) + ">" + " " * (20 - int((t / dance_duration) * 20)) + "🎵"
        print(f"\rDancing: {dance_bar} {int(dance_duration - t)}s", end="")
        
        time.sleep(dt)
    
    print("\n\nFinishing dance with a bow...")
    
    # Bow motion (subtle)
    bow_positions = initial_positions.copy()
    bow_positions["shoulder_lift"] = initial_positions["shoulder_lift"] + 15.0
    bow_positions["elbow_flex"] = initial_positions["elbow_flex"] + 10.0
    motors_bus.sync_write("Goal_Position", bow_positions)
    time.sleep(1.5)
    
    # Return to initial positions
    print("Returning to initial positions...")
    motors_bus.sync_write("Goal_Position", initial_positions)
    time.sleep(2)
    print("Dance complete! 🎉")

def check_motor_status(motors_bus):
    """Check and display current status of all motors"""
    print("\n=== Motor Status Check ===")
    
    # Read various parameters
    positions = motors_bus.sync_read("Present_Position")
    loads = motors_bus.sync_read("Present_Load")
    voltages = motors_bus.sync_read("Present_Voltage")
    temperatures = motors_bus.sync_read("Present_Temperature")
    
    print("\nMotor Status:")
    print("-" * 60)
    print(f"{'Motor':<15} {'Position':>8} {'Load':>8} {'Voltage':>8} {'Temp':>8}")
    print("-" * 60)
    
    for motor_name in motors_bus.motors.keys():
        print(f"{motor_name:<15} {positions[motor_name]:>8.2f} "
              f"{loads[motor_name]:>8.2f} {voltages[motor_name]:>8.1f}V "
              f"{temperatures[motor_name]:>8}°C")

def main():
    """Main test routine"""
    print("LeRobot SO101 Follower Arm Test Script")
    print("=======================================")
    
    # Configuration for SO101 follower arm
    # Update the port to match your system (e.g., /dev/ttyACM0, /dev/ttyUSB0, COM3)
    # Follower
    port = "/dev/tty.usbmodem5A7A0547071"  # UPDATE THIS TO YOUR PORT
    # Leader
    # port = "/dev/tty.usbmodem5A7A0590501"

    
    # Load calibration data
    calibration_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/jakob_follower_arm.json"
    calibration = {}
    
    if calibration_path.exists():
        print(f"\nLoading calibration from: {calibration_path}")
        with open(calibration_path) as f:
            calibration_data = json.load(f)
            for motor_name, cal_data in calibration_data.items():
                calibration[motor_name] = MotorCalibration(
                    id=cal_data["id"],
                    drive_mode=cal_data["drive_mode"],
                    homing_offset=cal_data["homing_offset"],
                    range_min=cal_data["range_min"],
                    range_max=cal_data["range_max"]
                )
        print("Calibration loaded successfully!")
    else:
        print(f"\nWARNING: No calibration found at {calibration_path}")
        print("Running without calibration - be careful!")
    
    # Motor configuration matching SO101 follower arm
    motors = {
        "shoulder_pan": Motor(id=1, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100),
        "shoulder_lift": Motor(id=2, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100),
        "elbow_flex": Motor(id=3, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100),
        "wrist_flex": Motor(id=4, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100),
        "wrist_roll": Motor(id=5, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100),
        "gripper": Motor(id=6, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
    }
    
    # Create motor bus instance with calibration
    motors_bus = FeetechMotorsBus(port=port, motors=motors, calibration=calibration)
    
    try:
        # Connect to motors
        print("\nConnecting to motors...")
        motors_bus.connect()
        print("Successfully connected!")
        
        # Check initial status
        check_motor_status(motors_bus)
        
        # Safety prompt
        input("\nWARNING: Testing with wider range of motion!\nPress Enter to start motor tests (ensure area is clear)...")
        
        # Run tests
        test_individual_motors(motors_bus, test_duration=3)
        time.sleep(1)
        
        # test_coordinated_motion(motors_bus, test_duration=8)  # Replaced by dance
        # time.sleep(1)
        
        # test_gripper(motors_bus)  # Skipping gripper test
        
        # Robot dance!
        input("\nPress Enter to see the robot dance! 🎵")
        robot_dance(motors_bus, dance_duration=10)
        
        # Final status check
        check_motor_status(motors_bus)
        
        print("\n✓ All tests completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always disconnect
        if motors_bus.is_connected:
            print("\nDisconnecting from motors...")
            motors_bus.disconnect()
            print("Disconnected")

if __name__ == "__main__":
    main()

