#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import tty
import termios
from dynamixel_sdk import *
import time # Import for time.sleep

def getch():
    """
    Reads a single character from standard input without requiring the user to press Enter.
    Used for simple keyboard commands.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class DynamixelController:
    """
    A controller class for Dynamixel motors, supporting both Position Control and PWM Control modes.
    It includes functionalities for setting modes, writing goals, reading feedback, and waiting for
    motors to reach targets or specific load conditions.
    """
    def __init__(self, ids, z_axis_id, z_up_position, z_down_pwm, position_limits=None, device_name='/dev/ttyUSB0', baudrate=1_000_000, protocol_version=2.0):
        """
        Initializes the DynamixelController.

        Args:
            ids (list): A list of Dynamixel IDs to control.
            z_axis_id (int): The ID of the Z-axis motor.
            z_up_position (int): The predefined goal position for the Z-axis "up" state.
            z_down_pwm (int): The PWM value to apply for moving the Z-axis "down".
            position_limits (list, optional): A list of (min, max) position tuples for each motor ID.
                                              Defaults to [(1100, 2900)] for all motors if None.
            device_name (str, optional): The serial port device name. Defaults to '/dev/ttyUSB0'.
            baudrate (int, optional): The communication baud rate. Defaults to 1,000,000.
            protocol_version (float, optional): The Dynamixel protocol version. Defaults to 2.0.
        """
        assert isinstance(ids, list), "Motor IDs must be a list."
        assert z_axis_id in ids, "Z-axis ID must be one of the provided motor IDs."
        self.dxl_ids = ids
        self.z_axis_id = z_axis_id
        self.z_up_position = z_up_position
        self.z_down_pwm = z_down_pwm
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        # Control Table Addresses (for X-series Dynamixels)
        self.ADDR_OPERATING_MODE = 11      # Operating Mode (1 byte)
        self.ADDR_TORQUE_ENABLE = 64       # Torque Enable (1 byte)
        self.ADDR_GOAL_POSITION = 116      # Goal Position (4 bytes)
        self.ADDR_PRESENT_POSITION = 132   # Present Position (4 bytes)
        self.ADDR_PRESENT_VELOCITY = 128   # Present Velocity (4 bytes)
        self.ADDR_PRESENT_LOAD = 126       # Present Load (2 bytes) - NEW: Read present load/current
        self.ADDR_GOAL_PWM = 100           # Goal PWM (2 bytes)
        self.ADDR_PWM_LIMIT = 36           # PWM Limit (2 bytes)
        self.ADDR_PROFILE_ACCELERATION = 108 # Profile Acceleration (4 bytes)
        self.ADDR_PROFILE_VELOCITY = 112   # Profile Velocity (4 bytes)

        # Operational Values
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.MOVING_THRESHOLD_POSITION = 10 # Position tolerance for position control
        self.STOPPED_VELOCITY_THRESHOLD = 5 # Velocity threshold for "stopped" in PWM mode (less relevant now)
        self.STOPPED_LOAD_THRESHOLD = 18    # NEW: Load threshold to indicate "reached" in PWM mode

        # Operating Modes
        self.OPERATING_MODE_POSITION = 3 # Position Control Mode
        self.OPERATING_MODE_PWM = 16     # PWM Control Mode

        # Default motion range for non-Z motors
        if position_limits is None:
            self.position_limits = [(1100, 2900)] * len(ids)
        else:
            assert len(position_limits) == len(ids), "position_limits must match the number of IDs"
            self.position_limits = position_limits

        self.position_indices = [0] * len(ids) # Not directly used in this version, but kept for compatibility
        self.current_modes = {dxl_id: self.OPERATING_MODE_POSITION for dxl_id in self.dxl_ids} # Track current mode of each motor

        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        self._open_port()
        self._initial_setup()

    def _open_port(self):
        """Opens the serial port for communication with Dynamixels."""
        if not self.portHandler.openPort():
            print("Failed to open port")
            sys.exit(1)
        if not self.portHandler.setBaudRate(self.baudrate):
            print("Failed to set baudrate")
            sys.exit(1)
        print("Port opened and baudrate set")

    def _initial_setup(self):
        """
        Performs initial setup for all motors: disables torque, sets initial mode to Position Control,
        sets profile acceleration/velocity, sets PWM limit, and re-enables torque.
        """
        print("\n--- Performing initial setup for all motors ---")
        for dxl_id in self.dxl_ids:
            # Always start by disabling torque to change operating mode safely
            self.set_torque_enable(dxl_id, self.TORQUE_DISABLE)
            
            # Set to Position Control mode initially for all motors
            self.set_operating_mode(dxl_id, self.OPERATING_MODE_POSITION) 
            
            # Set profile acceleration and velocity for position control
            # These values affect how quickly the motor accelerates/decelerates to reach the goal position.
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_ACCELERATION, 10)
            self._check_comm(result, error, f"Setting profile acceleration for ID {dxl_id}")
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_VELOCITY, 100)
            self._check_comm(result, error, f"Setting profile velocity for ID {dxl_id}")
            
            # Set a default PWM limit for safety (max 885 for XM/XW series)
            self.set_pwm_limit(dxl_id, 885) 
            
            # Re-enable torque
            self.set_torque_enable(dxl_id, self.TORQUE_ENABLE)
        print("--- Initial setup complete ---")

    def _check_comm(self, result, error, context=""):
        """
        Checks the communication result and packet error, printing appropriate messages.

        Args:
            result (int): The communication result code from PacketHandler.
            error (int): The packet error code from PacketHandler.
            context (str, optional): A descriptive string for the operation being checked.
        """
        if result != COMM_SUCCESS:
            print(f"ERROR: {context}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"ERROR: {context}: {self.packetHandler.getRxPacketError(error)}")

    def set_torque_enable(self, dxl_id, enable):
        """
        Enables or disables torque for a specified Dynamixel motor.

        Args:
            dxl_id (int): The ID of the Dynamixel motor.
            enable (int): 1 to enable torque, 0 to disable torque.
        """
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, enable)
        self._check_comm(result, error, f"Setting torque {'enable' if enable else 'disable'} for ID {dxl_id}")

    def set_operating_mode(self, dxl_id, mode):
        """
        Sets the operating mode for a specified Dynamixel motor.
        Torque is temporarily disabled before changing mode and re-enabled afterward.

        Args:
            dxl_id (int): The ID of the Dynamixel motor.
            mode (int): The operating mode value (e.g., self.OPERATING_MODE_POSITION).
        """
        # Torque must be disabled to change operating mode
        self.set_torque_enable(dxl_id, self.TORQUE_DISABLE)
        
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_OPERATING_MODE, mode)
        self._check_comm(result, error, f"Setting operating mode to {mode} for ID {dxl_id}")
        if result == COMM_SUCCESS and error == 0:
            self.current_modes[dxl_id] = mode
            mode_name = 'Position Control' if mode == self.OPERATING_MODE_POSITION else \
                        'PWM Control' if mode == self.OPERATING_MODE_PWM else \
                        'Unknown Mode'
            print(f"ID {dxl_id} operating mode set to: {mode_name}")
        
        # Re-enable torque after changing mode
        self.set_torque_enable(dxl_id, self.TORQUE_ENABLE)

    def set_pwm_limit(self, dxl_id, limit):
        """
        Sets the PWM limit for a specified Dynamixel motor. This limits the maximum PWM value.

        Args:
            dxl_id (int): The ID of the Dynamixel motor.
            limit (int): The PWM limit value (0-885).
        """
        # PWM Limit is a 2-byte value. Max 885 for X-series.
        result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_PWM_LIMIT, limit)
        self._check_comm(result, error, f"Setting PWM limit to {limit} for ID {dxl_id}")

    def write_positions(self, dxl_ids_to_move, target_positions):
        """
        Writes goal positions for multiple Dynamixel motors.
        Only applies to motors currently in Position Control Mode.

        Args:
            dxl_ids_to_move (list): A list of Dynamixel IDs to command.
            target_positions (list): A list of corresponding goal positions.
        """
        assert len(dxl_ids_to_move) == len(target_positions), "Length of dxl_ids_to_move and target_positions must match."
        for i, dxl_id in enumerate(dxl_ids_to_move):
            if self.current_modes[dxl_id] != self.OPERATING_MODE_POSITION:
                print(f"WARNING: ID {dxl_id} is not in Position Control Mode. Skipping position write.")
                continue
            goal_pos = target_positions[i]
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
            self._check_comm(result, error, f"Writing goal position {goal_pos} for ID {dxl_id}")
    
    def write_pwms(self, dxl_ids_to_move, target_pwms):
        """
        Writes goal PWMs for multiple Dynamixel motors.
        Only applies to motors currently in PWM Control Mode.

        Args:
            dxl_ids_to_move (list): A list of Dynamixel IDs to command.
            target_pwms (list): A list of corresponding goal PWM values.
        """
        assert len(dxl_ids_to_move) == len(target_pwms), "Length of dxl_ids_to_move and target_pwms must match."
        for i, dxl_id in enumerate(dxl_ids_to_move):
            if self.current_modes[dxl_id] != self.OPERATING_MODE_PWM:
                print(f"WARNING: ID {dxl_id} is not in PWM Control Mode. Skipping PWM write.")
                continue
            goal_pwm = target_pwms[i]
            # Goal PWM is a 2-byte signed value. Range: -885 to 885.
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_PWM, goal_pwm)
            self._check_comm(result, error, f"Writing goal PWM {goal_pwm} for ID {dxl_id}")

    def read_position(self, dxl_id):
        """
        Reads the present position of a Dynamixel motor.

        Args:
            dxl_id (int): The ID of the Dynamixel motor.

        Returns:
            int: The present position value.
        """
        present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
        self._check_comm(result, error, f"Reading position for ID {dxl_id}")
        return present_pos
    
    def read_velocity(self, dxl_id):
        """
        Reads the present velocity of a Dynamixel motor.

        Args:
            dxl_id (int): The ID of the Dynamixel motor.

        Returns:
            int: The present velocity value (signed).
        """
        present_vel, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_VELOCITY)
        self._check_comm(result, error, f"Reading velocity for ID {dxl_id}")
        # Velocity is signed, so abs() for threshold check if needed elsewhere
        return present_vel

    def read_load(self, dxl_id):
        """
        Reads the present load/current of a Dynamixel motor.

        Args:
            dxl_id (int): The ID of the Dynamixel motor.

        Returns:
            int: The present load value (signed, 0-2047, where 1024 indicates no load,
                 1025-2047 is one direction, 0-1023 is the other).
                 For practical purposes, we often use the absolute deviation from 1024 or just the raw value.
                 Here, we return the raw value, its interpretation depends on the motor model.
                 For X-series, 0-2047, 0-1023 is CCW, 1024-2047 is CW. The magnitude represents load.
        """
        present_load, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_LOAD)
        self._check_comm(result, error, f"Reading load for ID {dxl_id}")
        #invert the load bites
        # Dynamixel X-series motors report load as a 2-byte value.
        # The load is 
        print(f"ID {dxl_id} present load: {present_load}")
        # Dynamixel X-series motors report load as a 2-byte value.
        # It's a signed value, but represented from 0-2047.
        # 0-1023 (0x000-0x3FF) represents current in one direction (e.g., CCW), increasing with load.
        # 1024-2047 (0x400-0x7FF) represents current in the opposite direction (e.g., CW), increasing with load.
        # The user's request "over 18" implies checking the magnitude.
        # We will return the raw value, and its magnitude will be checked against the threshold.
        # If it's over 1024, convert it to a negative value to represent the other direction for consistent abs() check.
        if present_load > 1023:
            present_load=abs(present_load -2**16 ) # Mask to ensure we only get the lower 10 bits

        return present_load

    def wait_until_reached(self, dxl_ids_to_wait, target_positions):
        """
        Waits for specified motors to reach their target positions if in Position Control Mode.
        If a motor is in PWM Control Mode, it will call wait_until_load_threshold instead
        to check if it has met resistance (reached position).

        Args:
            dxl_ids_to_wait (list): A list of Dynamixel IDs to wait for.
            target_positions (list): A list of corresponding target positions (used only for Position Control).
        """
        assert len(dxl_ids_to_wait) == len(target_positions), "Length of dxl_ids_to_wait and target_positions must match."

        for i, dxl_id in enumerate(dxl_ids_to_wait):
            if self.current_modes[dxl_id] == self.OPERATING_MODE_POSITION:
                goal_pos = target_positions[i]
                print(f"Waiting for ID {dxl_id} to reach {goal_pos} (Position Control)...")
                while True:
                    present_pos = self.read_position(dxl_id)
                    if abs(goal_pos - present_pos) <= self.MOVING_THRESHOLD_POSITION:
                        print(f"ID {dxl_id} reached target position.")
                        break
                    time.sleep(0.01) # Small delay to prevent busy-waiting
            elif self.current_modes[dxl_id] == self.OPERATING_MODE_PWM:
                # If a motor is passed here that's in PWM mode, it means we want to wait for it to stop/hit something
                print(f"ID {dxl_id} is in PWM Control Mode. Waiting for load to be above threshold.")
                self.wait_until_load_threshold([dxl_id], self.STOPPED_LOAD_THRESHOLD)
            else:
                print(f"WARNING: ID {dxl_id} is in an unknown mode. Skipping wait.")

    def wait_until_load_threshold(self, dxl_ids_to_wait, threshold):
        """
        NEW: Waits for the present load of specified motors to be above a given threshold.
        This is useful for checking if a motor in PWM mode has encountered significant resistance,
        indicating it has reached a physical stop or picked up an object.

        Args:
            dxl_ids_to_wait (list): A list of Dynamixel IDs to wait for.
            threshold (int): The load threshold. The motor is considered "reached" when
                             its absolute present load value is >= this threshold.
        """
        assert isinstance(dxl_ids_to_wait, list), "dxl_ids_to_wait must be a list."
        print(f"Waiting for motors {dxl_ids_to_wait} to have absolute load >= {threshold}...")
        for dxl_id in dxl_ids_to_wait:
            if self.current_modes[dxl_id] != self.OPERATING_MODE_PWM:
                print(f"WARNING: ID {dxl_id} is not in PWM Control Mode. Cannot wait for load threshold. Skipping.")
                continue

            print(f"Waiting for ID {dxl_id} load to be >= {threshold}...")
            while True:
                present_load = self.read_load(dxl_id)
                # Check if the absolute load is greater than or equal to the threshold
                if abs(present_load) >= threshold:
                    print(f"ID {dxl_id} load ({present_load}) is above threshold ({threshold}). Assumed reached position.")
                    break
                print(f"ID {dxl_id} current load: {present_load}. Waiting...")
                time.sleep(0.01) # Small delay to prevent busy-waiting

    def move_z_up(self):
        """Moves the Z-axis motor to its predefined 'up' position using Position Control."""
        print(f"\n--- Moving Z-axis (ID: {self.z_axis_id}) UP to {self.z_up_position} (Position Control) ---")
        self.set_operating_mode(self.z_axis_id, self.OPERATING_MODE_POSITION)
        self.write_positions([self.z_axis_id], [self.z_up_position])
        self.wait_until_reached([self.z_axis_id], [self.z_up_position])
        print(f"Z-axis (ID: {self.z_axis_id}) is UP.")

    def move_z_down(self):
        """
        Moves the Z-axis motor 'down' using PWM Control until a specified load threshold is met.
        This simulates moving until it hits a surface or object.
        """
        print(f"\n--- Moving Z-axis (ID: {self.z_axis_id}) DOWN with PWM {self.z_down_pwm} (PWM Control) ---")
        self.set_operating_mode(self.z_axis_id, self.OPERATING_MODE_PWM)
        # Set a reasonable PWM limit for safety if not already done
        self.set_pwm_limit(self.z_axis_id, 800) # Example: set a limit for the Z-axis
        self.write_pwms([self.z_axis_id], [self.z_down_pwm])
        print(f"Z-axis (ID: {self.z_axis_id}) is moving DOWN with PWM {self.z_down_pwm}.")
        print("Waiting for Z-axis to reach position (load threshold)...")
        # Use the new wait_until_load_threshold for PWM controlled movement
        self.wait_until_load_threshold([self.z_axis_id], self.STOPPED_LOAD_THRESHOLD)
        print(f"Z-axis motor load indicates it has reached position (load >= {self.STOPPED_LOAD_THRESHOLD}).")
        # Optionally, stop the motor after reaching load threshold by setting PWM to 0
        self.write_pwms([self.z_axis_id], [0])
        print("Z-axis motor stopped after reaching load threshold.")


    def disable_torque_and_close(self):
        """Disables torque for all motors and closes the serial port."""
        print("\n--- Disabling torque for all motors and closing port ---")
        for dxl_id in self.dxl_ids:
            self.set_torque_enable(dxl_id, self.TORQUE_DISABLE)
        self.portHandler.closePort()
        print("Port closed")

if __name__ == "__main__":
    # Define Dynamixel IDs. Motor 3 is designated as the Z-axis motor for this example.
    dxl_ids = [1, 2, 3] 
    z_axis_id = 3
    z_up_position = 2800 # Predefined position for Z-axis "up" (e.g., home position)
    z_down_pwm = 30      # PWM value for Z-axis "down" (positive value for one direction, adjust as needed)

    # Individual limits for non-Z motors (Motor 1, 2)
    # The Z-axis motor (ID 3) will use its specific z_up_position for "up" movements
    # and PWM for "down" movements, where limits are less relevant for the 'reached' condition.
    limits = [(1100, 3000), (1100, 3000), (0, 0)] # Last element for Z-axis, its limits aren't used for "up"
    
    controller = DynamixelController(dxl_ids, z_axis_id, z_up_position, z_down_pwm, position_limits=limits)

    # Initial setup for all motors (they will initially be in Position Control mode)
    print("\n--- Initializing all motors to a safe position (Position Control) ---")
    initial_positions = [2000, 2000, 2000] # Set an initial position for the Z-axis as well
    controller.write_positions(dxl_ids, initial_positions)
    # When calling wait_until_reached with dxl_ids, it will properly handle the Z-axis
    # (waiting for position if still in position mode, or immediately skipping if already in PWM mode for some reason).
    controller.wait_until_reached(dxl_ids, initial_positions)
    
    print("\nPress 'U' to move Z-axis UP, 'D' to move Z-axis DOWN, 'Q' to quit.")

    while True:
        key = getch()
        if key == 'u' or key == 'U':
            controller.move_z_up()
            # Optionally, move other motors while Z is up
            print("\nMoving other motors (ID 1 and 2) to new positions...")
            controller.write_positions([1, 2], [2048, 2048])
            controller.wait_until_reached([1, 2], [2048, 2048])

        elif key == 'd' or key == 'D':
            controller.move_z_down()
            # After Z-axis has moved down and its load has settled, you can command other motors.
            # During the Z-axis's downward PWM movement, it's generally not advisable to command other motors
            # if they share power or control resources, but after it has stopped based on load, it's fine.
            print("\nMoving other motors (ID 1 and 2) to different positions after Z-down operation...")
            controller.write_positions([1, 2], [2100, 2200])
            controller.wait_until_reached([1, 2], [2100, 2200])

        elif key == 'q' or key == 'Q':
            print("\n--- Quitting Program ---")
            break
        else:
            print("Invalid input. Press 'U' for Z-up, 'D' for Z-down, 'Q' to quit.")

    controller.disable_torque_and_close()
