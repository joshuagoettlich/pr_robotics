#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import tty
import termios
from dynamixel_sdk import *
import time # Import for time.sleep
import rospy

class DynamixelController:
    """
    A thread-safe controller class for Dynamixel motors, supporting both Position and PWM Control modes.
    """
    def __init__(self, ids, z_axis_id, z_up_position, z_down_pwm, position_limits=None, device_name='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0):
        assert isinstance(ids, list), "Motor IDs must be a list."
        assert z_axis_id in ids, "Z-axis ID must be one of the provided motor IDs."
        self.dxl_ids = ids
        self.z_axis_id = z_axis_id
        self.z_up_position = z_up_position
        self.z_down_pwm = z_down_pwm
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        # Control Table Addresses
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_LOAD = 126
        self.ADDR_GOAL_PWM = 100
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_PROFILE_VELOCITY = 112

        self.ADDR_POSITION_D_GAIN = 80
        self.ADDR_POSITION_I_GAIN = 82
        self.ADDR_POSITION_P_GAIN = 84

        # Operational Values
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.MOVING_THRESHOLD_POSITION = 1
        self.STOPPED_LOAD_THRESHOLD = 20

        # Operating Modes
        self.OPERATING_MODE_POSITION = 3
        self.OPERATING_MODE_PWM = 16

        if position_limits is None: self.position_limits = [(1100, 2900)] * len(ids)
        else: self.position_limits = position_limits

        self.current_modes = {dxl_id: None for dxl_id in self.dxl_ids}
        self.port_is_open = False
        self.comm_lock = threading.Lock()

        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        self._open_port()
        self._initial_setup()

    def _open_port(self):
        with self.comm_lock:
            if self.portHandler.openPort():
                self.port_is_open = True
            else:
                rospy.logfatal(f"Failed to open port {self.device_name}")
                sys.exit(1)
            if not self.portHandler.setBaudRate(self.baudrate):
                rospy.logfatal(f"Failed to set baudrate to {self.baudrate}")
                sys.exit(1)
            rospy.loginfo(f"Dynamixel port {self.device_name} opened at {self.baudrate} bps.")

    def _initial_setup(self):
        with self.comm_lock:
            rospy.loginfo("--- Performing initial setup for all motors ---")
            for dxl_id in self.dxl_ids:
                self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
                self.set_operating_mode(dxl_id, self.OPERATING_MODE_POSITION, locked=True)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_ACCELERATION, 10)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_VELOCITY, 200)
                self.set_torque_enable(dxl_id, self.TORQUE_ENABLE, locked=True)
                self.set_pid_gains(dxl_id, 1000, 100, 100, locked=True)  # Example PID gains
            rospy.loginfo("--- Initial setup complete ---")

    def _check_comm(self, result, error, context=""):
        if result != COMM_SUCCESS:
            rospy.logwarn(f"COMM_ERROR: {context}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            rospy.logwarn(f"PACKET_ERROR: {context}: {self.packetHandler.getRxPacketError(error)}")

    def set_torque_enable(self, dxl_id, enable, locked=False):
        if not locked: self.comm_lock.acquire()
        try:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, enable)
            self._check_comm(result, error, f"Setting torque for ID {dxl_id}")
        finally:
            if not locked: self.comm_lock.release()

    def set_pid_gains(self, dxl_id, p_gain, i_gain, d_gain, locked=False):
        """Sets the Position P, I, and D gains for a specific motor."""
        if not locked: self.comm_lock.acquire()
        try:
            # It's good practice to disable torque before changing critical parameters
            self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
            
            # Write P, I, and D gains (Note: These are 2-byte values)
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_POSITION_P_GAIN, p_gain)
            self._check_comm(result, error, f"Setting P-Gain for ID {dxl_id}")
            
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_POSITION_I_GAIN, i_gain)
            self._check_comm(result, error, f"Setting I-Gain for ID {dxl_id}")
            
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_POSITION_D_GAIN, d_gain)
            self._check_comm(result, error, f"Setting D-Gain for ID {dxl_id}")
            
            self.set_torque_enable(dxl_id, self.TORQUE_ENABLE, locked=True)
            rospy.loginfo(f"Set PID gains for ID {dxl_id} to P:{p_gain}, I:{i_gain}, D:{d_gain}")
            
        finally:
            if not locked: self.comm_lock.release()


    def set_operating_mode(self, dxl_id, mode, locked=False):
        if self.current_modes.get(dxl_id) == mode: return
        if not locked: self.comm_lock.acquire()
        try:
            self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_OPERATING_MODE, mode)
            self._check_comm(result, error, f"Setting operating mode for ID {dxl_id}")
            if result == COMM_SUCCESS and error == 0: self.current_modes[dxl_id] = mode
            self.set_pid_gains(dxl_id, 1000, 100, 100, locked=True)  # Example PID gains

            self.set_torque_enable(dxl_id, self.TORQUE_ENABLE, locked=True)
        finally:
            if not locked: self.comm_lock.release()

    def write_positions(self, dxl_ids_to_move, target_positions):
        with self.comm_lock:
            for i, dxl_id in enumerate(dxl_ids_to_move):
                if self.current_modes.get(dxl_id) != self.OPERATING_MODE_POSITION: continue
                goal_pos = target_positions[i]
                result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
                self._check_comm(result, error, f"Writing position for ID {dxl_id}")

    def write_pwms(self, dxl_ids_to_move, target_pwms):
        with self.comm_lock:
            for i, dxl_id in enumerate(dxl_ids_to_move):
                if self.current_modes.get(dxl_id) != self.OPERATING_MODE_PWM: continue
                goal_pwm = target_pwms[i]
                result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_PWM, goal_pwm)
                self._check_comm(result, error, f"Writing PWM for ID {dxl_id}")

    def read_position(self, dxl_id):
        with self.comm_lock:
            if not self.port_is_open: return 0
            present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            self._check_comm(result, error, f"Reading position for ID {dxl_id}")
            return present_pos if result == COMM_SUCCESS else 0

    def read_positions(self):
        return [self.read_position(dxl_id) for dxl_id in self.dxl_ids]
        
    def read_load(self, dxl_id):
        with self.comm_lock:
            if not self.port_is_open: return 0
            present_load, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_LOAD)
            self._check_comm(result, error, f"Reading load for ID {dxl_id}")
            if result != COMM_SUCCESS: return 0
            return present_load - 65536 if present_load > 32767 else present_load

    def wait_until_reached(self, dxl_ids_to_wait, target_positions):
        for i, dxl_id in enumerate(dxl_ids_to_wait):
            goal_pos = target_positions[i]
            while not rospy.is_shutdown():
                if abs(goal_pos - self.read_position(dxl_id)) <= self.MOVING_THRESHOLD_POSITION: break
                rospy.sleep(0.02)

    def wait_until_load_threshold(self, dxl_id, threshold):
        rospy.loginfo(f"Waiting for ID {dxl_id} load to be >= {abs(threshold)}...")
        while not rospy.is_shutdown():
            if abs(self.read_load(dxl_id)) >= abs(threshold):
                rospy.loginfo(f"ID {dxl_id} load threshold reached.")
                break
            rospy.sleep(0.02)

    def move_z_up(self):
        rospy.loginfo(f"Moving Z-axis (ID: {self.z_axis_id}) UP to {self.z_up_position}")
        self.set_operating_mode(self.z_axis_id, self.OPERATING_MODE_POSITION)
        self.write_positions([self.z_axis_id], [self.z_up_position])
        self.wait_until_reached([self.z_axis_id], [self.z_up_position])
        rospy.loginfo(f"Z-axis is UP.")

    def move_z_down(self):
        rospy.loginfo(f"Moving Z-axis (ID: {self.z_axis_id}) DOWN with PWM {self.z_down_pwm}")
        self.set_operating_mode(self.z_axis_id, self.OPERATING_MODE_PWM)
        self.write_pwms([self.z_axis_id], [self.z_down_pwm])
        self.wait_until_load_threshold(self.z_axis_id, self.STOPPED_LOAD_THRESHOLD)
        self.write_pwms([self.z_axis_id], [0]) # Stop the motor
        rospy.loginfo("Z-axis is DOWN (stopped by load).")

    def disable_torque_and_close(self):
        with self.comm_lock:
            rospy.loginfo("Disabling torque for all motors.")
            for dxl_id in self.dxl_ids:
                self.set_torque_enable(dxl_id, self.TORQUE_DISABLE, locked=True)
            if self.port_is_open:
                self.portHandler.closePort()
                self.port_is_open = False
                rospy.loginfo("Port closed.")
