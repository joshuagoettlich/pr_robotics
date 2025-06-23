#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import tty
import termios
from dynamixel_sdk import *

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class DynamixelController:
    def __init__(self, ids, position_limits=None, device_name='/dev/ttyUSB0', baudrate=1_000_000, protocol_version=2.0):
        assert isinstance(ids, list), "Motor IDs must be a list."
        self.dxl_ids = ids
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.MOVING_THRESHOLD = 20

        # Default motion range
        if position_limits is None:
            self.position_limits = [(1100, 2900)] * len(ids)
        else:
            assert len(position_limits) == len(ids), "position_limits must match the number of IDs"
            self.position_limits = position_limits

        self.position_indices = [0] * len(ids)

        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        self._open_port()
        self._enable_torque()

    def _open_port(self):
        if not self.portHandler.openPort():
            print("Failed to open port")
            sys.exit(1)
        if not self.portHandler.setBaudRate(self.baudrate):
            print("Failed to set baudrate")
            sys.exit(1)
        print("Port opened and baudrate set")

    def _enable_torque(self):
        for dxl_id in self.dxl_ids:
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, 10)
            self._check_comm(result, error, f"Setting profile acceleration for ID {dxl_id}")
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, 100)
            self._check_comm(result, error, f"Setting profile velocity for ID {dxl_id}")
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            self._check_comm(result, error, f"Enabling torque for ID {dxl_id}")

    def _check_comm(self, result, error, context=""):
        if result != COMM_SUCCESS:
            print(f"{context}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"{context}: {self.packetHandler.getRxPacketError(error)}")

             
    def write_positions(self, target_positions):
        assert len(target_positions) == len(self.dxl_ids), "Length of target_positions must match motor count."
        print("Writing target positions:", target_positions)
        for i, dxl_id in enumerate(self.dxl_ids):
            goal_pos = target_positions[i]
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
            self._check_comm(result, error, f"Writing goal position {goal_pos} for ID {dxl_id}")
    
    def read_positions(self):
        positions = []
        for dxl_id in self.dxl_ids:
            present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            self._check_comm(result, error, f"Reading position for ID {dxl_id}")
            positions.append(present_pos)
        return positions
    
    def wait_until_reached(self, target_positions):
        assert len(target_positions) == len(self.dxl_ids), "Length of target_positions must match motor count."

        for i, dxl_id in enumerate(self.dxl_ids):
            goal_pos = target_positions[i]
            while True:
                present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
                self._check_comm(result, error, f"Reading position for ID {dxl_id}")
                #print(f"[ID:{dxl_id}] Goal: {goal_pos}, Present: {present_pos}")
                if abs(goal_pos - present_pos) <= self.MOVING_THRESHOLD:
                    break


    def move_and_monitor(self):
        while True:
            print("Press any key to move (ESC to quit):")
            if getch() == chr(0x1b):
                break

            for i, dxl_id in enumerate(self.dxl_ids):
                min_pos, max_pos = self.position_limits[i]
                goal_pos = min_pos if self.position_indices[i] == 0 else max_pos

                print(f"[ID:{dxl_id}] Moving to {goal_pos}")
                result, error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
                self._check_comm(result, error, f"Writing goal position for ID {dxl_id}")

            # Monitor until each motor reaches its goal
            for i, dxl_id in enumerate(self.dxl_ids):
                min_pos, max_pos = self.position_limits[i]
                goal_pos = min_pos if self.position_indices[i] == 0 else max_pos

                while True:
                    present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
                    self._check_comm(result, error, f"Reading position for ID {dxl_id}")
                    print(f"[ID:{dxl_id}] Goal: {goal_pos}, Present: {present_pos}")
                    if abs(goal_pos - present_pos) <= self.MOVING_THRESHOLD:
                        break

                # Toggle for next move
                self.position_indices[i] ^= 1  # Flip 0 <-> 1

    def disable_torque_and_close(self):
        for dxl_id in self.dxl_ids:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self._check_comm(result, error, f"Disabling torque for ID {dxl_id}")
        self.portHandler.closePort()
        print("Port closed")

if __name__ == "__main__":
    # Example usage: 3 motors with individual min/max limits
    dxl_ids = [1, 2, 3]
    limits = [(1200, 2800), (800, 1600), (2800, 3100)]  # Individual limits per motor
    controller = DynamixelController(dxl_ids, position_limits=limits)
    controller.write_positions([1500, 1200, 2900])  # Initial positions
    controller.wait_until_reached([1500, 1200, 2900])  # Wait until they reach the initial positions
    print("Press any key to start moving...")
    getch()  # Wait for user input to start moving
    print("Starting movement...")
    controller.write_positions([2400, 2000, 3000])  # Move to new positions
    controller.wait_until_reached([2400, 2000, 3000])  # Wait until they reach the new positions
    
    controller.disable_torque_and_close()

    