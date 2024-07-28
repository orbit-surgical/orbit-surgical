# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Keyboard controller for SE(3) control."""

import numpy as np
import weakref
from collections.abc import Callable
from scipy.spatial.transform.rotation import Rotation

import carb
import omni

from omni.isaac.lab.devices import DeviceBase


class Se3KeyboardDualArm(DeviceBase):
    """A dual-arm keyboard controller for sending SE(3) commands as delta poses and binary command (open/close).

    This class is designed to provide a dual-arm keyboard controller for robotic arms with grippers.
    It uses the Omniverse keyboard interface to listen to keyboard events and map them to robot's
    task-space commands.

    The command comprises of two parts:

    * delta pose: a 6D vector of (x, y, z, roll, pitch, yaw) in meters and radians.
    * gripper: a binary command to open or close the gripper.

    Key bindings:
        ============================== ================= =================
        Description                    Key (+ve axis)    Key (-ve axis)
        ============================== ================= =================
        First Arm:
        Toggle gripper (open/close)    O
        Move along x-axis              W                 S
        Move along y-axis              A                 D
        Move along z-axis              Q                 E
        Rotate along x-axis            Z                 X
        Rotate along y-axis            KEY_1             KEY_2
        Rotate along z-axis            C                 V

        Second Arm:
        Toggle gripper (open/close)    P
        Move along x-axis              Y                 H
        Move along y-axis              G                 J
        Move along z-axis              T                 U
        Rotate along x-axis            B                 N
        Rotate along y-axis            I                 K
        Rotate along z-axis            M                 ,
        ============================== ================= =================

    .. see also::

        The official documentation for the keyboard interface: `Carb Keyboard Interface <https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html>`__.

    """

    def __init__(self, pos_sensitivity: float = 0.4, rot_sensitivity: float = 0.8):
        """Initialize the keyboard layer.

        Args:
            pos_sensitivity: Magnitude of input position command scaling. Defaults to 0.05.
            rot_sensitivity: Magnitude of scale input rotation commands scaling. Defaults to 0.5.
        """
        # store inputs
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        # acquire omniverse interfaces
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        # note: Use weakref on callbacks to ensure that this object can be deleted when its destructor is called.
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
        )
        # bindings for keyboard to command
        self._create_key_bindings()
        # command buffers
        self._close_gripper_0 = False
        self._delta_pos_0 = np.zeros(3)  # (x, y, z)
        self._delta_rot_0 = np.zeros(3)  # (roll, pitch, yaw)
        self._close_gripper_1 = False
        self._delta_pos_1 = np.zeros(3)  # (x, y, z)
        self._delta_rot_1 = np.zeros(3)  # (roll, pitch, yaw)
        # dictionary for additional callbacks
        self._additional_callbacks = dict()

    def __del__(self):
        """Release the keyboard interface."""
        self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)
        self._keyboard_sub = None

    def __str__(self) -> str:
        """Returns: A string containing the information of joystick."""
        msg = f"Keyboard Controller for SE(3): {self.__class__.__name__}\n"
        msg += f"\tKeyboard name: {self._input.get_keyboard_name(self._keyboard)}\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tFirst Arm:\n"
        msg += "\tToggle gripper (open/close): O\n"
        msg += "\tMove arm along x-axis: W/S\n"
        msg += "\tMove arm along y-axis: A/D\n"
        msg += "\tMove arm along z-axis: Q/E\n"
        msg += "\tRotate arm along x-axis: Z/X\n"
        msg += "\tRotate arm along y-axis: 1/2\n"
        msg += "\tRotate arm along z-axis: C/V\n"
        msg += "\tSecond Arm:\n"
        msg += "\tToggle gripper (open/close): P\n"
        msg += "\tMove arm along x-axis: Y/H\n"
        msg += "\tMove arm along y-axis: G/J\n"
        msg += "\tMove arm along z-axis: T/U\n"
        msg += "\tRotate arm along x-axis: B/N\n"
        msg += "\tRotate arm along y-axis: I/K\n"
        msg += "\tRotate arm along z-axis: M/,"
        return msg

    """
    Operations
    """

    def reset(self):
        # default flags
        self._close_gripper_0 = False
        self._delta_pos_0 = np.zeros(3)  # (x, y, z)
        self._delta_rot_0 = np.zeros(3)  # (roll, pitch, yaw)
        self._close_gripper_1 = False
        self._delta_pos_1 = np.zeros(3)  # (x, y, z)
        self._delta_rot_1 = np.zeros(3)  # (roll, pitch, yaw)

    def add_callback(self, key: str, func: Callable):
        """Add additional functions to bind keyboard.

        A list of available keys are present in the
        `carb documentation <https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html>`__.

        Args:
            key: The keyboard button to check against.
            func: The function to call when key is pressed. The callback function should not
                take any arguments.
        """
        self._additional_callbacks[key] = func

    def advance(self) -> tuple[np.ndarray, bool, np.ndarray, bool]:
        """Provides the result from keyboard event state.

        Returns:
            A tuple containing the delta pose command and gripper commands.
        """
        # convert to rotation vector
        rot_vec_0 = Rotation.from_euler("XYZ", self._delta_rot_0).as_rotvec()
        rot_vec_1 = Rotation.from_euler("XYZ", self._delta_rot_1).as_rotvec()
        # return the command and gripper state
        return (
            np.concatenate([self._delta_pos_0, rot_vec_0]),
            self._close_gripper_0,
            np.concatenate([self._delta_pos_1, rot_vec_1]),
            self._close_gripper_1
        )

    """
    Internal helpers.
    """

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Subscriber callback to when kit is updated.

        Reference:
            https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html
        """
        # apply the command when pressed
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "L":
                self.reset()
            if event.input.name == "O":
                self._close_gripper_0 = not self._close_gripper_0
            elif event.input.name in ["W", "S", "A", "D", "Q", "E"]:
                self._delta_pos_0 += self._INPUT_KEY_MAPPING[event.input.name]
            elif event.input.name in ["Z", "X", "KEY_1", "KEY_2", "C", "V"]:
                self._delta_rot_0 += self._INPUT_KEY_MAPPING[event.input.name]
            elif event.input.name == "P":
                self._close_gripper_1 = not self._close_gripper_1
            elif event.input.name in ["Y", "H", "J", "G", "U", "T"]:
                self._delta_pos_1 += self._INPUT_KEY_MAPPING[event.input.name]
            elif event.input.name in ["B", "N", "I", "K", "M", "COMMA"]:
                self._delta_rot_1 += self._INPUT_KEY_MAPPING[event.input.name]
        # remove the command when un-pressed
        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in ["W", "S", "A", "D", "Q", "E"]:
                self._delta_pos_0 -= self._INPUT_KEY_MAPPING[event.input.name]
            elif event.input.name in ["Z", "X", "KEY_1", "KEY_2", "C", "V"]:
                self._delta_rot_0 -= self._INPUT_KEY_MAPPING[event.input.name]
            elif event.input.name in ["Y", "H", "J", "G", "U", "T"]:
                self._delta_pos_1 -= self._INPUT_KEY_MAPPING[event.input.name]
            elif event.input.name in ["B", "N", "I", "K", "M", "COMMA"]:
                self._delta_rot_1 -= self._INPUT_KEY_MAPPING[event.input.name]
        # additional callbacks
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._additional_callbacks:
                self._additional_callbacks[event.input.name]()

        # since no error, we are fine :)
        return True

    def _create_key_bindings(self):
        """Creates default key binding."""
        self._INPUT_KEY_MAPPING = {
            # toggle: gripper command
            "O": True,
            # x-axis (forward)
            "W": np.asarray([1.0, 0.0, 0.0]) * self.pos_sensitivity,
            "S": np.asarray([-1.0, 0.0, 0.0]) * self.pos_sensitivity,
            # y-axis (right-left)
            "D": np.asarray([0.0, 1.0, 0.0]) * self.pos_sensitivity,
            "A": np.asarray([0.0, -1.0, 0.0]) * self.pos_sensitivity,
            # z-axis (up-down)
            "Q": np.asarray([0.0, 0.0, 1.0]) * self.pos_sensitivity,
            "E": np.asarray([0.0, 0.0, -1.0]) * self.pos_sensitivity,
            # roll (around x-axis)
            "Z": np.asarray([1.0, 0.0, 0.0]) * self.rot_sensitivity,
            "X": np.asarray([-1.0, 0.0, 0.0]) * self.rot_sensitivity,
            # pitch (around y-axis)
            "KEY_1": np.asarray([0.0, 1.0, 0.0]) * self.rot_sensitivity,
            "KEY_2": np.asarray([0.0, -1.0, 0.0]) * self.rot_sensitivity,
            # yaw (around z-axis)
            "C": np.asarray([0.0, 0.0, 1.0]) * self.rot_sensitivity,
            "V": np.asarray([0.0, 0.0, -1.0]) * self.rot_sensitivity,
            # toggle: gripper command
            "P": True,
            # x-axis (forward)
            "Y": np.asarray([1.0, 0.0, 0.0]) * self.pos_sensitivity,
            "H": np.asarray([-1.0, 0.0, 0.0]) * self.pos_sensitivity,
            # y-axis (right-left)
            "J": np.asarray([0.0, 1.0, 0.0]) * self.pos_sensitivity,
            "G": np.asarray([0.0, -1.0, 0.0]) * self.pos_sensitivity,
            # z-axis (up-down)
            "T": np.asarray([0.0, 0.0, 1.0]) * self.pos_sensitivity,
            "U": np.asarray([0.0, 0.0, -1.0]) * self.pos_sensitivity,
            # roll (around x-axis)
            "B": np.asarray([1.0, 0.0, 0.0]) * self.rot_sensitivity,
            "N": np.asarray([-1.0, 0.0, 0.0]) * self.rot_sensitivity,
            # pitch (around y-axis)
            "I": np.asarray([0.0, 1.0, 0.0]) * self.rot_sensitivity,
            "K": np.asarray([0.0, -1.0, 0.0]) * self.rot_sensitivity,
            # yaw (around z-axis)
            "M": np.asarray([0.0, 0.0, 1.0]) * self.rot_sensitivity,
            "COMMA": np.asarray([0.0, 0.0, -1.0]) * self.rot_sensitivity,
        }
