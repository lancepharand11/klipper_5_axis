# Author: Lance Pharand
import math 
import stepper
import logging
import numpy as np


class FiveAxisKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # Setup axis rails (X, Y, Z for toolhead translation and A, B for bed rotation)
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzab']
        for rail, axis in zip(self.rails, 'xyzab'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())

        # Linear axes limits (X, Y, Z)
        ranges = [r.get_range() for r in self.rails[:3]]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

        # Rotational axes limits (A, B in radians)
        self.rotational_limits = {
            'a': (-math.radians(45), math.radians(45)),
            'b': (-math.radians(45), math.radians(45)),
        }

        # Initialize bed rotation angles
        self.bed_rotation_a = 0.0 # rad
        self.bed_rotation_b = 0.0 # rad

        # Register steppers with the toolhead
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        
        # Setup boundary checks: Linear axes
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3

        # Setup boundary checks: Rotational axes (in radians)
        # NOTE: User set params
        self.max_velocity = 0.5 # rad/s
        self.max_accel = 0.1 # rad/s^2
        self.max_angle = 0.785398 # rad
        
        # Overwrite with values from config file, if used. Otherwise, default to the 2nd input param
        self.max_accel_a = config.getfloat('max_accel_a', 0.1, above=0., maxval=self.max_accel) # Max rotational accel (rad/s^2)
        self.max_accel_b = config.getfloat('max_accel_b', 0.1, above=0., maxval=self.max_accel) # Max rotational accel (rad/s^2)
        self.max_velo_a = config.getfloat('max_velo_a', 0.5, above=0., maxval=self.max_velocity) # Max rotational velo (rad/s)
        self.max_velo_b = config.getfloat('max_velo_b', 0.5, above=0., maxval=self.max_velocity) # Max rotational velo (rad/s)
        # TODO: Keep? 
        self.a_steps_per_rad = config.getfloat('a_steps_per_rad', 10) # Conversion between steps and rad 
        self.b_steps_per_rad = config.getfloat('b_steps_per_rad', 10) # Conversion between steps and rad

    def get_steppers(self):
        """
        Get all steppers, including linear and rotational axes
        """
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        """
        Calculate the toolhead's Cartesian position relative to the rotated bed.
        Applies the bed's rotation (A and B axes) to the toolhead's position.
        """
        # Retrieve toolhead's Cartesian coordinates
        x, y, z = [stepper_positions[rail.get_name()] for rail in self.rails[:3]]

        # TODO: Ensure radians are stored here
        # Retrieve bed rotation angles (in radians)
        a_angle = stepper_positions[self.rails[3].get_name()]
        b_angle = stepper_positions[self.rails[4].get_name()]

        # TODO: Check that axes of rotation are correct on printer
        # Rotation matrix for A-axis (Assumed as X-axis rotation)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(a_angle), -np.sin(a_angle)],
            [0, np.sin(a_angle), np.cos(a_angle)]
        ])

        # Rotation matrix for B-axis (Assumed as Y-axis rotation)
        R_y = np.array([
            [np.cos(b_angle), 0, np.sin(b_angle)],
            [0, 1, 0],
            [-np.sin(b_angle), 0, np.cos(b_angle)]
        ])

        R_combined = R_y @ R_x
        toolhead_position = np.array([[x], 
                                      [y], 
                                      [z]])
        transformed_position = R_combined @ toolhead_position

        # Updated coordinates relative to the rotated coordinate frame
        x_rot, y_rot, z_rot = transformed_position

        return [x_rot, y_rot, z_rot]

    def set_position(self, newpos, homing_axes):
        """
        Set the positions of the steppers based on the new toolhead position.
        Includes both linear (X, Y, Z) and rotational (A, B) axes.
        """
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        for axis in homing_axes:
            if axis in (3, 4):  # A or B axis
                self.update_bed_rotation(axis, newpos[axis])

    def update_bed_rotation(self, axis, angle):
        """
        Update the rotation angle of the bed for the given axis (A or B)
        """
        if axis == 3: # A-axis (Assumed as rotation about X)
            if not self.rotational_limits['a'][0] <= angle <= self.rotational_limits['a'][1]:
                raise ValueError(f"A-axis rotation {angle} exceeds limits.")
            self.bed_rotation_a = angle
        elif axis == 4: # B-axis (Assumed as rotation about Y)
            if not self.rotational_limits['b'][0] <= angle <= self.rotational_limits['b'][1]:
                raise ValueError(f"B-axis rotation {angle} exceeds limits.")
            self.bed_rotation_b = angle

    def home_axis(self, homing_state, axis, rail):
        """
        Perform homing for a single axis
        """
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()

        # TODO: cartesian.py has 4x None in homepos. Why?
        homepos = [None, None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        """
        Perform homing for all axes
        """
        for axis in homing_state.get_axes():
            self.home_axis(homing_state, axis, self.rails[axis])

    def _motor_off(self, print_time):
        """ 
        TODO: Figure out why these values were used.
        """
        self.limits = [(1.0, -1.0)] * 3

    def _check_endstops(self, move):
        """
        For linear axes, check that end position is within limits. 
        """
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        """
        Check that all limits are satisfied. 
        """
        # Linear limits
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

        # Rotational limits 
        rot_positions = move.end_pos[3:]
        if rot_positions[0] < self.rotational_limits['a'][0] or rot_positions[0] > self.rotational_limits['a'][1]:
            raise move.move_error(f"A-axis out of bounds: {rot_positions[0]} rad")
        if rot_positions[1] < self.rotational_limits['b'][0] or rot_positions[1] > self.rotational_limits['b'][1]:
            raise move.move_error(f"B-axis out of bounds: {rot_positions[1]} rad")

    def get_status(self, eventtime):
        """
        Return the current values of the kinematics.
        """
        # TODO: Can a and b be included here? 
        axes = [a for a, (l, h) in zip("xyzab", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'bed_rotation_a': self.bed_rotation_a,
            'bed_rotation_b': self.bed_rotation_b,
        }

def load_kinematics(toolhead, config):
    return FiveAxisKinematics(toolhead, config)
