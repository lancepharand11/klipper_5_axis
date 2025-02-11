# Author: Lance Pharand
import math 
import stepper
import logging
import numpy as np
from . import idex_modes


class FiveAxisKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # Setup rotational steppers (U, W for bed rotation)
        stepper_u = stepper.PrinterStepper(config.getsection('stepper_u'),
                                             units_in_radians=True)
        stepper_w = stepper.PrinterStepper(config.getsection('stepper_w'),
                                             units_in_radians=True)
        stepper_u.setup_itersolve('rot_stepper_alloc', b'u')
        stepper_w.setup_itersolve('rot_stepper_alloc', b'v')
        
        # Define these incase used in other processes
        self.dc_module = None
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []

        # Setup axis rails (X, Y, Z for toolhead translation)
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']
        for rail, axis in zip(self.rails, 'xyz'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())

        # Store all steppers in 1 place
        self.steppers = [stepper_u, stepper_w] + [ s for r in self.rails
                                                  for s in r.get_steppers() ]

        # Linear axes limits (X, Y, Z)
        ranges = [r.get_range() for r in self.rails[:3]]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

        # Rotational axes limits (in radians)
        self.rotational_limits = {
            'u': (-math.radians(45), math.radians(45)),
            'w': (-math.radians(45), math.radians(45)),
        }

        # Initialize bed rotation angles
        self.bed_rotation_u = 0.0 # rad
        self.bed_rotation_w = 0.0 # rad

        # Register steppers with the toolhead
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        
        # Setup motion boundary checks: Linear axes
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.linear_limits = [(1.0, -1.0)] * 3

        # Setup motion boundary checks: Rotational axes (in radians)
        # NOTE: User set params
        self.max_velocity = 0.5 # rad/s
        self.max_accel = 0.1 # rad/s^2
        self.max_angle = 0.785398 # rad
        
        # TODO: Integrate this into the config file
        # Overwrite with values from config file, if used. Otherwise, default to the 2nd input param
        self.max_accel_u = config.getfloat('max_accel_u', 0.1, above=0., maxval=self.max_accel) # Max rotational accel (rad/s^2)
        self.max_accel_w = config.getfloat('max_accel_w', 0.1, above=0., maxval=self.max_accel) # Max rotational accel (rad/s^2)
        self.max_velo_u = config.getfloat('max_velo_u', 0.5, above=0., maxval=self.max_velocity) # Max rotational velo (rad/s)
        self.max_velo_w = config.getfloat('max_velo_w', 0.5, above=0., maxval=self.max_velocity) # Max rotational velo (rad/s)
        # TODO: Keep? 
        self.u_steps_per_rad = config.getfloat('u_steps_per_rad', 10) # Conversion between steps and rad 
        self.w_steps_per_rad = config.getfloat('w_steps_per_rad', 10) # Conversion between steps and rad

    # def get_steppers(self):
    #     """
    #     Get all steppers (linear and rotational axes)
    #     """
    #     return [s for rail in self.rails for s in rail.get_steppers()]
    def get_steppers(self):
        """
        Get all steppers (linear and rotational axes)
        """
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        """
        Calculate the toolhead's Cartesian position relative to the rotated bed.
        Applies the bed's rotation (U and W axes) to the toolhead's position.

        :param stepper_positions:
            Dictionary keyed by stepper name, containing current stepper 
            positions. Linear axes in mm, rotational axes in radians.
        """
        # Retrieve toolhead's Cartesian coordinates
        x, y, z = [stepper_positions[rail.get_name()] for rail in self.rails[:3]]

        # Retrieve bed rotation angles (in radians)
        u_angle = stepper_positions[self.steppers[0].get_name()]
        w_angle = stepper_positions[self.steppers[1].get_name()]

        # TODO: Check that axes of rotation are correct on printer
        # Rotation matrix for U-axis (Assumed as X-axis rotation)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(u_angle), -np.sin(u_angle)],
            [0, np.sin(u_angle), np.cos(u_angle)]
        ])

        # Rotation matrix for W-axis (Assumed as Y-axis rotation)
        R_y = np.array([
            [np.cos(w_angle), 0, np.sin(w_angle)],
            [0, 1, 0],
            [-np.sin(w_angle), 0, np.cos(w_angle)]
        ])

        # TODO: May need to change the order that the rotation matrices are applied in? 
        R_combined = R_y @ R_x
        toolhead_position = np.array([[x], 
                                      [y], 
                                      [z]])
        transformed_position = R_combined @ toolhead_position

        # Updated coordinates relative to the rotated coordinate frame
        x_rot, y_rot, z_rot = transformed_position

        return [x_rot, y_rot, z_rot]
    
    # NOTE: Not used anywhere
    def update_limits(self, i, range):
        """
        Update homed limits for a given linear axis (X, Y, or Z).

        :param i:
            Integer index of the axis (0=X, 1=Y, 2=Z).
        :param range:
            Tuple (min_val, max_val) specifying the new valid range 
            for the specified axis.
        """
        l, h = self.linear_limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.linear_limits[i] = range

    def set_position(self, newpos, homing_axes):
        """
        Set the positions of the steppers based on the new toolhead position.
        Includes both linear and rotational (U, W) axes.

        :param newpos:
            A list or tuple giving the target positions in the order 
            [X, Y, Z, U, W, ...].
        :param homing_axes:
            A list of axis indices currently being homed. For example, 
            [0] = X-axis only, [3,4] = U and W axes, etc.
        """
        for s in self.steppers:
            s.set_position(newpos)

        for axis in homing_axes:
            if axis in (3, 4):  # U or W axis
                self.update_bed_rotation(axis, newpos[axis])
            else:
                rail = self.rails[axis]
                self.linear_limits[axis] = rail.get_range()
    
    def note_z_not_homed(self):
        """
        Helper for Safe Z Home
        """
        self.linear_limits[2] = (1.0, -1.0)

    def update_bed_rotation(self, axis, angle):
        """
        Update the rotation angle of the bed for the given axis (U or W)

        :param axis:
            Axis index (3=U, 4=W).
        :param angle:
            The new angle (in radians) to set for that axis.
        :raises ValueError:
            If `angle` is outside the configured rotational limits.
        """
        if axis == 3: # U-axis (Assumed as rotation about X)
            if not self.rotational_limits['u'][0] <= angle <= self.rotational_limits['u'][1]:
                raise ValueError(f"U-axis rotation {angle} exceeds limits.")
            self.bed_rotation_u = angle
        elif axis == 4: # W-axis (Assumed as rotation about Y)
            if not self.rotational_limits['w'][0] <= angle <= self.rotational_limits['w'][1]:
                raise ValueError(f"W-axis rotation {angle} exceeds limits.")
            self.bed_rotation_w = angle
        
    def home_axis(self, homing_state, axis, stepper):
        """
        Perform homing for a single axis

        :param homing_state:
            Object containing methods and state for the homing process 
            (e.g., homing_state.home_rails(...)).
        :param axis:
            Axis index (0=X, 1=Y, 2=Z, 3=U, 4=W).
        :param stepper:
            The stepper object corresponding to the axis being homed.
        """
        # Determine movement
        position_min, position_max = stepper.get_range()
        hi = stepper.get_homing_info()
        homepos = [None, None, None, None, None]  # TODO: Saw this line had 1 extra None in cartesian.py
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)

        # TODO: may need to modify the scaling here
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        
        # Perform homing
        homing_state.home_rails([stepper], forcepos, homepos)

    def home(self, homing_state):
        """
        Perform homing for all axes

        :param homing_state:
            An object that tracks which axes need homing and provides 
            homing operations.
        """
        for axis in homing_state.get_axes():
            self.home_axis(homing_state, axis, self.steppers[axis])

    def _motor_off(self, print_time):
        """ 
        TODO: Figure out why these values were used.
        """
        self.linear_limits = [(1.0, -1.0)] * 3

    def _check_endstops(self, move):
        """
        NOTE: For linear axes ONLY, check that end position is within limits. 
        """
        
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.linear_limits[i][0]
                     or end_pos[i] > self.linear_limits[i][1])):
                if self.linear_limits[i][0] > self.linear_limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        """
        Check that all limits are satisfied. 
        """
        # Linear limits
        limits = self.linear_limits
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
        if rot_positions[0] < self.rotational_limits['u'][0] or rot_positions[0] > self.rotational_limits['u'][1]:
            raise move.move_error(msg=f"U-axis out of bounds: {rot_positions[0]} rad")
    
        if rot_positions[1] < self.rotational_limits['w'][0] or rot_positions[1] > self.rotational_limits['w'][1]:
            raise move.move_error(msg=f"W-axis out of bounds: {rot_positions[1]} rad")

    def get_status(self, eventtime):
        """
        Return the current values of the kinematics.
        NOTE: Not an important method. Called in toolhead.py during test scripts only
        """
        axes = [a for a, (l, h) in zip("xyz", self.linear_limits) if l <= h]
        axes.append('u')
        axes.append('w')

        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return FiveAxisKinematics(toolhead, config)
