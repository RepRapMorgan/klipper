# Code for handling the kinematics of  Morgan SCARA robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024       Quentin Harley <quentin@morgan3dp.com>
# Copyright (C) 2020       Pontus Borg <glpontus@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging
import stepper
import chelper
#import mathutil

class MorganScaraKinematics:
    # This is a class for handling the kinematics of Morgan SCARA robots.
    # Init is a main function that initializes the class.
    def __init__(self, toolhead, config):
        # Setup arm rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abz']
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], units_in_radians=True)
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], units_in_radians=True)
        rail_z = stepper.LookupMultiRail(
            stepper_configs[2], units_in_radians=False)

        self.rails = [rail_a, rail_b, rail_z]

        printer_config = config.getsection('printer')
        # Read arm lengths (allow either per-stepper arm_length or
        # legacy printer-level inner/outer arm length parameters).
        l1_default = printer_config.getfloat('inner_arm_length', None,
                                             above=0.)
        l2_default = printer_config.getfloat('outer_arm_length', None,
                                             above=0.)
        self.l1 = stepper_configs[0].getfloat('arm_length', l1_default,
                                              above=0.)
        self.l2 = stepper_configs[1].getfloat('arm_length', l2_default,
                                              above=0.)
        self.l1_sq = self.l1**2
        self.l2_sq = self.l2**2

        self.column_x = printer_config.getfloat('column_x', default=190.)
        self.column_y = printer_config.getfloat('column_y', default=-70.)
        self.d_limit = printer_config.getfloat('d_limit', default=0.95,
                                               minval=0., maxval=1.)

        #self.abs_endstops = [(rail.get_homing_info().position_endstop
        #                      + math.sqrt(arm2 - radius**2))
        #                     for rail, arm2 in zip(self.rails, self.arm2)]

        # Setup itersolve for the steppers
        self.rails[0].setup_itersolve('morgan_scara_stepper_alloc', b'a',
            self.l1, self.l2, self.column_x, self.column_y, self.d_limit)
        self.rails[1].setup_itersolve('morgan_scara_stepper_alloc', b'b',
            self.l1, self.l2, self.column_x, self.column_y, self.d_limit)
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        # Setup max velocity
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity,
            above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)

        # Setup boundary checks
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        self.min_xy2 = max(0., self.l1_sq + self.l2_sq
                   - 2.0 * self.l1 * self.l2 * self.d_limit)
        self.max_xy2 = max(self.min_xy2, self.l1_sq + self.l2_sq
                   + 2.0 * self.l1 * self.l2 * self.d_limit)
        min_base_distance = printer_config.getfloat('min_base_distance', 0.,
                                minval=0.)
        self.min_xy2 = max(self.min_xy2, min_base_distance**2)
        self.need_home = True
        self.home_position = self.calc_home_position()

        max_xy = math.sqrt(self.max_xy2)
        min_z, max_z = self.rails[2].get_range()
        self.axes_min = toolhead.Coord(self.column_x - max_xy,
                           self.column_y - max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(self.column_x + max_xy,
                           self.column_y + max_xy, max_z, 0.)
        self.set_position([0., 0., 0.], "")

        # Homing trickery fake cartesial kinematic:
        # Borrowed from bondus/5barscara

        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cartesian_kinematics_a = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        self.cartesian_kinematics_b = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'y'), ffi_lib.free)

        logging.info("Morgan SCARA %.2f %.2f %.2f %.2f %.2f",
                     self.l1, self.l2, self.column_x,
                     self.column_y, self.d_limit)

    def get_steppers(self):
        # Return a list of steppers involved in the kinematics
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        # Convert current stepper positions to Cartesian (Forward Kinematics)
        theta = stepper_positions[self.rails[0].get_name()]
        psi_theta = stepper_positions[self.rails[1].get_name()]
        x_pos, y_pos = self.forward_kinematics(theta, psi_theta)
        z_pos = stepper_positions[self.rails[2].get_name()]
        return [x_pos, y_pos, z_pos]

    def calc_home_position(self):
        homes = [rail.get_homing_info().position_endstop for rail in self.rails]
        home_xy = self.forward_kinematics(homes[0], homes[1])
        home_pos = [home_xy[0], home_xy[1], homes[2]]
        return home_pos

    def set_position(self, newpos, homing_axes):
        # Update internal position state
        for rail in self.rails:
            rail.set_position(newpos)
        if "x" in homing_axes and "y" in homing_axes:
            self.limit_xy2 = self.max_xy2
        if "z" in homing_axes:
            self.limit_z = self.rails[2].get_range()
        self.need_home = self.limit_xy2 < 0. or self.limit_z[0] > self.limit_z[1]

    def clear_homing_state(self, clear_axes):
        if "x" in clear_axes or "y" in clear_axes:
            self.limit_xy2 = -1.
        if "z" in clear_axes:
            self.limit_z = (1.0, -1.0)
        self.need_home = self.limit_xy2 < 0. or self.limit_z[0] > self.limit_z[1]

    def home(self, homing_state):
        # Define homing behavior
        # code borrowed and adapted from bondus/5barscara
        # a and b axes are always homed simultaneously
        axes = homing_state.get_axes()
        home_xy = 0 in axes or 1 in axes
        home_z = 2 in axes
        updated_axes = []
        if home_xy:
            updated_axes.extend([0, 1])
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        logging.info("SCARA home %s", updated_axes)

        if home_xy:
            # Home left and right at the same time
            rails = [self.rails[0], self.rails[1]]
            homepos = [None, None, None, None]
            forcepos = [None, None, None, None]
            for i, rail in enumerate(rails):
                rail_min, rail_max = rail.get_range()
                hi = rail.get_homing_info()
                homepos[i] = hi.position_endstop
                if hi.positive_dir:
                    forcepos[i] = (hi.position_endstop
                                   - 1.5 * (hi.position_endstop - rail_min))
                else:
                    forcepos[i] = (hi.position_endstop
                                   + 1.5 * (rail_max - hi.position_endstop))

            # Swap to linear kinematics
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()

            steppers = [rail.get_steppers()[0] for rail in rails]
            kinematics = [self.cartesian_kinematics_a,
                          self.cartesian_kinematics_b]
            prev_sks = [s.set_stepper_kinematics(kinematic)
                        for s, kinematic in zip(steppers, kinematics)]

            try:
                logging.info("SCARA home AB %s %s %s", rails, forcepos, homepos)

                homing_state.home_rails(rails, forcepos, homepos)

                x, y = self.forward_kinematics(
                    rails[0].get_homing_info().position_endstop,
                    rails[1].get_homing_info().position_endstop)
                curpos = toolhead.get_position()
                toolhead.set_position([x, y, curpos[2], curpos[3]],
                                      homing_axes="xy")
                toolhead.flush_step_generation()

            except Exception:
                for s, prev_sk in zip(steppers, prev_sks):
                    s.set_stepper_kinematics(prev_sk)
                toolhead.flush_step_generation()
                raise
            for s, prev_sk in zip(steppers, prev_sks):
                s.set_stepper_kinematics(prev_sk)

        if home_z:
            logging.info("SCARA home Z %s", axes)
            rail = self.rails[2]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None]
            homepos[2] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[2] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[2] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            logging.info("SCARA home Z %s %s %s",
                         [rail], forcepos, homepos)
            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        del print_time
        self.clear_homing_state("xyz")

    def check_move(self, move):
        # Validate if the move is within kinematic limits
        end_pos = move.end_pos
        rel_x = end_pos[0] - self.column_x
        rel_y = end_pos[1] - self.column_y
        xy2 = rel_x * rel_x + rel_y * rel_y

        if self.limit_xy2 < 0.:
            if move.axes_d[0] or move.axes_d[1]:
                raise move.move_error("Must home axis first")
        elif xy2 < self.min_xy2 or xy2 > self.limit_xy2:
            raise move.move_error()

        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        # Return kinematic status
        del eventtime
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

    def inverse_kinematics(self, x, y):
        # Calculate the inverse kinematics for a given point
        x = x - self.column_x
        y = y - self.column_y

        # Calculate the distance to the point
        r_squared = x * x + y * y

        # Check if the point is reachable
        if (r_squared > (self.l1 + self.l2)**2
            or r_squared < (self.l1 - self.l2)**2):
            raise ValueError("Target point is out of reach")

        # Calculate psi
        d = (r_squared - self.l1**2 - self.l2**2) / (2.0 * self.l1 * self.l2)
        # Clamp d to prevent sqrt from returning NaN and collisions
        d = min(max(d, -self.d_limit), self.d_limit)

        # Psi in Morgan kinematics:
        # Distal arm is always on the right side of the proximal arm
        psi = math.atan2(math.sqrt(1 - d * d), d)
        psi = math.copysign(psi, -1.0)  # Negate psi if positive

        # Calculate theta
        theta = math.atan2(y, x) - math.atan2(self.l2 * math.sin(psi),
                                              self.l1 + self.l2 * math.cos(psi))

        # Return psi, as a sum with theta
        # Morgan kinematics: Distal arm is driven from the base
        return [theta, psi + theta]

    def forward_kinematics(self, theta, psi_theta):
        # Convert stepper positions to Cartesian (Forward Kinematics)
        x_pos = (self.column_x
                 + self.l1 * math.cos(theta)
                 + self.l2 * math.cos(psi_theta))
        y_pos = (self.column_y
                 + self.l1 * math.sin(theta)
                 + self.l2 * math.sin(psi_theta))
        return [x_pos, y_pos]


    def get_calibration(self):
        pass

def load_kinematics(toolhead, config):
    return MorganScaraKinematics(toolhead, config)
