# Code for handling the kinematics of  Morgan SCARA robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024       Quentin Harley <quentin@morgan3dp.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging
import stepper #,mathutil, chelper

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

        # Read arm lengths

        self.l1 = stepper_configs[0].getfloat('arm_length', above=0.)
        self.l2 = stepper_configs[1].getfloat('arm_length', above=0.)
        self.l1_sq = self.l1**2
        self.l2_sq = self.l2**2

        printer_config = config.getsection('printer')
        self.column_x = printer_config.getfloat('column_x', default=190.)
        self.column_y = printer_config.getfloat(
            'column_y', below=0., default=-70.)
        self.d_limit = printer_config.getfloat('D_limit', default=0.95)

        # Flag for homing to disable the kinematics
        self.homing_active = False

        #self.abs_endstops = [(rail.get_homing_info().position_endstop
        #                      + math.sqrt(arm2 - radius**2))
        #                     for rail, arm2 in zip(self.rails, self.arm2)]

        # Setup itersolve for the steppers
        self.rails[0].setup_itersolve('morgan_scara_stepper_alloc', 'a',
            self.l1, self.l2, self.column_x, self.column_y, self.d_limit)
        self.rails[1].setup_itersolve('morgan_scara_stepper_alloc', 'b',
            self.l1, self.l2, self.column_x, self.column_y, self.d_limit)
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)

        # Setup boundary checks
        self.need_home = False
        self.home_position = self.calc_home_position(stepper_configs)
        self.set_position([0., 0., 0.], ())

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

    def calc_home_position(self, stepper_configs):
        homes = [config.getfloat('home_position', 0.)
                       for config in stepper_configs]
        home_xy = self.forward_kinematics(homes[0], homes[1])
        home_pos = [home_xy[0], home_xy[1], homes[2]]
        return home_pos

    def set_position(self, newpos, homing_axes):
        # Update internal position state
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False

    def home(self, homing_state):
        # Define homing behavior
        # All axes are homed simultaneously
        logging.info("Mogan home was called")
        self.rails[0].setup_itersolve('cartesian_stepper_alloc', b'a')
        self.rails[1].setup_itersolve('cartesian_stepper_alloc', b'b')
        
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
            
        logging.info("Kinematic changed")
        #logging.info(
        #    "Delta max build height %.2fmm (radius tapered above %.2fmm)"
        #    % (self.max_z, self.limit_z))
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        # homing_state.home_rails(self.rails, forcepos, self.home_position)
        logging.info("all done except the actual homing")
        # Setup itersolve for the steppers
        self.rails[0].setup_itersolve('morgan_scara_stepper_alloc', 'a',
            self.l1, self.l2, self.column_x, self.column_y, self.d_limit)
        self.rails[1].setup_itersolve('morgan_scara_stepper_alloc', 'b',
            self.l1, self.l2, self.column_x, self.column_y, self.d_limit)
        
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        self.set_position(self.home_position, (0, 1, 2))
        logging.info("Kinematic restored and home position set")


    def _motor_off(self, print_time):
        #self.limit_xy2 = -1.
        self.need_home = False

    def check_move(self, move):
        # Validate if the move is within kinematic limits
        pass

    def get_status(self, eventtime):
        # Return kinematic status
        return {
            'homed_axes': '' if self.need_home else 'xyz',

        }

    def inverse_kinematics(self, x, y):
        # Calculate the inverse kinematics for a given point

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
        x_pos = self.l1 * math.cos(theta) + self.l2 * math.cos(psi_theta)
        y_pos = self.l1 * math.sin(theta) + self.l2 * math.sin(psi_theta)
        return [x_pos, y_pos]


    def get_calibration(self):
        pass

def load_kinematics(toolhead, config):
    return MorganScaraKinematics(toolhead, config)
