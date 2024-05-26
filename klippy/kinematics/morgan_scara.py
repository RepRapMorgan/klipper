# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper

class MorganScaraKinematics:
    def __init__(self, toolhead, config):
        # Setup morgan rails
        stepper_configs = [config.getsection('stepper_' + s) for s in 'abz']

        # Link A (proximal arm segment)
        rail_a = stepper.PrinterStepper(stepper_configs[0],
                                     units_in_radians = True)
        # Link B (distal arm segment)
        rail_b = stepper.PrinterStepper(stepper_configs[1],
                                     units_in_radians = True)
        # Elevator
        rail_z = stepper.LookupMultiRail(stepper_configs[2])
        self.steppers = [rail_a, rail_b] + rail_z.get_steppers()
        self.rails = [rail_z]
        
        # Create two dummy cartesian kinematics for homing
        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cartesian_kinematics_L = ffi_main.gc(ffi_lib.cartesian_stepper_alloc('x'), ffi_lib.free)
        self.cartesian_kinematics_R = ffi_main.gc(ffi_lib.cartesian_stepper_alloc('y'), ffi_lib.free)

        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off)

        # Read config
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.link_a = config.getfloat('link_a_length', above=0.)
        self.link_b = config.getfloat('link_b_length', above=0.)
        # Setup rotary morgan calibration helper
        
        # Setup iterative solver
        rail_a.setup_itersolve('morgan_scara_stepper_alloc',
                              'a', self.link_a, self.link_b)
        rail_b.setup_itersolve('morgan_scara_stepper_alloc',
                              'b', self.link_a, self.link_b)
        rail_z.setup_itersolve('cartesian_stepper_alloc', 'z')
        
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        # Setup boundary checks
        self.need_home = True
        # Self Limit prevents collisions with arm hardware
        self.limit_xy_magnitude = (
            # Distances of less than 45mm can cause collition
            config.getfloat('min_base_distance', 45., above=0.),
            self.link_a + self.link_b - 1
        )

        logging.info('SCARA with proximal/distal linkages {:.1f}/{:.1f}mm'
                     .format(self.link_a, self.link_b))
        logging.info(
            'SCARA with min/max (x,y) distances {:.1f}/{:.1f}mm'
            .format(self.limit_xy_magnitude[0], self.limit_xy_magnitude[1]))
        self.home_position = (0, self.limit_xy_magnitude[0], 0)
        self.set_position(self.home_position, ())

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_tag_position(self):
        spos = [s.get_tag_position() for s in self.steppers]
        a_angle, b_angle, z = spos
        midpoint = (a_angle - b_angle) / 2.0
        distance = (self.link_a * math.cos(midpoint)
            - math.sqrt(self.link_b ** 2
                        - (self.link_a ** 2) * (math.sin(midpoint) ** 2)))

        x, y = (math.cos(a_angle - midpoint) * distance,
        math.sin(a_angle - midpoint) * distance)
        return [x, y, z]

    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False

    def _motor_off(self, print_time):
        self.need_home = True

    def home(self, homing_state):
        kinematics = [self.cartesian_kinematics_L, self.cartesian_kinematics_R]
        prev_sks    = [stepper.set_stepper_kinematics(kinematic)
            for stepper, kinematic in zip(self.steppers, kinematics)]

        # home
        rails = [self.rails[0], self.rails[1], self.rails[2]]
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.
        homing_state.home_rails(rails, forcepos, self.home_position)

        # swap back to original kinematics
        for stepper, prev_sk in zip(self.steppers, prev_sks):
            stepper.set_stepper_kinematics(prev_sk)
            # do kinematic math to figure out what x,y the home position actually is, and set it
            #self.toolhead.set_position( [x, y, 0, 0], (0, 1))
        # All axes are homed simultaneously
        #homing_state.set_axes([0, 1, 2])
        #forcepos = list(self.home_position)
        #forcepos[2] = -1.
        #homing_state.home_rails(self.rails, forcepos, self.home_position)

    def check_move(self, move):
        end_pos = move.end_pos
        if self.need_home:
            raise move.move_error("Must home first")

        distance = math.sqrt(end_pos[0] ** 2 + end_pos[1] ** 2)
        if distance < self.limit_xy_magnitude[0]:
            raise move.move_error(end_pos, "(x,y) pos too close to base joint")
        elif distance >= self.limit_xy_magnitude[1]:
            raise move.move_error(end_pos, "(x,y) pos exceeds reach of arms")
        elif not move.axes_d[2]:
            return
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(self.max_z_velocity * z_ratio,
                         self.max_z_accel * z_ratio)

    def get_status(self):
        return {'homed_axes': '' if self.need_home else 'XYZ'}



def load_kinematics(toolhead, config):
    return MorganScaraKinematics(toolhead, config)
