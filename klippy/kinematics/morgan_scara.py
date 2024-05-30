# Code for handling the kinematics of  Morgan SCARA robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024       Quentin Harley <quentin@morgan3dp.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math #,logging
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
        self.L1 = stepper_configs[0].getfloat('arm_length', above=0.)
        self.L2 = stepper_configs[1].getfloat('arm_length', above=0.)
        self.L1SQ = self.L1**2
        self.L2SQ = self.L2**2
        
        printer_config = config.getsection('printer')    
        self.column_x = printer_config.getfloat('column_x', default=190.)
        self.column_y = printer_config.getfloat('column_y', below=0., default=-70.)
        self.D_limit = printer_config.getfloat('D_limit', default=0.95)
               
        #self.abs_endstops = [(rail.get_homing_info().position_endstop
        #                      + math.sqrt(arm2 - radius**2))
        #                     for rail, arm2 in zip(self.rails, self.arm2)]
        
        # Setup itersolve for the steppers
        rail_a.setup_itersolve('morgan_scara_stepper_alloc', 'a', 
                                self.L1, self.L2, self.column_x, self.column_y, self.D_limit)
        rail_b.setup_itersolve('morgan_scara_stepper_alloc', 'b', 
                                self.L1, self.L2, self.column_x, self.column_y, self.D_limit)
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        
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
        #self.need_home = True
        self.set_position([0., 0., 0.], ())

    def get_steppers(self):
        # Return a list of steppers involved in the kinematics
        return [s for rail in self.rails for s in rail.get_steppers()]
       
    def calc_position(self, stepper_positions):
        # Convert stepper positions to Cartesian coordinates (Forward Kinematics)
        theta1 = stepper_positions[self.steppers[0].get_name()]
        theta2 = stepper_positions[self.steppers[0].get_name()]
        z_pos = stepper_positions[self.rails[1].get_name()]
        x_pos = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y_pos = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        return [x_pos, y_pos, z_pos]
    
    #def calc_home_position(self, stepper_configs[])
        # Calculate the home position

    #    return [home_x, home_y, home_z]

    def set_position(self, newpos, homing_axes):
        pass
    
        # Update internal position state
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False
    
    def home(self, homing_state):
        # Define homing behavior
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        #forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)
    
    def _motor_off(self, print_time):
        self.limit_xy2 = -1.
        self.need_home = True
    
    def check_move(self, move):
        # Validate if the move is within kinematic limits
        pass
    
    def get_status(self, eventtime):
        # Return kinematic status
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            
        }
        
    def inverse_kinematics(self, x, y):
        # Calculate the distance to the point
        r_squared = x**2 + y**2
        L1_squared = self.L1**2
        L2_squared = self.L2**2

        # Check if the point is reachable
        if r_squared > (self.L1 + self.L2)**2 or r_squared < (self.L1 - self.L2)**2:
            raise ValueError("Target point is out of reach")

        # Calculate theta2
        D = (r_squared - L1_squared - L2_squared) / (2 * self.L1 * self.L2)
        if D < -1.0 or D > 1.0:
            raise ValueError("Target point is out of reach due to numerical issues")
        
        theta2 = math.atan2(math.sqrt(1 - D**2), D)
        
        # Calculate theta1
        theta1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(theta2),
                                               self.L1 + self.L2 * math.cos(theta2))

        # Return the angles in steps
        return [theta1 * self.steps_per_mm, theta2 * self.steps_per_mm]

    def get_calibration(self):
        pass
    
def load_kinematics(toolhead, config):
    return MorganScaraKinematics(toolhead, config)
