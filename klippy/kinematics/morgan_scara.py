import math, logging
import stepper, mathutil, chelper

class MorganScaraKinematics:
    def __init__(self, toolhead, config):
        # Read config
        self.link_a = config.getfloat('link_a_length', above=0.)
        self.link_b = config.getfloat('link_b_length', above=0.)
        stepper_configs = [config.getsection('stepper_' + s) for s in 'abz']

        # Link A (proximal arm segment)
        rail_a = stepper.PrinterRail(stepper_configs[0],
                                     #need_position_minmax = True,
                                     units_in_radians = True)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_a.setup_itersolve('morgan_scara_stepper_alloc',
                              'a', self.link_a, self.link_b)

        # Link B (distal arm segment)
        rail_b = stepper.PrinterRail(stepper_configs[1],
                                     #need_position_minmax = True,
                                     units_in_radians = True)
        b_endstop = rail_b.get_homing_info().position_endstop
        rail_b.setup_itersolve('morgan_scara_stepper_alloc',
                              'b', self.link_a, self.link_b)

        # Elevator
        rail_z = stepper.LookupMultiRail(stepper_configs[2])
        rail_z.setup_itersolve('cartesian_stepper_alloc', 'z')

        self.steppers = [rail_a, rail_b] + rail_z.get_steppers()
        self.rails = [rail_a]

        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off)

        # Setup stepper max halt velocity
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)

        #for rail in self.rails:
        #rail.set_max_jerk(9999999.9, 9999999.9)

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        # Setup boundary checks
        self.need_home = True
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

    def get_steppers(self, flags=""):
        #if flags == "Z":
        #           return self.rails[-1].get_steppers()
        return list(self.steppers)

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
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.
        homing_state.home_rails(self.rails, forcepos, self.home_position)

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
