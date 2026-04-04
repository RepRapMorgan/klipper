import math


class MorganScaraCalibration:
    def __init__(self, l1, l2, column_x, column_y, d_limit,
                 endstops, stepdists):
        self.l1 = l1
        self.l2 = l2
        self.column_x = column_x
        self.column_y = column_y
        self.d_limit = d_limit
        self.endstops = endstops
        self.stepdists = stepdists
        self.abs_endstops = list(endstops)

    def coordinate_descent_params(self, is_extended):
        adj_params = ('l1', 'l2', 'endstop_a', 'endstop_b', 'endstop_z')
        if is_extended:
            adj_params += ('column_x', 'column_y', 'd_limit')
        params = {
            'l1': self.l1,
            'l2': self.l2,
            'column_x': self.column_x,
            'column_y': self.column_y,
            'd_limit': self.d_limit,
        }
        for i, axis in enumerate('abz'):
            params['endstop_' + axis] = self.endstops[i]
            params['stepdist_' + axis] = self.stepdists[i]
        return adj_params, params

    def new_calibration(self, params):
        endstops = [params['endstop_' + a] for a in 'abz']
        stepdists = [params['stepdist_' + a] for a in 'abz']
        return MorganScaraCalibration(
            params['l1'], params['l2'],
            params['column_x'], params['column_y'], params['d_limit'],
            endstops, stepdists)

    def _inverse_kinematics(self, x, y):
        x -= self.column_x
        y -= self.column_y
        r_squared = x * x + y * y
        d = (r_squared - self.l1 * self.l1 - self.l2 * self.l2) \
            / (2.0 * self.l1 * self.l2)
        d = min(max(d, -self.d_limit), self.d_limit)
        psi = math.copysign(math.atan2(math.sqrt(max(0.0, 1.0 - d * d)), d),
                           -1.0)
        theta = math.atan2(y, x) - math.atan2(self.l2 * math.sin(psi),
                                               self.l1 + self.l2 * math.cos(psi))
        return theta, theta + psi

    def _forward_kinematics(self, theta, psi_theta):
        return [
            self.column_x + self.l1 * math.cos(theta)
            + self.l2 * math.cos(psi_theta),
            self.column_y + self.l1 * math.sin(theta)
            + self.l2 * math.sin(psi_theta),
        ]

    def get_position_from_stable(self, stable_position):
        spos = [ea - sp * sd
                for ea, sp, sd in zip(self.abs_endstops, stable_position,
                                      self.stepdists)]
        x_pos, y_pos = self._forward_kinematics(spos[0], spos[1])
        return [x_pos, y_pos, spos[2]]

    def calc_stable_position(self, coord):
        theta, psi_theta = self._inverse_kinematics(coord[0], coord[1])
        spos = [theta, psi_theta, coord[2]]
        return [(ep - sp) / sd
                for sd, ep, sp in zip(self.stepdists, self.abs_endstops, spos)]

    def save_state(self, configfile):
        configfile.set('printer', 'inner_arm_length', "%.6f" % (self.l1,))
        configfile.set('printer', 'outer_arm_length', "%.6f" % (self.l2,))
        configfile.set('printer', 'column_x', "%.6f" % (self.column_x,))
        configfile.set('printer', 'column_y', "%.6f" % (self.column_y,))
        configfile.set('printer', 'd_limit', "%.6f" % (self.d_limit,))
        for i, axis in enumerate('abz'):
            configfile.set('stepper_' + axis, 'position_endstop',
                           "%.6f" % (self.endstops[i],))
        gcode = configfile.get_printer().lookup_object("gcode")
        gcode.respond_info(
            "stepper_a: position_endstop: %.6f\n"
            "stepper_b: position_endstop: %.6f\n"
            "stepper_z: position_endstop: %.6f\n"
            "inner_arm_length: %.6f outer_arm_length: %.6f\n"
            "column_x: %.6f column_y: %.6f d_limit: %.6f"
            % (self.endstops[0], self.endstops[1], self.endstops[2],
               self.l1, self.l2, self.column_x, self.column_y, self.d_limit))


