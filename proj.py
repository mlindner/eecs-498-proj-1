from time import sleep
from joy import *


# TODO May want to constrain motors to be only certain names to prevent motors
# from being accidentally switched

MOTOR_LEFT = 0
MOTOR_RIGHT = 2
MOTOR_LASER = 1

sleep(0.5)

class Controller( JoyApp ):
    # Events names generated from the concatenation of event kind + event index
    # The format is:
    # 'eventname': (sensor_max_value, (forward_rate_max, turn_rate_max))
    events = {
        'play0':    (127, ( 1.0,  0.0)),
        'stop0':    (127, (-1.0,  0.0)),
        'forward0': (127, ( 0.0,  1.0)),
        'rewind0':  (127, ( 0.0, -1.0)),
        'slider1':  (127, ( 1.0,  0.0)),
        'slider2':  (127, (-1.0,  0.0)),
        'slider3':  (127, ( 0.0, -1.0)),
        'slider4':  (127, ( 0.0,  1.0)),
    }

    # Stored values for all event types to be summed for resultant motor speed
    values = [
        {key: 0 for key in events},
        {key: 0 for key in events},
    ]

    def __init__(self, spec, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)
        self.spec = spec

    def set_turn_rate(self, val):
        """
        Sets turning rate of the robot.

        INPUT:
            val -- float from -1.0 to 1.0 where -1.0 is full left and 1.0 is
            full right
        """
        self.turning_rate = val
        self.set_motor_speeds()

    def set_speed(self, val):
        """
        Sets speed of the robot.

        INPUT:
            val -- unit from -1.0 to 1.0
        """
        self.moving_rate = val
        self.set_motor_speeds()

    def set_turn_and_speed(self, speed, turn):
        self.moving_rate = speed
        self.turning_rate = turn
        self.set_motor_speeds()

    def fix_torque_range(self, val):
        """
        INPUT:
            val -- float from -1.0 to 1.0
        OUTPUT:
            val -- integer from 0 - 1023 or 1024 to 2047
        """
        val = int(val * 1023)
        if(val < 0):
            val = -val + 1024
        return val

    def onStart(self):
        self.output = self.setterOf(self.spec)
        self.turning_rate = 0.0
        self.moving_rate = 0.0

        self.motor_left = self.robot.items()[MOTOR_LEFT][1]
        self.motor_right = self.robot.items()[MOTOR_RIGHT][1]
        self.motor_laser = self.robot.items()[MOTOR_LASER][1]
        self.motor_left.set_mode(1)
        self.motor_right.set_mode(1)
        self.motor_laser.set_mode(1)
        self.motor_left.set_torque(0)
        self.motor_right.set_torque(0)
        self.motor_laser.set_torque(0)

    def onEvent(self, evt):
        # We only care about midi events right now
        if evt.type == MIDIEVENT:
            # Generate event name
            kind = evt.kind + str(evt.index)
            # Try in case we got an event we havent set
            try:
                print(kind)
                print(self.events[kind])
                params = self.events[kind]
            except KeyError:
                return JoyApp.onEvent(self, evt)

            # Computation the new motor input
            self.values[0][kind] = (evt.value * params[1][0]) / params[0]
            self.values[1][kind] = (evt.value * params[1][1]) / params[0]

            # Sum of all motor inputs and range constrain
            forward_rate_sum = sum(self.values[0].itervalues())
            forward_rate_speed = min(max(motor_left_sum, -1.0), 1.0)
            turn_rate_sum = sum(self.values[1].itervalues())
            turn_rate_speed = min(max(motor_right_sum, -1.0), 1.0)

            set_turn_and_speed(forward_rate_speed, turn_rate_speed)

    def compute_torques(self):
        motor_left_torq = self.fix_torque_range((self.moving_rate + self.turning_rate)/2)
        motor_right_torq = self.fix_torque_range(-(self.moving_rate - self.turning_rate)/2)
        return (motor_left_torq, motor_right_torq)

    def set_motor_speeds(self):
        motor_left_torq, motor_right_torq = self.compute_torques()
        self.motor_left.pna.mem_write_fast(self.motor_left.mcu.moving_speed, motor_left_torq)
        self.motor_right.pna.mem_write_fast(self.motor_right.mcu.moving_speed, motor_right_torq)


def start_app():
    sleep(1)
    app = Controller("#output ", robot=dict(count=3, port='/dev/ttyACM0'))
    app.run()
    return app

if __name__ == '__main__':
    start_app()
