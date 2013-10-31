from time import sleep
from joy import *


# TODO May want to constrain motors to be only certain names to prevent motors
# from being accidentally switched

sleep(0.5)

class Controller( JoyApp ):
    # Events names generated from the concatenation of event kind + event index
    # The format is:
    # 'eventname': (sensor_max_value, (left_motor_dir, right_motor_dir))
    events = {
        'play0': (127, (1, 1)),
        'stop0': (127, (-1, -1)),
        'forward0': (127, (1, -1)),
        'rewind0': (127, (-1, 1)),
        'slider1': (127, (1, 1)),
        'slider2': (127, (-1, -1)),
        'slider3': (127, (-1, 1)),
        'slider4': (127, (1, -1)),
    }

    # Stored values for all event types to be summed for resultant motor speed
    values = [
        {key: 0 for key in events},
        {key: 0 for key in events},
    ]

    def __init__(self, spec, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)
        self.spec = spec

    def onStart(self):
        self.output = self.setterOf(self.spec)
        self.motor1 = self.robot.items()[0][1]
        self.motor2 = self.robot.items()[1][1]
        self.motor3 = self.robot.items()[2][1]
        self.motor1.set_mode(1)
        self.motor2.set_mode(1)
        self.motor3.set_mode(1)
        self.motor1.set_torque(0)
        self.motor2.set_torque(0)
        self.motor3.set_torque(0)

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
            self.values[0][kind] = (evt.value * 1023 * params[1][0]) / params[0]
            self.values[1][kind] = (evt.value * 1023 * params[1][1]) / params[0]

            # Sum of all motor inputs and range constrain
            motor_left_sum = sum(self.values[0].itervalues())
            motor_left_speed = min(max(motor_left_sum, -1023), 1023)
            motor_right_sum = sum(self.values[1].itervalues())
            motor_right_speed = min(max(motor_right_sum, -1023), 1023)

            # Invert right motor because our motors are 180 degrees seperated
            motor_right_speed = -motor_right_speed
            # Setup the motor's wacky way of doing turn direction
            if(motor_right_speed < 0):
                motor_right_speed = -motor_right_speed + 1024
            if(motor_left_speed < 0):
                motor_left_speed = -motor_left_speed + 1024

            # Set motor speeds
            self.motor1.pna.mem_write_fast(self.motor1.mcu.moving_speed, motor_left_speed)
            self.motor2.pna.mem_write_fast(self.motor2.mcu.moving_speed, motor_right_speed)

app = Controller("#output ", robot=dict(count=3, port='/dev/ttyACM0'))
app.run()
