from joy import *
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
from math import pi, atan2, sqrt

import proj

# Calculate the angle of the line from point1 to point2
def calculate_angle(point1, point2):
    return atan2(point2[1] - point1[1], point2[0] - point1[0])

# Convert radians to degrees
def radians_to_degrees(radians):
    return radians / pi * 180

# Signum function: 1 if positive, -1 is negative, 0 if zero
def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

# Clip value to lower and upper bounds
def clip(value, lower, upper):
    return min(max(value, lower), upper)

# Convert l and r values into robot offset (positive is to right of line)
# This code partially assumes the robot is oriented more or less properly
# This code currently does no low-pass filtering to remove noise
# XXX The sqrts should probably taken on the l and r comparents individually
def calculate_offset(l, r):
    maximum = 255   # maximum reading for f & b values
    close = 30      # minimum reading for f & b values when robot is straddling
    straddle = 90   # minimum reading for min(f, b) when straddling
    if l == 0 and r == 0:
        raw = 0     # No information, so drive straight!
                    # (maybe I should return last offset and reset to zero on turn?)
    elif l > close and r > close: # and l + r > 2 * straddle: # XXX could help?
        # Robot is straddling line: estimate offset as difference
        raw = l - r
    else:
        # Robot is to one side of line: estimate offset as difference from maximum
        if l > r:
            raw = 2 * maximum - (l + r) # Robot is to right of line (positive offset)
        else:
            raw = (l + r) - 2 * maximum # Robot is to left of line (negative offset)
    # Take square root to convert inverse squared law to linear offset
    return sgn(raw) * sqrt(abs(raw))

# PID controller
# Value returned is proportional to amount we should turn to course correct
integral = 0
noffsets = 2
offsets = [0] * noffsets
def pid(offset):
    global integral, noffsets, offsets
    # Parameters to tune
    k_p = 0.05
    k_i = 0.00
    k_i_limit = 20    # Maximum absolute value for integral to prevent windup
    k_d = 0.00
    k_d_limit = 1     # Maximum absolute value for derivative in case of glitch
    # Calculate integral term (sum of offsets)
    integral = clip(integral + offset, -k_i_limit, k_i_limit)
    # Calculate derivative term (how much the offset has changed recently)
    derivative = clip((sum(offsets) / noffsets) - offset,
                                        -k_d_limit, k_d_limit)
    offsets = offsets[1:] + [offset]
    # Calculate PID controller value
    return k_p * offset + k_i * integral + k_d * derivative

# Reset PID controller after executing a turn
def reset_pid():
    global integral, noffsets, offsets
    integral = 0
    offsets = [0] * noffsets
# Toggle for if we should run the automatic navigation
auto_toggle = False

class SensorPlan( Plan ):
    """
    SensorPlan is a concrete Plan subclass that uses the self.app's
    remote plan to read and decode WayPoint Task sensor and waypoint
    updates.
    """
    def __init__( self, app, peer, robot_app, *arg, **kw ):
        Plan.__init__(self, app, *arg, **kw )
        self.sock = None
        self.peer = peer
        self.robot_app = robot_app
        self.lastSensorReading = None
        self.current_waypoint = 0
        self.nwaypoints = 4
        self.waypoints = [None] * self.nwaypoints

    def _connect( self ):
        s = socket(AF_INET, SOCK_STREAM)
        try:
             s.connect( self.peer )
        except SocketError, se:
             progress("Failed to connect: "+str(se))
             return
        s.setblocking(0)
        self.sock = s

    def stop( self ):
        if self.sock is not None:
            self.sock.close()
        self.sock = None

    def behavior( self ):
        global auto_toggle
        # message buffer is empty
        msg = ''
        while True:
            # if not connected --> try to connect
            if self.sock is None:
                self._connect()
            # if not connected --> sleep for a bit
            if self.sock is None:
                yield self.forDuration(0.1)
                continue
            # if message buffer is empty --> fill it from socket
            # or wait for more data
            if len(msg)==0:
                # receive an update / skip
                try:
                    msg = self.sock.recv(1024)
                except SocketError, se:
                    # If there was no data on the socket
                    # --> not a real error, else
                    if se.errno != 11:
                        progress("Connection failed: "+str(se))
                        self.sock.close()
                        self.sock = None
                    yield
                    continue
            # Find the end of the first message in the buffer
            pfx = msg.find('}')
            if pfx<0:
                progress('JSON parse error: "%s"' % msg)
                msg = ''
                continue
            # Parse the first message
            dic = json_loads(msg[:pfx+1])
            # Remove it from the buffer
            msg = msg[pfx+1:]
            ts = self.app.now
            self.lastSensorReading = (ts, dic['f'], dic['b'])

            # Handle driving between waypoints
            offset = calculate_offset(self.lastSensorReading[1],
                                                                self.lastSensorReading[2])
            speed = 0.5
            turn = min(max(pid(offset), -1.0), 1.0) # * speed ?
            progress('Offset: ' + str(offset))
            progress('Turn:     ' + str(turn))
            if self.robot_app != None and auto_toggle:
                self.robot_app.set_turn_and_speed(speed, turn)
            else:
                self.robot_app.set_turn_and_speed(0, 0)

            # Handle waypoint update
            if 'w' in dic:
                # Store waypoint locations in array
                n = self.nwaypoints - len(dic['w'])
                for i in range(n, self.nwaypoints):
                    self.waypoints[i] = dic['w'][i-n]
                # If we hit a waypoint, update current waypoint and go into turn mode
                # XXX But if we just hit auto button we need to do this too
                if n > self.current_waypoint:
                    self.current_waypoint = n
                    # I think at this point we hit current_waypoint and we need to drive
                    # to current_waypoint+1
                    # We will hit an "auto" button if we were driving manually
                    # XXX Go into turn mode (calculate angle)
                    if self.current_waypoint == 0:
                        previous_angle = pi/2
                    elif self.current_waypoint == 3:
                        self.robot_app.set_turn_and_speed(0, 0) # stop robot at last waypoint
                    else:
                        previous_angle = calculate_angle(
                            self.waypoints[self.current_waypoint-1],
                            self.waypoints[self.current_waypoint])
                    current_angle = calculate_angle(
                        self.waypoints[self.current_waypoint],
                        self.waypoints[self.current_waypoint+1])
                    # XXX We may need to add or subtract 2*pi
                    turn_angle = current_angle - previous_angle
                    progress("Previous angle: " + str(radians_to_degrees(previous_angle)))
                    progress("Current angle:    " + str(radians_to_degrees(current_angle)))
                    progress("Turn robot " + str(radians_to_degrees(turn_angle)))
                    # XXX After turning, go into drive mode using offset and PID controller
            # If no more messages in buffer --> wait for a bit
            # XXX We can turn and assume the angle will change by a fixed amount each time
            if not msg:
                yield self.forDuration(0.1)

class WaypointSensorApp( JoyApp ):
    def __init__(self, spec, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)
        self.spec = spec
        self.manual_controller = ManualController(self.robot)

    def onStart( self ):
        self.output = self.setterOf(self.spec)
        # Set up the sensor receiver plan
        #self.sensor = SensorPlan(self,("141.213.30.218", 8080), self.manual_controller)
        self.sensor = SensorPlan(self,("127.0.0.1", 8080), self.manual_controller)
        self.sensor.start()

    def onEvent( self, evt ):
        # Punt to superclass
        # this is here to remind you to override it
        if evt.type is KEYDOWN and evt.key is K_p:
            progress('Sensor: ' + str(self.sensor.lastSensorReading))
            progress('Current waypoint: ' + str(self.sensor.current_waypoint))
            progress('Waypoints: ' + ','.join(map(str, self.sensor.waypoints)))
            offset = calculate_offset(self.sensor.lastSensorReading[1],
                                                                self.sensor.lastSensorReading[2])
            progress('Offset: ' + str(offset))
            progress('PID output: ' + str(pid(offset)))
        if evt.type is KEYDOWN and evt.key is K_q:
            self.manual_controller.set_turn_and_speed(0, 0)
        if not self.manual_controller.onEvent(evt):
            return super( WaypointSensorApp, self ).onEvent(evt)

    def onStop( self ):
        self.sensor.stop()
        return super( WaypointSensorApp, self ).onStop()

class ManualController:

    MOTOR_LEFT = 0
    MOTOR_RIGHT = 1
    MOTOR_LASER = 2

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

    def __init__(self, robot):
        self.robot = robot
        self.turning_rate = 0.0
        self.moving_rate = 0.0

        self.motor_left = self.robot.items()[self.MOTOR_LEFT][1]
        self.motor_right = self.robot.items()[self.MOTOR_RIGHT][1]
        self.motor_laser = self.robot.items()[self.MOTOR_LASER][1]
        self.motor_left.set_mode(1)
        self.motor_right.set_mode(1)
        self.motor_laser.set_mode(1)
        self.motor_left.set_torque(0)
        self.motor_right.set_torque(0)
        self.motor_laser.set_torque(0)

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

    def onEvent(self, evt):
        global auto_toggle
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
                if kind == 'record0' and evt.value != 0:
                    auto_toggle = not auto_toggle
                    return True
                else:
                    return False

            # Computation the new motor input
            self.values[0][kind] = (evt.value * params[1][0]) / params[0]
            self.values[1][kind] = (evt.value * params[1][1]) / params[0]

            # Sum of all motor inputs and range constrain
            forward_rate_sum = sum(self.values[0].itervalues())
            forward_rate_speed = min(max(forward_rate_sum, -1.0), 1.0)
            turn_rate_sum = sum(self.values[1].itervalues())
            turn_rate_speed = min(max(turn_rate_sum, -1.0), 1.0)

            self.set_turn_and_speed(forward_rate_speed, turn_rate_speed)
            return True

        return False

    def compute_torques(self):
        motor_left_torq = self.fix_torque_range((self.moving_rate + self.turning_rate)/2)
        motor_right_torq = self.fix_torque_range(-(self.moving_rate - self.turning_rate)/2)
        return (motor_left_torq, motor_right_torq)

    def set_motor_speeds(self):
        motor_left_torq, motor_right_torq = self.compute_torques()
        self.motor_left.pna.mem_write_fast(self.motor_left.mcu.moving_speed, motor_left_torq)
        self.motor_right.pna.mem_write_fast(self.motor_right.mcu.moving_speed, motor_right_torq)


if __name__=="__main__":
    print """
    Running the waypoint sensor demo

    Connects to waypoint application and reads sensor.

    The waypoint sensor send JSON maps with keys:
    'f', 'b' : front and back sensor values
    'w' : list of lists. Each sub-list is of length 2. List of waypoint
        coordinates, including the next waypoint. Each time the next
        waypoint changes, it means the previous waypoint was reached.
    """
    app=WaypointSensorApp("#output ", robot=dict(count=3)) #, port='/dev/ttyACM0'))
    app.run()
