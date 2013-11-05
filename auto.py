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
    raw = 0  # No information, so drive straight!
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
  k_i_limit = 20  # Maximum absolute value for integral to prevent windup
  k_d = 0.00
  k_d_limit = 1   # Maximum absolute value for derivative in case of glitch
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

class SensorPlan( Plan ):
  """
  SensorPlan is a concrete Plan subclass that uses the self.app's
  remote plan to read and decode WayPoint Task sensor and waypoint
  updates.
  """
  def __init__( self, app, peer, *arg, **kw ):
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.peer = peer
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
      #   or wait for more data
      if len(msg)==0:
        # receive an update / skip
        try:
          msg = self.sock.recv(1024)
        except SocketError, se:
          # If there was no data on the socket
          #   --> not a real error, else
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
      progress('Turn:   ' + str(turn))
      if (robot_app != None):
        robot_app.set_turn_and_speed(speed, turn)

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
            robot_app.set_turn_and_speed(0, 0) # stop robot at last waypoint
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
          progress("Current angle:  " + str(radians_to_degrees(current_angle)))
          progress("Turn robot " + str(radians_to_degrees(turn_angle)))
          # XXX After turning, go into drive mode using offset and PID controller
      # If no more messages in buffer --> wait for a bit
      # XXX We can turn and assume the angle will change by a fixed amount each time
      if not msg:
        yield self.forDuration(0.1)

class WaypointSensorApp( JoyApp ):
  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self,("141.213.30.218",8080))
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
      robot_app.set_turn_and_speed(0, 0)
    return super( WaypointSensorApp, self ).onEvent(evt)

  def onStop( self ):
    self.sensor.stop()
    return super( WaypointSensorApp, self ).onStop()

if __name__=="__main__":
  global robot_app
  robot_app = None
  print """
  Running the waypoint sensor demo

  Connects to waypoint application and reads sensor.

  The waypoint sensor send JSON maps with keys:
  'f', 'b' : front and back sensor values
  'w' : list of lists. Each sub-list is of length 2. List of waypoint
    coordinates, including the next waypoint. Each time the next
    waypoint changes, it means the previous waypoint was reached.
  """
  robot_app = proj.start_app()
  app=WaypointSensorApp()
  app.run()
