package org.mavlink;
import org.unirail.AdHoc.*;
public class python_array_test{
 public static class CommunicationChannel extends AdvProtocol implements GroundControl.CommunicationInterface, MicroAirVehicle.CommunicationInterface {}
public static class GroundControl implements  InKT, InCS{
public interface CommunicationInterface extends GroundControlHandledPacks, CommonPacks {}public interface GroundControlHandledPacks  { 
/**
The heartbeat message shows that a system or component is present and responding. The type and autopilot
fields (along with the message component id), allow the receiving system to treat further messages from
this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice
is documented at https://mavlink.io/en/services/heartbeat.htm*/
@id(0) public static class HEARTBEAT{
/**
Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter,
etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference
to component id for identifying the component type*/
MAV_TYPE type;
MAV_AUTOPILOT autopilot;//Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
MAV_MODE_FLAG base_mode;//System mode bitmap.
@I  int  custom_mode;//A bitfield for use for autopilot-specific flags
MAV_STATE system_status;//System status flag.
@I byte  mavlink_version;//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versio
}

/**
The general system state. If the system is following the MAVLink standard, the system state is mainly
defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and
locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position
setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined
the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents
the internal navigation state machine. The system status shows whether the system is currently active
or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered
to be active, but should start emergency procedures autonomously. After a failure occurred it should
first move from active to critical to allow manual intervention and then move to emergency after a certain
timeout*/
@id(1) public static class SYS_STATUS{
/**
Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
present*/
MAV_SYS_STATUS_SENSOR onboard_control_sensors_present;
/**
Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1:
enabled*/
MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled;
/**
Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
Value of 1: healthy*/
MAV_SYS_STATUS_SENSOR onboard_control_sensors_health;
@I short  load;//Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
@I short  voltage_battery;//Battery voltage, UINT16_MAX: Voltage not sent by autopilot
 short  current_battery;//Battery current, -1: Current not sent by autopilot
 byte  battery_remaining;//Battery energy remaining, -1: Battery remaining energy not sent by autopilot
/**
Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
on reception on the MAV*/
@I short  drop_rate_comm;
/**
Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
on reception on the MAV*/
@I short  errors_comm;
@I short  errors_count1;//Autopilot-specific errors
@I short  errors_count2;//Autopilot-specific errors
@I short  errors_count3;//Autopilot-specific errors
@I short  errors_count4;//Autopilot-specific errors
}

/**
The system time is the time of the master clock, typically the computer clock of the main onboard computer*/
@id(2) public static class SYSTEM_TIME{
@I long  time_unix_usec;//Timestamp (UNIX epoch time).
@I  int  time_boot_ms;//Timestamp (time since system boot).
}

/**
A ping message either requesting or responding to a ping. This allows to measure the system latencies,
including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.htm*/
@id(4) public static class PING{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I  int  seq;//PING sequence
/**
0: request ping from all receiving systems. If greater than 0: message is a ping response and number is
the system id of the requesting syste*/
@I byte  target_system;
/**
0: request ping from all receiving components. If greater than 0: message is a ping response and number
is the component id of the requesting component*/
@I byte  target_component;
}

/**
Request to control this MAV*/
@id(5) public static class CHANGE_OPERATOR_CONTROL{
@I byte  target_system;//System the GCS requests control for
@I byte  control_request;//0: request control of this MAV, 1: Release control of this MAV
/**
0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
message indicating an encryption mismatch*/
@I byte  version;
/**
Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
characters may involve A-Z, a-z, 0-9, and "!?,.-*/
@__(25) String  passkey;
}

/**
Accept / deny control of this MAV*/
@id(6) public static class CHANGE_OPERATOR_CONTROL_ACK{
@I byte  gcs_system_id;//ID of the GCS this message 
@I byte  control_request;//0: request control of this MAV, 1: Release control of this MAV
/**
0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
contro*/
@I byte  ack;
}

/**
Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
so transmitting the key requires an encrypted channel for true safety*/
@id(7) public static class AUTH_KEY{
@__(32) String  key;//key
}

/**
Status generated in each node in the communication chain and injected into MAVLink stream.*/
@id(8) public static class LINK_NODE_STATUS{
@I long  timestamp;//Timestamp (time since system boot).
@I byte  tx_buf;//Remaining free transmit buffer space
@I byte  rx_buf;//Remaining free receive buffer space
@I  int  tx_rate;//Transmit rate
@I  int  rx_rate;//Receive rate
@I short  rx_parse_err;//Number of bytes that could not be parsed correctly.
@I short  tx_overflows;//Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
@I short  rx_overflows;//Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
@I  int  messages_sent;//Messages sent
@I  int  messages_received;//Messages received (estimated from counting seq)
@I  int  messages_lost;//Messages lost (estimated from counting seq)
}

/**
Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition
for the overall aircraft, not only for one component*/
@id(11) public static class SET_MODE{
@I byte  target_system;//The system setting the mode
MAV_MODE base_mode;//The new base mode.
@I  int  custom_mode;//The new autopilot-specific mode. This field can be ignored by an autopilot.
}

/**
value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation
of QGroundControl and IMU code*/
@id(20) public static class PARAM_REQUEST_READ{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as strin*/
@__(16) String  param_id;
 short  param_index;//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
}

/**
Request all parameters of this component. After this request, all parameters are emitted. The parameter
microservice is documented at https://mavlink.io/en/services/parameter.htm*/
@id(21) public static class PARAM_REQUEST_LIST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
}

/**
Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
the recipient to keep track of received parameters and allows him to re-request missing parameters after
a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.htm*/
@id(22) public static class PARAM_VALUE{
/**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as strin*/
@__(16) String  param_id;
 float  param_value;//Onboard parameter value
MAV_PARAM_TYPE param_type;//Onboard parameter type.
@I short  param_count;//Total number of onboard parameters
@I short  param_index;//Index of this onboard parameter
}

/**
Set a parameter value (write new value to permanent storage). IMPORTANT: The receiving component should
acknowledge the new parameter value by sending a PARAM_VALUE message to all communication partners. This
will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS
did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
The parameter microservice is documented at https://mavlink.io/en/services/parameter.htm*/
@id(23) public static class PARAM_SET{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as strin*/
@__(16) String  param_id;
 float  param_value;//Onboard parameter value
MAV_PARAM_TYPE param_type;//Onboard parameter type.
}

/**
The global position, as returned by the Global Positioning System (GPS). This is
NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.*/
@id(24) public static class GPS_RAW_INT{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
GPS_FIX_TYPE fix_type;//GPS fix type.
  int  lat;//Latitude (WGS84, EGM96 ellipsoid)
  int  lon;//Longitude (WGS84, EGM96 ellipsoid)
/**
Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition
to the WGS84 altitude*/
  int  alt;
@I short  eph;//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
@I short  epv;//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
@I short  vel;//GPS ground speed. If unknown, set to: UINT16_MAX
/**
Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
unknown, set to: UINT16_MA*/
@I short  cog;
@I byte  satellites_visible;//Number of satellites visible. If unknown, set to 255
@I_()   int  alt_ellipsoid;//Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
@I_  int  h_acc;//Position uncertainty. Positive for up.
@I_  int  v_acc;//Altitude uncertainty. Positive for up.
@I_  int  vel_acc;//Speed uncertainty. Positive for up.
@I_  int  hdg_acc;//Heading / track uncertainty
}

/**
The positioning status, as reported by GPS. This message is intended to display status information about
each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
This message can contain information for up to 20 satellites*/
@id(25) public static class GPS_STATUS{
@I byte  satellites_visible;//Number of satellites visible
@I @__( 20 )  byte  satellite_prn;//Global satellite ID
@I @__( 20 )  byte  satellite_used;//0: Satellite not used, 1: used for localization
@I @__( 20 )  byte  satellite_elevation;//Elevation (0: right on top of receiver, 90: on the horizon) of satellite
@I @__( 20 )  byte  satellite_azimuth;//Direction of satellite, 0: 0 deg, 255: 360 deg.
@I @__( 20 )  byte  satellite_snr;//Signal to noise ratio of satellite
}

/**
The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
the described unit*/
@id(26) public static class SCALED_IMU{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 short  xacc;//X acceleration
 short  yacc;//Y acceleration
 short  zacc;//Z acceleration
 short  xgyro;//Angular speed around X axis
 short  ygyro;//Angular speed around Y axis
 short  zgyro;//Angular speed around Z axis
 short  xmag;//X Magnetic field
 short  ymag;//Y Magnetic field
 short  zmag;//Z Magnetic field
@I_()  short  temperature;//Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C)
}

/**
The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should
always contain the true raw values without any scaling to allow data capture and system debugging*/
@id(27) public static class RAW_IMU{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 short  xacc;//X acceleration (raw)
 short  yacc;//Y acceleration (raw)
 short  zacc;//Z acceleration (raw)
 short  xgyro;//Angular speed around X axis (raw)
 short  ygyro;//Angular speed around Y axis (raw)
 short  zgyro;//Angular speed around Z axis (raw)
 short  xmag;//X Magnetic field (raw)
 short  ymag;//Y Magnetic field (raw)
 short  zmag;//Z Magnetic field (raw)
@I_ byte  id;//Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0
@I_()  short  temperature;//Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C)
}

/**
The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
sensor. The sensor values should be the raw, UNSCALED ADC values*/
@id(28) public static class RAW_PRESSURE{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 short  press_abs;//Absolute pressure (raw)
 short  press_diff1;//Differential pressure 1 (raw, 0 if nonexistent)
 short  press_diff2;//Differential pressure 2 (raw, 0 if nonexistent)
 short  temperature;//Raw Temperature measurement (raw)
}

/**
The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
are as specified in each field*/
@id(29) public static class SCALED_PRESSURE{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  press_abs;//Absolute pressure
 float  press_diff;//Differential pressure 1
 short  temperature;//Temperature
}

/**
The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).*/
@id(30) public static class ATTITUDE{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  roll;//Roll angle (-pi..+pi)
 float  pitch;//Pitch angle (-pi..+pi)
 float  yaw;//Yaw angle (-pi..+pi)
 float  rollspeed;//Roll angular speed
 float  pitchspeed;//Pitch angular speed
 float  yawspeed;//Yaw angular speed
}

/**
The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
@id(31) public static class ATTITUDE_QUATERNION{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  q1;//Quaternion component 1, w (1 in null-rotation)
 float  q2;//Quaternion component 2, x (0 in null-rotation)
 float  q3;//Quaternion component 3, y (0 in null-rotation)
 float  q4;//Quaternion component 4, z (0 in null-rotation)
 float  rollspeed;//Roll angular speed
 float  pitchspeed;//Pitch angular speed
 float  yawspeed;//Yaw angular speed
@I_()  @__( 4 )  float  repr_offset_q;//in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.
}

/**
The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
Z-axis down (aeronautical frame, NED / north-east-down convention*/
@id(32) public static class LOCAL_POSITION_NED{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  x;//X Position
 float  y;//Y Position
 float  z;//Z Position
 float  vx;//X Speed
 float  vy;//Y Speed
 float  vz;//Z Speed
}

/**
The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
 is designed as scaled integer message since the resolution of float is not sufficient.*/
@id(33) public static class GLOBAL_POSITION_INT{
@I  int  time_boot_ms;//Timestamp (time since system boot).
  int  lat;//Latitude, expressed
  int  lon;//Longitude, expressed
  int  alt;//Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
  int  relative_alt;//Altitude above ground
 short  vx;//Ground X Speed (Latitude, positive north)
 short  vy;//Ground Y Speed (Longitude, positive east)
 short  vz;//Ground Z Speed (Altitude, positive down)
@I short  hdg;//Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
}

/**
The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
inactive should be set to UINT16_MAX*/
@id(34) public static class RC_CHANNELS_SCALED{
@I  int  time_boot_ms;//Timestamp (time since system boot).
/**
Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN,
1 = AUX*/
@I byte  port;
 short  chan1_scaled;//RC channel 1 value scaled.
 short  chan2_scaled;//RC channel 2 value scaled.
 short  chan3_scaled;//RC channel 3 value scaled.
 short  chan4_scaled;//RC channel 4 value scaled.
 short  chan5_scaled;//RC channel 5 value scaled.
 short  chan6_scaled;//RC channel 6 value scaled.
 short  chan7_scaled;//RC channel 7 value scaled.
 short  chan8_scaled;//RC channel 8 value scaled.
@I byte  rssi;//Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown
}

/**
The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters
might violate this specification*/
@id(35) public static class RC_CHANNELS_RAW{
@I  int  time_boot_ms;//Timestamp (time since system boot).
/**
Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN,
1 = AUX*/
@I byte  port;
@I short  chan1_raw;//RC channel 1 value.
@I short  chan2_raw;//RC channel 2 value.
@I short  chan3_raw;//RC channel 3 value.
@I short  chan4_raw;//RC channel 4 value.
@I short  chan5_raw;//RC channel 5 value.
@I short  chan6_raw;//RC channel 6 value.
@I short  chan7_raw;//RC channel 7 value.
@I short  chan8_raw;//RC channel 8 value.
@I byte  rssi;//Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown
}

/**
Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the remote,
use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000
microseconds: 100%*/
@id(36) public static class SERVO_OUTPUT_RAW{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I  int  time_usec;
/**
Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN,
1 = AUX*/
@I byte  port;
@I short  servo1_raw;//Servo output 1 value
@I short  servo2_raw;//Servo output 2 value
@I short  servo3_raw;//Servo output 3 value
@I short  servo4_raw;//Servo output 4 value
@I short  servo5_raw;//Servo output 5 value
@I short  servo6_raw;//Servo output 6 value
@I short  servo7_raw;//Servo output 7 value
@I short  servo8_raw;//Servo output 8 value
@I_ short  servo9_raw;//Servo output 9 value
@I_ short  servo10_raw;//Servo output 10 value
@I_ short  servo11_raw;//Servo output 11 value
@I_ short  servo12_raw;//Servo output 12 value
@I_ short  servo13_raw;//Servo output 13 value
@I_ short  servo14_raw;//Servo output 14 value
@I_ short  servo15_raw;//Servo output 15 value
@I_ short  servo16_raw;//Servo output 16 value
}

/**
Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html.
If start and end index are the same, just send one waypoint*/
@id(37) public static class MISSION_REQUEST_PARTIAL_LIST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
 short  start_index;//Start index
 short  end_index;//End index, -1 by default (-1: send list to end). Else a valid index of the list
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
This message is sent to the MAV to write a partial list. If start index == end index, only one item will
be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
be REJECTED*/
@id(38) public static class MISSION_WRITE_PARTIAL_LIST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
 short  start_index;//Start index. Must be smaller / equal to the largest index of the current onboard list.
 short  end_index;//End index, equal or greater than start index.
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Message encoding a mission item. This message is emitted to announce
the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/services/mission.html.*/
@id(39) public static class MISSION_ITEM{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  seq;//Sequence
MAV_FRAME frame;//The coordinate system of the waypoint.
MAV_CMD command;//The scheduled action for the waypoint.
@I byte  current;//false:0, true:1
@I byte  autocontinue;//Autocontinue to next waypoint
 float  param1;//PARAM1, see MAV_CMD enum
 float  param2;//PARAM2, see MAV_CMD enum
 float  param3;//PARAM3, see MAV_CMD enum
 float  param4;//PARAM4, see MAV_CMD enum
 float  x;//PARAM5 / local: X coordinate, global: latitude
 float  y;//PARAM6 / local: Y coordinate, global: longitude
 float  z;//PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Request the information of the mission item with the sequence number seq. The response of the system to
this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.htm*/
@id(40) public static class MISSION_REQUEST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  seq;//Sequence
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Set the mission item with sequence number seq as current item. This means that the MAV will continue to
this mission item on the shortest path (not following the mission items in-between)*/
@id(41) public static class MISSION_SET_CURRENT{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  seq;//Sequence
}

/**
Message that announces the sequence number of the current active mission item. The MAV will fly towards
this mission item*/
@id(42) public static class MISSION_CURRENT{
@I short  seq;//Sequence
}

/**
Request the overall list of mission items from the system/component.*/
@id(43) public static class MISSION_REQUEST_LIST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
The GCS can then request the individual mission item based on the knowledge of the total number of waypoints*/
@id(44) public static class MISSION_COUNT{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  count;//Number of mission items in the sequence
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Delete all mission items at once.*/
@id(45) public static class MISSION_CLEAR_ALL{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
or (if the autocontinue on the WP was set) continue to the next waypoint*/
@id(46) public static class MISSION_ITEM_REACHED{
@I short  seq;//Sequence
}

/**
Acknowledgment message during waypoint handling. The type field states if this message is a positive ack
(type=0) or if an error happened (type=non-zero)*/
@id(47) public static class MISSION_ACK{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
MAV_MISSION_RESULT type;//Mission result.
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Sets the GPS co-ordinates of the vehicle local origin (0,0,0) position. Vehicle should emit GPS_GLOBAL_ORIGIN
irrespective of whether the origin is changed. This enables transform between the local coordinate frame
and the global (GPS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings
are connected and the MAV should move from in- to outdoor*/
@id(48) public static class SET_GPS_GLOBAL_ORIGIN{
@I byte  target_system;//System ID
  int  latitude;//Latitude (WGS84)
  int  longitude;//Longitude (WGS84)
  int  altitude;//Altitude (MSL). Positive for up.
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I_ long  time_usec;
}

/**
Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local
position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message*/
@id(49) public static class GPS_GLOBAL_ORIGIN{
  int  latitude;//Latitude (WGS84)
  int  longitude;//Longitude (WGS84)
  int  altitude;//Altitude (MSL). Positive for up.
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I_ long  time_usec;
}

/**
Bind a RC channel to a parameter. The parameter should change according to the RC channel value.*/
@id(50) public static class PARAM_MAP_RC{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as strin*/
@__(16) String  param_id;
/**
Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
send -2 to disable any existing map for this rc_channel_index*/
 short  param_index;
/**
Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a potentiometer-knob
on the RC*/
@I byte  parameter_rc_channel_index;
 float  param_value0;//Initial parameter value
 float  scale;//Scale, maps the RC range [-1, 1] to a parameter value
/**
Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
on implementation*/
 float  param_value_min;
/**
Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
on implementation*/
 float  param_value_max;
}

/**
Request the information of the mission item with the sequence number seq. The response of the system to
this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.htm*/
@id(51) public static class MISSION_REQUEST_INT{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  seq;//Sequence
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
A broadcast message to notify any ground station or SDK if a mission, geofence or safe points have changed
on the vehicle*/
@id(52) public static class MISSION_CHANGED{
 short  start_index;//Start index for partial mission change (-1 for all items).
/**
End index of a partial mission change. -1 is a synonym for the last mission item (i.e. selects all items
from start_index). Ignore field if start_index=-1*/
 short  end_index;
@I byte  origin_sysid;//System ID of the author of the new mission.
MAV_COMPONENT origin_compid;//Compnent ID of the author of the new mission.
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
or competition regulations*/
@id(54) public static class SAFETY_SET_ALLOWED_AREA{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis
down*/
MAV_FRAME frame;
 float  p1x;//x position 1 / Latitude 1
 float  p1y;//y position 1 / Longitude 1
 float  p1z;//z position 1 / Altitude 1
 float  p2x;//x position 2 / Latitude 2
 float  p2y;//y position 2 / Longitude 2
 float  p2z;//z position 2 / Altitude 2
}

/**
Read out the safety zone the MAV currently assumes.*/
@id(55) public static class SAFETY_ALLOWED_AREA{
/**
Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis
down*/
MAV_FRAME frame;
 float  p1x;//x position 1 / Latitude 1
 float  p1y;//y position 1 / Longitude 1
 float  p1z;//z position 1 / Altitude 1
 float  p2x;//x position 2 / Latitude 2
 float  p2y;//y position 2 / Longitude 2
 float  p2z;//z position 2 / Altitude 2
}

/**
The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
@id(61) public static class ATTITUDE_QUATERNION_COV{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 @__( 4 )  float  q;//Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 float  rollspeed;//Roll angular speed
 float  pitchspeed;//Pitch angular speed
 float  yawspeed;//Yaw angular speed
/**
Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries
are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first
element in the array*/
 @__( 9 )  float  covariance;
}

/**
The state of the fixed wing navigation and position controller.*/
@id(62) public static class NAV_CONTROLLER_OUTPUT{
 float  nav_roll;//Current desired roll
 float  nav_pitch;//Current desired pitch
 short  nav_bearing;//Current desired heading
 short  target_bearing;//Bearing to current waypoint/target
@I short  wp_dist;//Distance to active waypoint
 float  alt_error;//Current altitude error
 float  aspd_error;//Current airspeed error
 float  xtrack_error;//Current crosstrack error on x-y plane
}

/**
The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset*/
@id(63) public static class GLOBAL_POSITION_INT_COV{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
MAV_ESTIMATOR_TYPE estimator_type;//Class id of the estimator this estimate originated from.
  int  lat;//Latitude
  int  lon;//Longitude
  int  alt;//Altitude in meters above MSL
  int  relative_alt;//Altitude above ground
 float  vx;//Ground X Speed (Latitude)
 float  vy;//Ground Y Speed (Longitude)
 float  vz;//Ground Z Speed (Altitude)
/**
Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon,
alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If
unknown, assign NaN value to first element in the array*/
 @__( 36 )  float  covariance;
}

/**
The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
Z-axis down (aeronautical frame, NED / north-east-down convention*/
@id(64) public static class LOCAL_POSITION_NED_COV{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
MAV_ESTIMATOR_TYPE estimator_type;//Class id of the estimator this estimate originated from.
 float  x;//X Position
 float  y;//Y Position
 float  z;//Z Position
 float  vx;//X Speed
 float  vy;//Y Speed
 float  vz;//Z Speed
 float  ax;//X Acceleration
 float  ay;//Y Acceleration
 float  az;//Z Acceleration
/**
Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right
triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries
are the second row, etc.). If unknown, assign NaN value to first element in the array*/
 @__( 45 )  float  covariance;
}

/**
The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters
might violate this specification*/
@id(65) public static class RC_CHANNELS{
@I  int  time_boot_ms;//Timestamp (time since system boot).
/**
Total number of RC channels being received. This can be larger than 18, indicating that more channels
are available but not given in this message. This value should be 0 when no RC channels are available*/
@I byte  chancount;
@I short  chan1_raw;//RC channel 1 value.
@I short  chan2_raw;//RC channel 2 value.
@I short  chan3_raw;//RC channel 3 value.
@I short  chan4_raw;//RC channel 4 value.
@I short  chan5_raw;//RC channel 5 value.
@I short  chan6_raw;//RC channel 6 value.
@I short  chan7_raw;//RC channel 7 value.
@I short  chan8_raw;//RC channel 8 value.
@I short  chan9_raw;//RC channel 9 value.
@I short  chan10_raw;//RC channel 10 value.
@I short  chan11_raw;//RC channel 11 value.
@I short  chan12_raw;//RC channel 12 value.
@I short  chan13_raw;//RC channel 13 value.
@I short  chan14_raw;//RC channel 14 value.
@I short  chan15_raw;//RC channel 15 value.
@I short  chan16_raw;//RC channel 16 value.
@I short  chan17_raw;//RC channel 17 value.
@I short  chan18_raw;//RC channel 18 value.
@I byte  rssi;//Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown
}

/**
Request a data stream.*/
@id(66) public static class REQUEST_DATA_STREAM{
@I byte  target_system;//The target requested to send the message stream.
@I byte  target_component;//The target requested to send the message stream.
@I byte  req_stream_id;//The ID of the requested data stream
@I short  req_message_rate;//The requested message rate
@I byte  start_stop;//1 to start sending, 0 to stop sending.
}

/**
Data stream status information.*/
@id(67) public static class DATA_STREAM{
@I byte  stream_id;//The ID of the requested data stream
@I short  message_rate;//The message rate
@I byte  on_off;//1 stream is enabled, 0 stream is stopped.
}

/**
This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
boolean values of their*/
@id(69) public static class MANUAL_CONTROL{
@I byte  target;//The system to be controlled.
/**
X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
 short  x;
/**
Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
 short  y;
/**
Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
thrust*/
 short  z;
/**
R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
being -1000, and the yaw of a vehicle*/
 short  r;
/**
A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
bit corresponds to Button 1*/
@I short  buttons;
}

/**
The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
100%. Individual receivers/transmitters might violate this specification*/
@id(70) public static class RC_CHANNELS_OVERRIDE{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  chan1_raw;//RC channel 1 value. A value of UINT16_MAX means to ignore this field.
@I short  chan2_raw;//RC channel 2 value. A value of UINT16_MAX means to ignore this field.
@I short  chan3_raw;//RC channel 3 value. A value of UINT16_MAX means to ignore this field.
@I short  chan4_raw;//RC channel 4 value. A value of UINT16_MAX means to ignore this field.
@I short  chan5_raw;//RC channel 5 value. A value of UINT16_MAX means to ignore this field.
@I short  chan6_raw;//RC channel 6 value. A value of UINT16_MAX means to ignore this field.
@I short  chan7_raw;//RC channel 7 value. A value of UINT16_MAX means to ignore this field.
@I short  chan8_raw;//RC channel 8 value. A value of UINT16_MAX means to ignore this field.
@I_ short  chan9_raw;//RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan10_raw;//RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan11_raw;//RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan12_raw;//RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan13_raw;//RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan14_raw;//RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan15_raw;//RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan16_raw;//RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan17_raw;//RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field.
@I_ short  chan18_raw;//RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field.
}

/**
ght handed (ENU). See also https://mavlink.io/en/services/mission.html.*/
@id(73) public static class MISSION_ITEM_INT{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
sequence (0,1,2,3,4)*/
@I short  seq;
MAV_FRAME frame;//The coordinate system of the waypoint.
MAV_CMD command;//The scheduled action for the waypoint.
@I byte  current;//false:0, true:1
@I byte  autocontinue;//Autocontinue to next waypoint
 float  param1;//PARAM1, see MAV_CMD enum
 float  param2;//PARAM2, see MAV_CMD enum
 float  param3;//PARAM3, see MAV_CMD enum
 float  param4;//PARAM4, see MAV_CMD enum
  int  x;//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
  int  y;//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
 float  z;//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
MAV_MISSION_TYPE mission_type;//Mission type.
}

/**
Metrics typically displayed on a HUD for fixed wing aircraft.*/
@id(74) public static class VFR_HUD{
 float  airspeed;//Current indicated airspeed (IAS).
 float  groundspeed;//Current ground speed.
 short  heading;//Current heading in compass units (0-360, 0=north).
@I short  throttle;//Current throttle setting (0 to 100).
 float  alt;//Current altitude (MSL).
 float  climb;//Current climb rate.
}

/**
Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
The command microservice is documented at https://mavlink.io/en/services/command.htm*/
@id(75) public static class COMMAND_INT{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
MAV_FRAME frame;//The coordinate system of the COMMAND.
MAV_CMD command;//The scheduled action for the mission item.
@I byte  current;//false:0, true:1
@I byte  autocontinue;//autocontinue to next wp
 float  param1;//PARAM1, see MAV_CMD enum
 float  param2;//PARAM2, see MAV_CMD enum
 float  param3;//PARAM3, see MAV_CMD enum
 float  param4;//PARAM4, see MAV_CMD enum
  int  x;//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
  int  y;//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 float  z;//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
}

/**
Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.htm*/
@id(76) public static class COMMAND_LONG{
@I byte  target_system;//System which should execute the command
@I byte  target_component;//Component which should execute the command, 0 for all components
MAV_CMD command;//Command ID (of command to send).
@I byte  confirmation;//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 float  param1;//Parameter 1 (for the specific command).
 float  param2;//Parameter 2 (for the specific command).
 float  param3;//Parameter 3 (for the specific command).
 float  param4;//Parameter 4 (for the specific command).
 float  param5;//Parameter 5 (for the specific command).
 float  param6;//Parameter 6 (for the specific command).
 float  param7;//Parameter 7 (for the specific command).
}

/**
Report status of a command. Includes feedback whether the command was executed. The command microservice
is documented at https://mavlink.io/en/services/command.htm*/
@id(77) public static class COMMAND_ACK{
MAV_CMD command;//Command ID (of acknowledged command).
MAV_RESULT result;//Result of command.
/**
WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
@I_ byte  progress;
/**
WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
be denied*/
@I_()   int  result_param2;
@I_ byte  target_system;//WIP: System which requested the command to be executed
@I_ byte  target_component;//WIP: Component which requested the command to be executed
}

/**
Setpoint in roll, pitch, yaw and thrust from the operator*/
@id(81) public static class MANUAL_SETPOINT{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  roll;//Desired roll rate
 float  pitch;//Desired pitch rate
 float  yaw;//Desired yaw rate
 float  thrust;//Collective thrust, normalized to 0 .. 1
@I byte  mode_switch;//Flight mode switch position, 0.. 255
@I byte  manual_override_switch;//Override mode switch position, 0.. 255
}

/**
Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
or other system)*/
@id(82) public static class SET_ATTITUDE_TARGET{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
@I byte  type_mask;
 @__( 4 )  float  q;//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 float  body_roll_rate;//Body roll rate
 float  body_pitch_rate;//Body pitch rate
 float  body_yaw_rate;//Body yaw rate
 float  thrust;//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
}

/**
Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
@id(83) public static class ATTITUDE_TARGET{
@I  int  time_boot_ms;//Timestamp (time since system boot).
/**
Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
@I byte  type_mask;
 @__( 4 )  float  q;//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 float  body_roll_rate;//Body roll rate
 float  body_pitch_rate;//Body pitch rate
 float  body_yaw_rate;//Body yaw rate
 float  thrust;//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
}

/**
Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
to command the vehicle (manual controller or other system)*/
@id(84) public static class SET_POSITION_TARGET_LOCAL_NED{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
= */
MAV_FRAME coordinate_frame;
POSITION_TARGET_TYPEMASK type_mask;//Bitmap to indicate which dimensions should be ignored by the vehicle.
 float  x;//X Position in NED frame
 float  y;//Y Position in NED frame
 float  z;//Z Position in NED frame (note, altitude is negative in NED)
 float  vx;//X velocity in NED frame
 float  vy;//Y velocity in NED frame
 float  vz;//Z velocity in NED frame
 float  afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  yaw;//yaw setpoint
 float  yaw_rate;//yaw rate setpoint
}

/**
Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
this way*/
@id(85) public static class POSITION_TARGET_LOCAL_NED{
@I  int  time_boot_ms;//Timestamp (time since system boot).
/**
Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
= */
MAV_FRAME coordinate_frame;
POSITION_TARGET_TYPEMASK type_mask;//Bitmap to indicate which dimensions should be ignored by the vehicle.
 float  x;//X Position in NED frame
 float  y;//Y Position in NED frame
 float  z;//Z Position in NED frame (note, altitude is negative in NED)
 float  vx;//X velocity in NED frame
 float  vy;//Y velocity in NED frame
 float  vz;//Z velocity in NED frame
 float  afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  yaw;//yaw setpoint
 float  yaw_rate;//yaw rate setpoint
}

/**
Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
Used by an external controller to command the vehicle (manual controller or other system)*/
@id(86) public static class SET_POSITION_TARGET_GLOBAL_INT{
/**
Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system
to compensate for the transport delay of the setpoint. This allows the system to compensate processing
latency*/
@I  int  time_boot_ms;
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
= 1*/
MAV_FRAME coordinate_frame;
POSITION_TARGET_TYPEMASK type_mask;//Bitmap to indicate which dimensions should be ignored by the vehicle.
  int  lat_int;//X Position in WGS84 frame
  int  lon_int;//Y Position in WGS84 frame
 float  alt;//Altitude (MSL, Relative to home, or AGL - depending on frame)
 float  vx;//X velocity in NED frame
 float  vy;//Y velocity in NED frame
 float  vz;//Z velocity in NED frame
 float  afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  yaw;//yaw setpoint
 float  yaw_rate;//yaw rate setpoint
}

/**
Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
this way*/
@id(87) public static class POSITION_TARGET_GLOBAL_INT{
/**
Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system
to compensate for the transport delay of the setpoint. This allows the system to compensate processing
latency*/
@I  int  time_boot_ms;
/**
Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
= 1*/
MAV_FRAME coordinate_frame;
POSITION_TARGET_TYPEMASK type_mask;//Bitmap to indicate which dimensions should be ignored by the vehicle.
  int  lat_int;//X Position in WGS84 frame
  int  lon_int;//Y Position in WGS84 frame
 float  alt;//Altitude (MSL, AGL or relative to home altitude, depending on frame)
 float  vx;//X velocity in NED frame
 float  vy;//Y velocity in NED frame
 float  vz;//Z velocity in NED frame
 float  afx;//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afy;//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  afz;//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float  yaw;//yaw setpoint
 float  yaw_rate;//yaw rate setpoint
}

/**
The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
convention*/
@id(89) public static class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  x;//X Position
 float  y;//Y Position
 float  z;//Z Position
 float  roll;//Roll
 float  pitch;//Pitch
 float  yaw;//Yaw
}

/**
Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware
in the loop simulations*/
 }
public interface CommonPacks {  @id(90) public static class HIL_STATE{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  roll;//Roll angle
 float  pitch;//Pitch angle
 float  yaw;//Yaw angle
 float  rollspeed;//Body frame roll / phi angular speed
 float  pitchspeed;//Body frame pitch / theta angular speed
 float  yawspeed;//Body frame yaw / psi angular speed
  int  lat;//Latitude
  int  lon;//Longitude
  int  alt;//Altitude
 short  vx;//Ground X Speed (Latitude)
 short  vy;//Ground Y Speed (Longitude)
 short  vz;//Ground Z Speed (Altitude)
 short  xacc;//X acceleration
 short  yacc;//Y acceleration
 short  zacc;//Z acceleration
}

/**
Sent from autopilot to simulation. Hardware in the loop control outputs*/
@id(91) public static class HIL_CONTROLS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  roll_ailerons;//Control output -1 .. 1
 float  pitch_elevator;//Control output -1 .. 1
 float  yaw_rudder;//Control output -1 .. 1
 float  throttle;//Throttle 0 .. 1
 float  aux1;//Aux 1, -1 .. 1
 float  aux2;//Aux 2, -1 .. 1
 float  aux3;//Aux 3, -1 .. 1
 float  aux4;//Aux 4, -1 .. 1
MAV_MODE mode;//System mode.
@I byte  nav_mode;//Navigation mode (MAV_NAV_MODE)
}

/**
Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
violate this specification*/
@id(92) public static class HIL_RC_INPUTS_RAW{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I short  chan1_raw;//RC channel 1 value
@I short  chan2_raw;//RC channel 2 value
@I short  chan3_raw;//RC channel 3 value
@I short  chan4_raw;//RC channel 4 value
@I short  chan5_raw;//RC channel 5 value
@I short  chan6_raw;//RC channel 6 value
@I short  chan7_raw;//RC channel 7 value
@I short  chan8_raw;//RC channel 8 value
@I short  chan9_raw;//RC channel 9 value
@I short  chan10_raw;//RC channel 10 value
@I short  chan11_raw;//RC channel 11 value
@I short  chan12_raw;//RC channel 12 value
@I byte  rssi;//Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown
}

/**
Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
@id(93) public static class HIL_ACTUATOR_CONTROLS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 @__( 16 )  float  controls;//Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
MAV_MODE_FLAG mode;//System mode. Includes arming state.
@I long  flags;//Flags as bitfield, reserved for future use.
}

/**
Optical flow from a flow sensor (e.g. optical mouse sensor)*/
@id(100) public static class OPTICAL_FLOW{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  sensor_id;//Sensor ID
 short  flow_x;//Flow in x-sensor direction
 short  flow_y;//Flow in y-sensor direction
 float  flow_comp_m_x;//Flow in x-sensor direction, angular-speed compensated
 float  flow_comp_m_y;//Flow in y-sensor direction, angular-speed compensated
@I byte  quality;//Optical flow quality / confidence. 0: bad, 255: maximum quality
 float  ground_distance;//Ground distance. Positive value: distance known. Negative value: Unknown distance
@I_()  float  flow_rate_x;//Flow rate about X axis
@I_()  float  flow_rate_y;//Flow rate about Y axis
}

/**
Global position/attitude estimate from a vision source.*/
@id(101) public static class GLOBAL_VISION_POSITION_ESTIMATE{
@I long  usec;//Timestamp (UNIX time or since system boot)
 float  x;//Global X position
 float  y;//Global Y position
 float  z;//Global Z position
 float  roll;//Roll angle
 float  pitch;//Pitch angle
 float  yaw;//Yaw angle
/**
Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x_global, y_global,
z_global, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW,
etc.). If unknown, assign NaN value to first element in the array*/
@I_()  @__( 21 )  float  covariance;
/**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps*/
@I_ byte  reset_counter;
}

/**
Local position/attitude estimate from a vision source.*/
@id(102) public static class VISION_POSITION_ESTIMATE{
@I long  usec;//Timestamp (UNIX time or time since system boot)
 float  x;//Local X position
 float  y;//Local Y position
 float  z;//Local Z position
 float  roll;//Roll angle
 float  pitch;//Pitch angle
 float  yaw;//Yaw angle
/**
Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll,
pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown,
assign NaN value to first element in the array*/
@I_()  @__( 21 )  float  covariance;
/**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps*/
@I_ byte  reset_counter;
}

/**
Speed estimate from a vision source.*/
@id(103) public static class VISION_SPEED_ESTIMATE{
@I long  usec;//Timestamp (UNIX time or time since system boot)
 float  x;//Global X speed
 float  y;//Global Y speed
 float  z;//Global Z speed
/**
Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries
- 1st row, etc.). If unknown, assign NaN value to first element in the array*/
@I_()  @__( 9 )  float  covariance;
/**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps*/
@I_ byte  reset_counter;
}

/**
Global position estimate from a Vicon motion system source.*/
@id(104) public static class VICON_POSITION_ESTIMATE{
@I long  usec;//Timestamp (UNIX time or time since system boot)
 float  x;//Global X position
 float  y;//Global Y position
 float  z;//Global Z position
 float  roll;//Roll angle
 float  pitch;//Pitch angle
 float  yaw;//Yaw angle
/**
Row-major representation of 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll,
pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown,
assign NaN value to first element in the array*/
@I_()  @__( 21 )  float  covariance;
}

/**
The IMU readings in SI units in NED body frame*/
@id(105) public static class HIGHRES_IMU{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  xacc;//X acceleration
 float  yacc;//Y acceleration
 float  zacc;//Z acceleration
 float  xgyro;//Angular speed around X axis
 float  ygyro;//Angular speed around Y axis
 float  zgyro;//Angular speed around Z axis
 float  xmag;//X Magnetic field
 float  ymag;//Y Magnetic field
 float  zmag;//Z Magnetic field
 float  abs_pressure;//Absolute pressure
 float  diff_pressure;//Differential pressure
 float  pressure_alt;//Altitude calculated from pressure
 float  temperature;//Temperature
@I short  fields_updated;//Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
@I_ byte  id;//Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0
}

/**
Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
@id(106) public static class OPTICAL_FLOW_RAD{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  sensor_id;//Sensor ID
/**
Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow.
The integration time also indicates the*/
@I  int  integration_time_us;
/**
Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion
along the positive Y axis induces a negative flow.*/
 float  integrated_x;
/**
Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion
along the positive X axis induces a positive flow.*/
 float  integrated_y;
 float  integrated_xgyro;//RH rotation around X axis
 float  integrated_ygyro;//RH rotation around Y axis
 float  integrated_zgyro;//RH rotation around Z axis
 short  temperature;//Temperature
@I byte  quality;//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
@I  int  time_delta_distance_us;//Time since the distance was sampled.
/**
Distance to the center of the flow field. Positive value (including zero): distance known. Negative value:
Unknown distance*/
 float  distance;
}

/**
The IMU readings in SI units in NED body frame*/
@id(107) public static class HIL_SENSOR{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  xacc;//X acceleration
 float  yacc;//Y acceleration
 float  zacc;//Z acceleration
 float  xgyro;//Angular speed around X axis in body frame
 float  ygyro;//Angular speed around Y axis in body frame
 float  zgyro;//Angular speed around Z axis in body frame
 float  xmag;//X Magnetic field
 float  ymag;//Y Magnetic field
 float  zmag;//Z Magnetic field
 float  abs_pressure;//Absolute pressure
 float  diff_pressure;//Differential pressure (airspeed)
 float  pressure_alt;//Altitude calculated from pressure
 float  temperature;//Temperature
/**
Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
reset of attitude/position/velocities/etc was performed in sim*/
@I  int  fields_updated;
}

/**
Status of simulation environment, if used*/
@id(108) public static class SIM_STATE{
 float  q1;//True attitude quaternion component 1, w (1 in null-rotation)
 float  q2;//True attitude quaternion component 2, x (0 in null-rotation)
 float  q3;//True attitude quaternion component 3, y (0 in null-rotation)
 float  q4;//True attitude quaternion component 4, z (0 in null-rotation)
 float  roll;//Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 float  pitch;//Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 float  yaw;//Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 float  xacc;//X acceleration
 float  yacc;//Y acceleration
 float  zacc;//Z acceleration
 float  xgyro;//Angular speed around X axis
 float  ygyro;//Angular speed around Y axis
 float  zgyro;//Angular speed around Z axis
 float  lat;//Latitude
 float  lon;//Longitude
 float  alt;//Altitude
 float  std_dev_horz;//Horizontal position standard deviation
 float  std_dev_vert;//Vertical position standard deviation
 float  vn;//True velocity in north direction in earth-fixed NED frame
 float  ve;//True velocity in east direction in earth-fixed NED frame
 float  vd;//True velocity in down direction in earth-fixed NED frame
}

/**
Status generated by radio and injected into MAVLink stream.*/
@id(109) public static class RADIO_STATUS{
/**
Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254],
255: invalid/unknown*/
@I byte  rssi;
/**
Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254],
255: invalid/unknown*/
@I byte  remrssi;
@I byte  txbuf;//Remaining free transmitter buffer space.
/**
Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios).
Values: [0-254], 255: invalid/unknown*/
@I byte  noise;
/**
Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios).
Values: [0-254], 255: invalid/unknown*/
@I byte  remnoise;
@I short  rxerrors;//Count of radio packet receive errors (since boot).
@I short  fixed;//Count of error corrected radio packets (since boot).
}

/**
File transfer message*/
@id(110) public static class FILE_TRANSFER_PROTOCOL{
@I byte  target_network;//Network ID (0 for broadcast)
@I byte  target_system;//System ID (0 for broadcast)
@I byte  target_component;//Component ID (0 for broadcast)
/**
Variable length payload. The length is defined by the remaining message length when subtracting the header
and other fields.  The entire content of this block is opaque unless you understand any the encoding
message_type.  The particular encoding used can be extension specific and might not always be documented
as part of the mavlink specification*/
@I @__( 251 )  byte  payload;
}

/**
Time synchronization message.*/
@id(111) public static class TIMESYNC{
 long  tc1;//Time sync timestamp 1
 long  ts1;//Time sync timestamp 2
}

/**
Camera-IMU triggering and synchronisation message.*/
@id(112) public static class CAMERA_TRIGGER{
/**
Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp
format (since 1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I  int  seq;//Image frame sequence
}

/**
The global position, as returned by the Global Positioning System (GPS). This is
 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.*/
@id(113) public static class HIL_GPS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
/**
0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
at least two, so always correctly fill in the fix*/
@I byte  fix_type;
  int  lat;//Latitude (WGS84)
  int  lon;//Longitude (WGS84)
  int  alt;//Altitude (MSL). Positive for up.
@I short  eph;//GPS HDOP horizontal dilution of position. If unknown, set to: 65535
@I short  epv;//GPS VDOP vertical dilution of position. If unknown, set to: 65535
@I short  vel;//GPS ground speed. If unknown, set to: 65535
 short  vn;//GPS velocity in north direction in earth-fixed NED frame
 short  ve;//GPS velocity in east direction in earth-fixed NED frame
 short  vd;//GPS velocity in down direction in earth-fixed NED frame
/**
Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to:
6553*/
@I short  cog;
@I byte  satellites_visible;//Number of satellites visible. If unknown, set to 255
}

/**
Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
@id(114) public static class HIL_OPTICAL_FLOW{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  sensor_id;//Sensor ID
/**
Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow.
The integration time also indicates the*/
@I  int  integration_time_us;
/**
Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
motion along the positive Y axis induces a negative flow.*/
 float  integrated_x;
/**
Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
motion along the positive X axis induces a positive flow.*/
 float  integrated_y;
 float  integrated_xgyro;//RH rotation around X axis
 float  integrated_ygyro;//RH rotation around Y axis
 float  integrated_zgyro;//RH rotation around Z axis
 short  temperature;//Temperature
@I byte  quality;//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
@I  int  time_delta_distance_us;//Time since the distance was sampled.
/**
Distance to the center of the flow field. Positive value (including zero): distance known. Negative value:
Unknown distance*/
 float  distance;
}

/**
Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
for high throughput applications such as hardware in the loop simulations*/
@id(115) public static class HIL_STATE_QUATERNION{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 @__( 4 )  float  attitude_quaternion;//Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
 float  rollspeed;//Body frame roll / phi angular speed
 float  pitchspeed;//Body frame pitch / theta angular speed
 float  yawspeed;//Body frame yaw / psi angular speed
  int  lat;//Latitude
  int  lon;//Longitude
  int  alt;//Altitude
 short  vx;//Ground X Speed (Latitude)
 short  vy;//Ground Y Speed (Longitude)
 short  vz;//Ground Z Speed (Altitude)
@I short  ind_airspeed;//Indicated airspeed
@I short  true_airspeed;//True airspeed
 short  xacc;//X acceleration
 short  yacc;//Y acceleration
 short  zacc;//Z acceleration
}

/**
The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
the described unit*/
@id(116) public static class SCALED_IMU2{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 short  xacc;//X acceleration
 short  yacc;//Y acceleration
 short  zacc;//Z acceleration
 short  xgyro;//Angular speed around X axis
 short  ygyro;//Angular speed around Y axis
 short  zgyro;//Angular speed around Z axis
 short  xmag;//X Magnetic field
 short  ymag;//Y Magnetic field
 short  zmag;//Z Magnetic field
@I_()  short  temperature;//Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C)
}

/**
Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
is called*/
@id(117) public static class LOG_REQUEST_LIST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  start;//First log id (0 for first available)
@I short  end;//Last log id (0xffff for last available)
}

/**
Reply to LOG_REQUEST_LIST*/
@id(118) public static class LOG_ENTRY{
@I short  id;//Log id
@I short  num_logs;//Total number of logs
@I short  last_log_num;//High log number
@I  int  time_utc;//UTC timestamp of log since 1970, or 0 if not available
@I  int  size;//Size of the log (may be approximate)
}

/**
Request a chunk of a log*/
@id(119) public static class LOG_REQUEST_DATA{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I short  id;//Log id (from LOG_ENTRY reply)
@I  int  ofs;//Offset into the log
@I  int  count;//Number of bytes
}

/**
Reply to LOG_REQUEST_DATA*/
@id(120) public static class LOG_DATA{
@I short  id;//Log id (from LOG_ENTRY reply)
@I  int  ofs;//Offset into the log
@I byte  count;//Number of bytes (zero for end of log)
@I @__( 90 )  byte  data;//log data
}

/**
Erase all logs*/
@id(121) public static class LOG_ERASE{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
}

/**
Stop log transfer and resume normal logging*/
@id(122) public static class LOG_REQUEST_END{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
}

/**
Data for injecting into the onboard GPS (used for DGPS)*/
@id(123) public static class GPS_INJECT_DATA{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@I byte  len;//Data length
@I @__( 110 )  byte  data;//Raw data (110 is enough for 12 satellites of RTCMv2)
}

/**
Second GPS data.*/
@id(124) public static class GPS2_RAW{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
GPS_FIX_TYPE fix_type;//GPS fix type.
  int  lat;//Latitude (WGS84)
  int  lon;//Longitude (WGS84)
  int  alt;//Altitude (MSL). Positive for up.
@I short  eph;//GPS HDOP horizontal dilution of position. If unknown, set to: UINT16_MAX
@I short  epv;//GPS VDOP vertical dilution of position. If unknown, set to: UINT16_MAX
@I short  vel;//GPS ground speed. If unknown, set to: UINT16_MAX
/**
Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to:
UINT16_MA*/
@I short  cog;
@I byte  satellites_visible;//Number of satellites visible. If unknown, set to 255
@I byte  dgps_numch;//Number of DGPS satellites
@I  int  dgps_age;//Age of DGPS info
}

/**
Power supply status*/
@id(125) public static class POWER_STATUS{
@I short  Vcc;//5V rail voltage.
@I short  Vservo;//Servo rail voltage.
MAV_POWER_STATUS flags;//Bitmap of power supply status flags.
}

/**
Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
@id(126) public static class SERIAL_CONTROL{
SERIAL_CONTROL_DEV device;//Serial control device type.
SERIAL_CONTROL_FLAG flags;//Bitmap of serial control flags.
@I short  timeout;//Timeout for reply data
@I  int  baudrate;//Baudrate of transfer. Zero means no change.
@I byte  count;//how many bytes in this transfer
@I @__( 70 )  byte  data;//serial data
}

/**
RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
@id(127) public static class GPS_RTK{
@I  int  time_last_baseline_ms;//Time since boot of last baseline message received.
@I byte  rtk_receiver_id;//Identification of connected RTK receiver.
@I short  wn;//GPS Week Number of last baseline
@I  int  tow;//GPS Time of Week of last baseline
@I byte  rtk_health;//GPS-specific health report for RTK data.
@I byte  rtk_rate;//Rate of baseline messages being received by GPS
@I byte  nsats;//Current number of sats used for RTK calculation.
RTK_BASELINE_COORDINATE_SYSTEM baseline_coords_type;//Coordinate system of baseline
  int  baseline_a_mm;//Current baseline in ECEF x or NED north component.
  int  baseline_b_mm;//Current baseline in ECEF y or NED east component.
  int  baseline_c_mm;//Current baseline in ECEF z or NED down component.
@I  int  accuracy;//Current estimate of baseline accuracy.
  int  iar_num_hypotheses;//Current number of integer ambiguity hypotheses.
}

/**
RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
@id(128) public static class GPS2_RTK{
@I  int  time_last_baseline_ms;//Time since boot of last baseline message received.
@I byte  rtk_receiver_id;//Identification of connected RTK receiver.
@I short  wn;//GPS Week Number of last baseline
@I  int  tow;//GPS Time of Week of last baseline
@I byte  rtk_health;//GPS-specific health report for RTK data.
@I byte  rtk_rate;//Rate of baseline messages being received by GPS
@I byte  nsats;//Current number of sats used for RTK calculation.
RTK_BASELINE_COORDINATE_SYSTEM baseline_coords_type;//Coordinate system of baseline
  int  baseline_a_mm;//Current baseline in ECEF x or NED north component.
  int  baseline_b_mm;//Current baseline in ECEF y or NED east component.
  int  baseline_c_mm;//Current baseline in ECEF z or NED down component.
@I  int  accuracy;//Current estimate of baseline accuracy.
  int  iar_num_hypotheses;//Current number of integer ambiguity hypotheses.
}

/**
The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
unit*/
@id(129) public static class SCALED_IMU3{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 short  xacc;//X acceleration
 short  yacc;//Y acceleration
 short  zacc;//Z acceleration
 short  xgyro;//Angular speed around X axis
 short  ygyro;//Angular speed around Y axis
 short  zgyro;//Angular speed around Z axis
 short  xmag;//X Magnetic field
 short  ymag;//Y Magnetic field
 short  zmag;//Z Magnetic field
@I_()  short  temperature;//Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C)
}

/**
Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol:
https://mavlink.io/en/services/image_transmission.html*/
@id(130) public static class DATA_TRANSMISSION_HANDSHAKE{
MAVLINK_DATA_STREAM_TYPE type;//Type of requested/acknowledged data.
@I  int  size;//total data size (set on ACK only).
@I short  width;//Width of a matrix or image.
@I short  height;//Height of a matrix or image.
@I short  packets;//Number of packets being sent (set on ACK only).
/**
Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
ACK only)*/
@I byte  payload;
@I byte  jpg_quality;//JPEG quality. Values: [1-100].
}

/**
Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html*/
@id(131) public static class ENCAPSULATED_DATA{
@I short  seqnr;//sequence number (starting with 0 on every transmission)
@I @__( 253 )  byte  data;//image data bytes
}

/**
Distance sensor information for an onboard rangefinder.*/
@id(132) public static class DISTANCE_SENSOR{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I short  min_distance;//Minimum distance the sensor can measure
@I short  max_distance;//Maximum distance the sensor can measure
@I short  current_distance;//Current distance reading
MAV_DISTANCE_SENSOR type;//Type of distance sensor.
@I byte  id;//Onboard ID of the sensor
/**
Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing:
ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_27*/
MAV_SENSOR_ORIENTATION orientation;
@I byte  covariance;//Measurement variance. Max standard deviation is 6cm. 255 if unknown.
/**
Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known.
Otherwise this is set to 0*/
@I_()  float  horizontal_fov;
/**
Vertical Field of View (angle) where the distance measurement is valid and the field of view is known.
Otherwise this is set to 0*/
@I_()  float  vertical_fov;
/**
Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0,
0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set
to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid.*/
@I_()  @__( 4 )  float  quaternion;
}

/**
Request for terrain data and terrain status*/
@id(133) public static class TERRAIN_REQUEST{
  int  lat;//Latitude of SW corner of first grid
  int  lon;//Longitude of SW corner of first grid
@I short  grid_spacing;//Grid spacing
@I long  mask;//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
}

/**
Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
@id(134) public static class TERRAIN_DATA{
  int  lat;//Latitude of SW corner of first grid
  int  lon;//Longitude of SW corner of first grid
@I short  grid_spacing;//Grid spacing
@I byte  gridbit;//bit within the terrain request mask
 @__( 16 )  short  data;//Terrain data MSL
}

/**
Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
has all terrain data needed for a mission*/
@id(135) public static class TERRAIN_CHECK{
  int  lat;//Latitude
  int  lon;//Longitude
}

/**
Response from a TERRAIN_CHECK request*/
@id(136) public static class TERRAIN_REPORT{
  int  lat;//Latitude
  int  lon;//Longitude
@I short  spacing;//grid spacing (zero if terrain at this location unavailable)
 float  terrain_height;//Terrain height MSL
 float  current_height;//Current vehicle height above lat/lon terrain height
@I short  pending;//Number of 4x4 terrain blocks waiting to be received or read from disk
@I short  loaded;//Number of 4x4 terrain blocks in memory
}

/**
Barometer readings for 2nd barometer*/
@id(137) public static class SCALED_PRESSURE2{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  press_abs;//Absolute pressure
 float  press_diff;//Differential pressure
 short  temperature;//Temperature measurement
}

/**
Motion capture attitude and position*/
@id(138) public static class ATT_POS_MOCAP{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 @__( 4 )  float  q;//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 float  x;//X position (NED)
 float  y;//Y position (NED)
 float  z;//Z position (NED)
/**
Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z,
roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If
unknown, assign NaN value to first element in the array*/
@I_()  @__( 21 )  float  covariance;
}

/**
Set the vehicle attitude and body angular rates.*/
@id(139) public static class SET_ACTUATOR_CONTROL_TARGET{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
/**
Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
this field to difference between instances*/
@I byte  group_mlx;
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
mixer to repurpose them as generic outputs*/
 @__( 8 )  float  controls;
}

/**
Set the vehicle attitude and body angular rates.*/
@id(140) public static class ACTUATOR_CONTROL_TARGET{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
/**
Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
this field to difference between instances*/
@I byte  group_mlx;
/**
Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
mixer to repurpose them as generic outputs*/
 @__( 8 )  float  controls;
}

/**
The current system altitude.*/
@id(141) public static class ALTITUDE{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
/**
This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
local altitude change). The only guarantee on this field is that it will never be reset and is consistent
within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
time. This altitude will also drift and vary between flights*/
 float  altitude_monotonic;
/**
This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output MSL
by default and not the WGS84 altitude*/
 float  altitude_amsl;
/**
This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
to the coordinate origin (0, 0, 0). It is up-positive*/
 float  altitude_local;
 float  altitude_relative;//This is the altitude above the home position. It resets on each change of the current home position
/**
This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
than -1000 should be interpreted as unknown*/
 float  altitude_terrain;
/**
This is not the altitude, but the clear space below the system according to the fused clearance estimate.
It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
target. A negative value indicates no measurement available*/
 float  bottom_clearance;
}

/**
The autopilot is requesting a resource (file, binary, other type of data)*/
@id(142) public static class RESOURCE_REQUEST{
@I byte  request_id;//Request ID. This ID should be re-used when sending back URI contents
@I byte  uri_type;//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
/**
The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
on the URI type enum*/
@I @__( 120 )  byte  uri;
@I byte  transfer_type;//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
/**
The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
has a storage associated (e.g. MAVLink FTP)*/
@I @__( 120 )  byte  storage;
}

/**
Barometer readings for 3rd barometer*/
@id(143) public static class SCALED_PRESSURE3{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  press_abs;//Absolute pressure
 float  press_diff;//Differential pressure
 short  temperature;//Temperature measurement
}

/**
Current motion information from a designated system*/
@id(144) public static class FOLLOW_TARGET{
@I long  timestamp;//Timestamp (time since system boot).
@I byte  est_capabilities;//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
  int  lat;//Latitude (WGS84)
  int  lon;//Longitude (WGS84)
 float  alt;//Altitude (MSL)
 @__( 3 )  float  vel;//target velocity (0,0,0) for unknown
 @__( 3 )  float  acc;//linear target acceleration (0,0,0) for unknown
 @__( 4 )  float  attitude_q;//(1 0 0 0 for unknown)
 @__( 3 )  float  rates;//(0 0 0 for unknown)
 @__( 3 )  float  position_cov;//eph epv
@I long  custom_state;//button states or switches of a tracker device
}

/**
The smoothed, monotonic system state used to feed the control loops of the system.*/
@id(146) public static class CONTROL_SYSTEM_STATE{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  x_acc;//X acceleration in body frame
 float  y_acc;//Y acceleration in body frame
 float  z_acc;//Z acceleration in body frame
 float  x_vel;//X velocity in body frame
 float  y_vel;//Y velocity in body frame
 float  z_vel;//Z velocity in body frame
 float  x_pos;//X position in local frame
 float  y_pos;//Y position in local frame
 float  z_pos;//Z position in local frame
 float  airspeed;//Airspeed, set to -1 if unknown
 @__( 3 )  float  vel_variance;//Variance of body velocity estimate
 @__( 3 )  float  pos_variance;//Variance in local position
 @__( 4 )  float  q;//The attitude, represented as Quaternion
 float  roll_rate;//Angular rate in roll axis
 float  pitch_rate;//Angular rate in pitch axis
 float  yaw_rate;//Angular rate in yaw axis
}

/**
Battery information. Updates GCS with flight controller battery status. Use SMART_BATTERY_* messages instead
for smart batteries*/
@id(147) public static class BATTERY_STATUS{
@I byte  id;//Battery ID
MAV_BATTERY_FUNCTION battery_function;//Function of the battery
MAV_BATTERY_TYPE type;//Type (chemistry) of the battery
 short  temperature;//Temperature of the battery. INT16_MAX for unknown temperature.
/**
Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX
value*/
@I @__( 10 )  short  voltages;
 short  current_battery;//Battery current, -1: autopilot does not measure the current
  int  current_consumed;//Consumed charge, -1: autopilot does not provide consumption estimate
  int  energy_consumed;//Consumed energy, -1: autopilot does not provide energy consumption estimate
 byte  battery_remaining;//Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
@I_()   int  time_remaining;//Remaining battery time, 0: autopilot does not provide remaining battery time estimate
MAV_BATTERY_CHARGE_STATE charge_state;//State for extent of discharge, provided by autopilot for warning or external reactions
}

/**
Version and capability of autopilot software. This should be emitted in response to a MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
command*/
@id(148) public static class AUTOPILOT_VERSION{
MAV_PROTOCOL_CAPABILITY capabilities;//Bitmap of capabilities
@I  int  flight_sw_version;//Firmware version number
@I  int  middleware_sw_version;//Middleware version number
@I  int  os_sw_version;//Operating system version number
@I  int  board_version;//HW / board version (last 8 bytes should be silicon ID, if any)
/**
Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
should allow to identify the commit using the main version number even for very large code bases*/
@I @__( 8 )  byte  flight_custom_version;
/**
Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
should allow to identify the commit using the main version number even for very large code bases*/
@I @__( 8 )  byte  middleware_custom_version;
/**
Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
should allow to identify the commit using the main version number even for very large code bases*/
@I @__( 8 )  byte  os_custom_version;
@I short  vendor_id;//ID of the board vendor
@I short  product_id;//ID of the product
@I long  uid;//UID if provided by hardware (see uid2)
/**
UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
use uid*/
@I_ @__( 18 )  byte  uid2;
}

/**
The location of a landing target. See: https://mavlink.io/en/services/landing_target.html*/
@id(149) public static class LANDING_TARGET{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  target_num;//The ID of the target if multiple targets are present
MAV_FRAME frame;//Coordinate frame used for following fields.
 float  angle_x;//X-axis angular offset of the target from the center of the image
 float  angle_y;//Y-axis angular offset of the target from the center of the image
 float  distance;//Distance to the target from the vehicle
 float  size_x;//Size of target along x-axis
 float  size_y;//Size of target along y-axis
@I_()  float  x;//X Position of the landing target in MAV_FRAME
@I_()  float  y;//Y Position of the landing target in MAV_FRAME
@I_()  float  z;//Z Position of the landing target in MAV_FRAME
@I_()  @__( 4 )  float  q;//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
LANDING_TARGET_TYPE type;//Type of landing target
/**
Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information
(valid: 1, invalid: 0). Default is 0 (invalid)*/
@I_ byte  position_valid;
}

/**
Status of geo-fencing. Sent in extended status stream when fencing enabled.*/
@id(162) public static class FENCE_STATUS{
@I byte  breach_status;//Breach status (0 if currently inside fence, 1 if outside).
@I short  breach_count;//Number of fence breaches.
FENCE_BREACH breach_type;//Last breach type.
@I  int  breach_time;//Time (since boot) of last breach.
FENCE_MITIGATE breach_mitigation;//Active action to prevent fence breach
}

/**
Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
enum definition for further information. The innovation test ratios show the magnitude of the sensor
innovation divided by the innovation check threshold. Under normal operation the innovation test ratios
should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal
operation and indicate that a measurement has been rejected by the filter. The user should be notified
if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between
0.5 and 1.0 should be optional and controllable by the user*/
@id(230) public static class ESTIMATOR_STATUS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
ESTIMATOR_STATUS_FLAGS flags;//Bitmap indicating which EKF outputs are valid.
 float  vel_ratio;//Velocity innovation test ratio
 float  pos_horiz_ratio;//Horizontal position innovation test ratio
 float  pos_vert_ratio;//Vertical position innovation test ratio
 float  mag_ratio;//Magnetometer innovation test ratio
 float  hagl_ratio;//Height above terrain innovation test ratio
 float  tas_ratio;//True airspeed innovation test ratio
 float  pos_horiz_accuracy;//Horizontal position 1-STD accuracy relative to the EKF local origin
 float  pos_vert_accuracy;//Vertical position 1-STD accuracy relative to the EKF local origin
}

/**
Wind covariance estimate from vehicle.*/
@id(231) public static class WIND_COV{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  wind_x;//Wind in X (NED) direction
 float  wind_y;//Wind in Y (NED) direction
 float  wind_z;//Wind in Z (NED) direction
 float  var_horiz;//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
 float  var_vert;//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
 float  wind_alt;//Altitude (MSL) that this measurement was taken at
 float  horiz_accuracy;//Horizontal speed 1-STD accuracy
 float  vert_accuracy;//Vertical speed 1-STD accuracy
}

/**
GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
estimate of the system*/
@id(232) public static class GPS_INPUT{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  gps_id;//ID of the GPS for multiple GPS inputs
GPS_INPUT_IGNORE_FLAGS ignore_flags;//Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
@I  int  time_week_ms;//GPS time (from start of GPS week)
@I short  time_week;//GPS week number
@I byte  fix_type;//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
  int  lat;//Latitude (WGS84)
  int  lon;//Longitude (WGS84)
 float  alt;//Altitude (MSL). Positive for up.
 float  hdop;//GPS HDOP horizontal dilution of position
 float  vdop;//GPS VDOP vertical dilution of position
 float  vn;//GPS velocity in north direction in earth-fixed NED frame
 float  ve;//GPS velocity in east direction in earth-fixed NED frame
 float  vd;//GPS velocity in down direction in earth-fixed NED frame
 float  speed_accuracy;//GPS speed accuracy
 float  horiz_accuracy;//GPS horizontal accuracy
 float  vert_accuracy;//GPS vertical accuracy
@I byte  satellites_visible;//Number of satellites visible.
@I_ short  yaw;//Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
}

/**
RTCM message for injecting into the onboard GPS (used for DGPS)*/
@id(233) public static class GPS_RTCM_DATA{
/**
LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
corrupt RTCM data, and to recover from a unreliable transport delivery order*/
@I byte  flags;
@I byte  len;//data length
@I @__( 180 )  byte  data;//RTCM message (may be fragmented)
}

/**
Message appropriate for high latency connections like Iridium*/
@id(234) public static class HIGH_LATENCY{
MAV_MODE_FLAG base_mode;//Bitmap of enabled system modes.
@I  int  custom_mode;//A bitfield for use for autopilot-specific flags.
MAV_LANDED_STATE landed_state;//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 short  roll;//roll
 short  pitch;//pitch
@I short  heading;//heading
 byte  throttle;//throttle (percentage)
 short  heading_sp;//heading setpoint
  int  latitude;//Latitude
  int  longitude;//Longitude
 short  altitude_amsl;//Altitude above mean sea level
 short  altitude_sp;//Altitude setpoint relative to the home position
@I byte  airspeed;//airspeed
@I byte  airspeed_sp;//airspeed setpoint
@I byte  groundspeed;//groundspeed
 byte  climb_rate;//climb rate
@I byte  gps_nsat;//Number of satellites visible. If unknown, set to 255
GPS_FIX_TYPE gps_fix_type;//GPS Fix type.
@I byte  battery_remaining;//Remaining battery (percentage)
 byte  temperature;//Autopilot temperature (degrees C)
 byte  temperature_air;//Air temperature (degrees C) from airspeed sensor
/**
failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
bit3:GCS, bit4:fence*/
@I byte  failsafe;
@I byte  wp_num;//current waypoint number
@I short  wp_distance;//distance to target
}

/**
Message appropriate for high latency connections like Iridium (version 2)*/
@id(235) public static class HIGH_LATENCY2{
@I  int  timestamp;//Timestamp (milliseconds since boot or Unix epoch)
MAV_TYPE type;//Type of the MAV (quadrotor, helicopter, etc.)
MAV_AUTOPILOT autopilot;//Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
@I short  custom_mode;//A bitfield for use for autopilot-specific flags (2 byte version).
  int  latitude;//Latitude
  int  longitude;//Longitude
 short  altitude;//Altitude above mean sea level
 short  target_altitude;//Altitude setpoint
@I byte  heading;//Heading
@I byte  target_heading;//Heading setpoint
@I short  target_distance;//Distance to target waypoint or position
@I byte  throttle;//Throttle
@I byte  airspeed;//Airspeed
@I byte  airspeed_sp;//Airspeed setpoint
@I byte  groundspeed;//Groundspeed
@I byte  windspeed;//Windspeed
@I byte  wind_heading;//Wind heading
@I byte  eph;//Maximum error horizontal position since last message
@I byte  epv;//Maximum error vertical position since last message
 byte  temperature_air;//Air temperature from airspeed sensor
 byte  climb_rate;//Maximum climb rate magnitude since last message
 byte  battery;//Battery level (-1 if field not provided).
@I short  wp_num;//Current waypoint number
HL_FAILURE_FLAG failure_flags;//Bitmap of failure flags.
 byte  custom0;//Field for custom payload.
 byte  custom1;//Field for custom payload.
 byte  custom2;//Field for custom payload.
}

/**
Vibration levels and accelerometer clipping*/
@id(241) public static class VIBRATION{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  vibration_x;//Vibration levels on X-axis
 float  vibration_y;//Vibration levels on Y-axis
 float  vibration_z;//Vibration levels on Z-axis
@I  int  clipping_0;//first accelerometer clipping count
@I  int  clipping_1;//second accelerometer clipping count
@I  int  clipping_2;//third accelerometer clipping count
}

/**
This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
will return to and land on. The position is set automatically by the system during the takeoff in case
it was not explicitly set by the operator before or after. The position the system will return to and
land on. The global and local positions encode the position in the respective coordinate frames, while
the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
the point to which the system should fly in normal flight mode and then perform a landing sequence along
the vector*/
@id(242) public static class HOME_POSITION{
  int  latitude;//Latitude (WGS84)
  int  longitude;//Longitude (WGS84)
  int  altitude;//Altitude (MSL). Positive for up.
 float  x;//Local X position of this position in the local coordinate frame
 float  y;//Local Y position of this position in the local coordinate frame
 float  z;//Local Z position of this position in the local coordinate frame
/**
World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
and slope of the groun*/
 @__( 4 )  float  q;
/**
Local X position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone*/
 float  approach_x;
/**
Local Y position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone*/
 float  approach_y;
/**
Local Z position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone*/
 float  approach_z;
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I_ long  time_usec;
}

/**
The position the system will return to and land on. The position is set automatically by the system during
the takeoff in case it was not explicitly set by the operator before or after. The global and local positions
encode the position in the respective coordinate frames, while the q parameter encodes the orientation
of the surface. Under normal conditions it describes the heading and terrain slope, which can be used
by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system
should fly in normal flight mode and then perform a landing sequence along the vector*/
@id(243) public static class SET_HOME_POSITION{
@I byte  target_system;//System ID.
  int  latitude;//Latitude (WGS84)
  int  longitude;//Longitude (WGS84)
  int  altitude;//Altitude (MSL). Positive for up.
 float  x;//Local X position of this position in the local coordinate frame
 float  y;//Local Y position of this position in the local coordinate frame
 float  z;//Local Z position of this position in the local coordinate frame
/**
World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
and slope of the groun*/
 @__( 4 )  float  q;
/**
Local X position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone*/
 float  approach_x;
/**
Local Y position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone*/
 float  approach_y;
/**
Local Z position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone*/
 float  approach_z;
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I_ long  time_usec;
}

/**
The interval between messages for a particular MAVLink message ID. This message is the response to the
MAV_CMD_GET_MESSAGE_INTERVAL command. This interface replaces DATA_STREAM*/
@id(244) public static class MESSAGE_INTERVAL{
@I short  message_id;//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
  int  interval_us;//0 indicates the interval at which it is sent.
}

/**
Provides state for additional features*/
@id(245) public static class EXTENDED_SYS_STATE{
MAV_VTOL_STATE vtol_state;//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
MAV_LANDED_STATE landed_state;//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
}

/**
The location and information of an ADSB vehicle*/
@id(246) public static class ADSB_VEHICLE{
@I  int  ICAO_address;//ICAO address
  int  lat;//Latitude
  int  lon;//Longitude
ADSB_ALTITUDE_TYPE altitude_type;//ADSB altitude type.
  int  altitude;//Altitude(ASL)
@I short  heading;//Course over ground
@I short  hor_velocity;//The horizontal velocity
 short  ver_velocity;//The vertical velocity. Positive is up
@__(9) String  callsign;//The callsign, 8+null
ADSB_EMITTER_TYPE emitter_type;//ADSB emitter type.
@I byte  tslc;//Time since last communication in seconds
ADSB_FLAGS flags;//Bitmap to indicate various statuses including valid data fields
@I short  squawk;//Squawk code
}

/**
Information about a potential collision*/
@id(247) public static class COLLISION{
MAV_COLLISION_SRC src;//Collision data source
@I  int  id;//Unique identifier, domain based on src field
MAV_COLLISION_ACTION action;//Action that is being taken to avoid this collision
MAV_COLLISION_THREAT_LEVEL threat_level;//How concerned the aircraft is about this collision
 float  time_to_minimum_delta;//Estimated time until collision occurs
 float  altitude_minimum_delta;//Closest vertical distance between vehicle and object
 float  horizontal_minimum_delta;//Closest horizontal distance between vehicle and object
}

/**
Message implementing parts of the V2 payload specs in V1 frames for transitional support.*/
 }}
public static class MicroAirVehicle implements  InC, InCPP{
public interface CommunicationInterface extends MicroAirVehicleHandledPacks, GroundControl.CommonPacks {}
public interface MicroAirVehicleHandledPacks  {
@id(248) public static class V2_EXTENSION{
@I byte  target_network;//Network ID (0 for broadcast)
@I byte  target_system;//System ID (0 for broadcast)
@I byte  target_component;//Component ID (0 for broadcast)
/**
A code that identifies the software component that understands this message (analogous to USB device classes
or mime type strings). If this code is less than 32768, it is considered a 'registered' protocol extension
and the corresponding entry should be added to https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml.
Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
Message_types greater than 32767 are considered local experiments and should not be checked in to any
widely distributed codebase*/
@I short  message_type;
/**
Variable length payload. The length must be encoded in the payload as part of the message_type protocol,
e.g. by including the length as payload data, or by terminating the payload data with a non-zero marker.
This is required in order to reconstruct zero-terminated payloads that are (or otherwise would be) trimmed
by MAVLink 2 empty-byte truncation. The entire content of the payload block is opaque unless you understand
the encoding message_type. The particular encoding used can be extension specific and might not always
be documented as part of the MAVLink specification*/
@I @__( 249 )  byte  payload;
}

/**
Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
way for testing new messages and getting experimental debug output*/
@id(249) public static class MEMORY_VECT{
@I short  address;//Starting address of the debug variables
@I byte  ver;//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
@I byte  type;//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q1
 @__( 32 )  byte  value;//Memory contents at specified address
}

/**
To debug something using a named 3D vector.*/
@id(250) public static class DEBUG_VECT{
@__(10) String  name;//Name
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  x;//x
 float  y;//y
 float  z;//z
}

/**
Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
efficient way for testing new messages and getting experimental debug output*/
@id(251) public static class NAMED_VALUE_FLOAT{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@__(10) String  name;//Name of the debug variable
 float  value;//Floating point value
}

/**
Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
efficient way for testing new messages and getting experimental debug output*/
@id(252) public static class NAMED_VALUE_INT{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@__(10) String  name;//Name of the debug variable
  int  value;//Signed integer value
}

/**
Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
They consume quite some bandwidth, so use only for important status and error messages. If implemented
wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)*/
@id(253) public static class STATUSTEXT{
MAV_SEVERITY severity;//Severity of status. Relies on the definitions within RFC-5424.
@__(50) String  text;//Status text message, without null termination character
}

/**
Send a debug value. The index is used to discriminate between values. These values show up in the plot
of QGroundControl as DEBUG N*/
@id(254) public static class DEBUG{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I byte  ind;//index of debug variable
 float  value;//DEBUG value
}

/**
Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
signin*/
@id(256) public static class SETUP_SIGNING{
@I byte  target_system;//system id of the target
@I byte  target_component;//component ID of the target
@I @__( 32 )  byte  secret_key;//signing key
@I long  initial_timestamp;//initial timestamp
}

/**
Report button state change.*/
@id(257) public static class BUTTON_CHANGE{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I  int  last_change_ms;//Time of last change of button state.
@I byte  state;//Bitmap for state of buttons.
}

/**
Control vehicle tone generation (buzzer).*/
@id(258) public static class PLAY_TUNE{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
@__(30) String  tune;//tune in board specific format
@__(200) String  tune2;//tune extension (appended to tune)
}

/**
Information about a camera*/
@id(259) public static class CAMERA_INFORMATION{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I @__( 32 )  byte  vendor_name;//Name of the camera vendor
@I @__( 32 )  byte  model_name;//Name of the camera model
@I  int  firmware_version;//0xff = Major)
 float  focal_length;//Focal length
 float  sensor_size_h;//Image sensor size horizontal
 float  sensor_size_v;//Image sensor size vertical
@I short  resolution_h;//Horizontal image resolution
@I short  resolution_v;//Vertical image resolution
@I byte  lens_id;//Reserved for a lens ID
CAMERA_CAP_FLAGS flags;//Bitmap of camera capability flags.
@I short  cam_definition_version;//Camera definition version (iteration)
/**
Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and
MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements
the Camera Protocol)*/
@__(140) String  cam_definition_uri;
}

/**
Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.*/
@id(260) public static class CAMERA_SETTINGS{
@I  int  time_boot_ms;//Timestamp (time since system boot).
CAMERA_MODE mode_id;//Camera mode
@I_()  float  zoomLevel;//Current zoom level (0.0 to 100.0, NaN if not known)
@I_()  float  focusLevel;//Current focus level (0.0 to 100.0, NaN if not known)
}

/**
Information about a storage medium. This message is sent in response to a request and whenever the status
of the storage changes (STORAGE_STATUS)*/
@id(261) public static class STORAGE_INFORMATION{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I byte  storage_id;//Storage ID (1 for first, 2 for second, etc.)
@I byte  storage_count;//Number of storage devices
STORAGE_STATUS status;//Status of storage
 float  total_capacity;//Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 float  used_capacity;//Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 float  available_capacity;//Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 float  read_speed;//Read speed.
 float  write_speed;//Write speed.
}

/**
Information about the status of a capture.*/
@id(262) public static class CAMERA_CAPTURE_STATUS{
@I  int  time_boot_ms;//Timestamp (time since system boot).
/**
Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
set and capture in progress*/
@I byte  image_status;
@I byte  video_status;//Current status of video capturing (0: idle, 1: capture in progress)
 float  image_interval;//Image capture interval
@I  int  recording_time_ms;//Time since recording started
 float  available_capacity;//Available storage capacity.
}

/**
Information about a captured image*/
@id(263) public static class CAMERA_IMAGE_CAPTURED{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I long  time_utc;//Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
@I byte  camera_id;//Camera ID (1 for first, 2 for second, etc.)
  int  lat;//Latitude where image was taken
  int  lon;//Longitude where capture was taken
  int  alt;//Altitude (MSL) where image was taken
  int  relative_alt;//Altitude above ground
 @__( 4 )  float  q;//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
  int  image_index;//Zero based index of this image (image count since armed -1)
 byte  capture_result;//Boolean indicating success (1) or failure (0) while capturing this image.
@__(205) String  file_url;//URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
}

/**
Information about flight since last arming.*/
@id(264) public static class FLIGHT_INFORMATION{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I long  arming_time_utc;//Timestamp at arming (time since UNIX epoch) in UTC, 0 for unknown
@I long  takeoff_time_utc;//Timestamp at takeoff (time since UNIX epoch) in UTC, 0 for unknown
@I long  flight_uuid;//Universally unique identifier (UUID) of flight, should correspond to name of log files
}

/**
Orientation of a mount*/
@id(265) public static class MOUNT_ORIENTATION{
@I  int  time_boot_ms;//Timestamp (time since system boot).
 float  roll;//Roll in global frame (set to NaN for invalid).
 float  pitch;//Pitch in global frame (set to NaN for invalid).
 float  yaw;//Yaw relative to vehicle (set to NaN for invalid).
@I_()  float  yaw_absolute;//Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
}

/**
A message containing logged data (see also MAV_CMD_LOGGING_START)*/
@id(266) public static class LOGGING_DATA{
@I byte  target_system;//system ID of the target
@I byte  target_component;//component ID of the target
@I short  sequence;//sequence number (can wrap)
@I byte  length;//data length
/**
offset into data where first message starts. This can be used for recovery, when a previous message got
lost (set to 255 if no start exists)*/
@I byte  first_message_offset;
@I @__( 249 )  byte  data;//logged data
}

/**
A message containing logged data which requires a LOGGING_ACK to be sent back*/
@id(267) public static class LOGGING_DATA_ACKED{
@I byte  target_system;//system ID of the target
@I byte  target_component;//component ID of the target
@I short  sequence;//sequence number (can wrap)
@I byte  length;//data length
/**
offset into data where first message starts. This can be used for recovery, when a previous message got
lost (set to 255 if no start exists)*/
@I byte  first_message_offset;
@I @__( 249 )  byte  data;//logged data
}

/**
An ack for a LOGGING_DATA_ACKED message*/
@id(268) public static class LOGGING_ACK{
@I byte  target_system;//system ID of the target
@I byte  target_component;//component ID of the target
@I short  sequence;//sequence number (must match the one in LOGGING_DATA_ACKED)
}

/**
Information about video stream*/
@id(269) public static class VIDEO_STREAM_INFORMATION{
@I byte  stream_id;//Video Stream ID (1 for first, 2 for second, etc.)
@I byte  count;//Number of streams available.
VIDEO_STREAM_TYPE type;//Type of stream.
VIDEO_STREAM_STATUS_FLAGS flags;//Bitmap of stream status flags.
 float  framerate;//Frame rate.
@I short  resolution_h;//Horizontal resolution.
@I short  resolution_v;//Vertical resolution.
@I  int  bitrate;//Bit rate.
@I short  rotation;//Video image rotation clockwise.
@I short  hfov;//Horizontal Field of view.
@__(32) String  name;//Stream name.
/**
Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station
should listen to)*/
@__(160) String  uri;
}

/**
Information about the status of a video stream.*/
@id(270) public static class VIDEO_STREAM_STATUS{
@I byte  stream_id;//Video Stream ID (1 for first, 2 for second, etc.)
VIDEO_STREAM_STATUS_FLAGS flags;//Bitmap of stream status flags
 float  framerate;//Frame rate
@I short  resolution_h;//Horizontal resolution
@I short  resolution_v;//Vertical resolution
@I  int  bitrate;//Bit rate
@I short  rotation;//Video image rotation clockwise
@I short  hfov;//Horizontal Field of view
}

/**
Configure AP SSID and Password.*/
@id(299) public static class WIFI_CONFIG_AP{
@__(32) String  ssid;//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
@__(64) String  password;//Password. Leave it blank for an open AP.
}

/**
Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION and
is used as part of the handshaking to establish which MAVLink version should be used on the network.
Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
should consider adding this into the default decoding state machine to allow the protocol core to respond
directly*/
@id(300) public static class PROTOCOL_VERSION{
@I short  version;//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
@I short  min_version;//Minimum MAVLink version supported
@I short  max_version;//Maximum MAVLink version supported (set to the same value as version by default)
@I @__( 8 )  byte  spec_version_hash;//The first 8 bytes (not characters printed in hex!) of the git hash.
@I @__( 8 )  byte  library_version_hash;//The first 8 bytes (not characters printed in hex!) of the git hash.
}

/**
The location and information of an AIS vessel*/
@id(301) public static class AIS_VESSEL{
@I  int  MMSI;//Mobile Marine Service Identifier, 9 decimal digits
  int  lat;//Latitude
  int  lon;//Longitude
@I short  COG;//Course over ground
@I short  heading;//True heading
@I short  velocity;//Speed over ground
 byte  turn_rate;//Turn rate
AIS_NAV_STATUS navigational_status;//Navigational status
AIS_TYPE type;//Type of vessels
@I short  dimension_bow;//Distance from lat/lon location to bow
@I short  dimension_stern;//Distance from lat/lon location to stern
@I byte  dimension_port;//Distance from lat/lon location to port side
@I byte  dimension_starboard;//Distance from lat/lon location to starboard side
@__(7) String  callsign;//The vessel callsign
@__(20) String  name;//The vessel name
@I short  tslc;//Time since last communication in seconds
AIS_FLAGS flags;//Bitmask to indicate various statuses including valid data fields
}

/**
General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
for the background information. The UAVCAN specification is available at http://uavcan.org*/
@id(310) public static class UAVCAN_NODE_STATUS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I  int  uptime_sec;//Time since the start-up of the node.
UAVCAN_NODE_HEALTH health;//Generalized node health status.
UAVCAN_NODE_MODE mode;//Generalized operating mode.
@I byte  sub_mode;//Not used currently.
@I short  vendor_specific_status_code;//Vendor-specific status information.
}

/**
General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
is available at http://uavcan.org*/
@id(311) public static class UAVCAN_NODE_INFO{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I  int  uptime_sec;//Time since the start-up of the node.
@__(80) String  name;//Node name string. For example, "sapog.px4.io".
@I byte  hw_version_major;//Hardware major version number.
@I byte  hw_version_minor;//Hardware minor version number.
@I @__( 16 )  byte  hw_unique_id;//Hardware unique 128-bit ID.
@I byte  sw_version_major;//Software major version number.
@I byte  sw_version_minor;//Software minor version number.
@I  int  sw_vcs_commit;//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
}

/**
Request to read the value of a parameter with the either the param_id string id or param_index.*/
@id(320) public static class PARAM_EXT_REQUEST_READ{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as strin*/
@__(16) String  param_id;
 short  param_index;//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
}

/**
Request all parameters of this component. After this request, all parameters are emitted.*/
@id(321) public static class PARAM_EXT_REQUEST_LIST{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
}

/**
Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
recipient to keep track of received parameters and allows them to re-request missing parameters after
a loss or timeout*/
@id(322) public static class PARAM_EXT_VALUE{
/**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as strin*/
@__(16) String  param_id;
@__(128) String  param_value;//Parameter value
MAV_PARAM_EXT_TYPE param_type;//Parameter type.
@I short  param_count;//Total number of parameters
@I short  param_index;//Index of this parameter
}

/**
Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
setting a parameter value and the new value is the same as the current value, you will immediately get
a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
a PARAM_ACK_IN_PROGRESS in response*/
@id(323) public static class PARAM_EXT_SET{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
/**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as strin*/
@__(16) String  param_id;
@__(128) String  param_value;//Parameter value
MAV_PARAM_EXT_TYPE param_type;//Parameter type.
}

/**
Response from a PARAM_EXT_SET message.*/
@id(324) public static class PARAM_EXT_ACK{
/**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as strin*/
@__(16) String  param_id;
@__(128) String  param_value;//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
MAV_PARAM_EXT_TYPE param_type;//Parameter type.
PARAM_ACK param_result;//Result code.
}

/**
Obstacle distances in front of the sensor, starting from the left in increment degrees to the right*/
@id(330) public static class OBSTACLE_DISTANCE{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
MAV_DISTANCE_SENSOR sensor_type;//Class id of the distance sensor type.
/**
Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless otherwise
specified in the frame. A value of 0 is valid and means that the obstacle is practically touching the
sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not
used. In a array element, one unit corresponds to 1cm*/
@I @__( 72 )  short  distances;
/**
Angular width in degrees of each array element. Increment direction is clockwise. This field is ignored
if increment_f is non-zero*/
@I byte  increment;
@I short  min_distance;//Minimum distance the sensor can measure.
@I short  max_distance;//Maximum distance the sensor can measure.
/**
Angular width in degrees of each array element as a float. If non-zero then this value is used instead
of the uint8_t increment field. Positive is clockwise direction, negative is counter-clockwise*/
@I_()  float  increment_f;
/**
Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward.
Positive is clockwise direction, negative is counter-clockwise*/
@I_()  float  angle_offset;
/**
Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL,
which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned*/
MAV_FRAME frame;
}

/**
Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard
for aerial vehicles (http://www.ros.org/reps/rep-0147.html)*/
@id(331) public static class ODOMETRY{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
MAV_FRAME frame_id;//Coordinate frame of reference for the pose data.
MAV_FRAME child_frame_id;//Coordinate frame of reference for the velocity in free space (twist) data.
 float  x;//X Position
 float  y;//Y Position
 float  z;//Z Position
 @__( 4 )  float  q;//Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 float  vx;//X linear speed
 float  vy;//Y linear speed
 float  vz;//Z linear speed
 float  rollspeed;//Roll angular speed
 float  pitchspeed;//Pitch angular speed
 float  yawspeed;//Yaw angular speed
/**
Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z,
roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If
unknown, assign NaN value to first element in the array*/
 @__( 21 )  float  pose_covariance;
/**
Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy,
vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second
ROW, etc.). If unknown, assign NaN value to first element in the array*/
 @__( 21 )  float  velocity_covariance;
/**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps*/
@I_ byte  reset_counter;
MAV_ESTIMATOR_TYPE estimator_type;//Type of estimator that is providing the odometry.
}

/**
Describe a trajectory using an array of up-to 5 waypoints in the local frame.*/
@id(332) public static class TRAJECTORY_REPRESENTATION_WAYPOINTS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  valid_points;//Number of valid points (up-to 5 waypoints are possible)
 @__( 5 )  float  pos_x;//X-coordinate of waypoint, set to NaN if not being used
 @__( 5 )  float  pos_y;//Y-coordinate of waypoint, set to NaN if not being used
 @__( 5 )  float  pos_z;//Z-coordinate of waypoint, set to NaN if not being used
 @__( 5 )  float  vel_x;//X-velocity of waypoint, set to NaN if not being used
 @__( 5 )  float  vel_y;//Y-velocity of waypoint, set to NaN if not being used
 @__( 5 )  float  vel_z;//Z-velocity of waypoint, set to NaN if not being used
 @__( 5 )  float  acc_x;//X-acceleration of waypoint, set to NaN if not being used
 @__( 5 )  float  acc_y;//Y-acceleration of waypoint, set to NaN if not being used
 @__( 5 )  float  acc_z;//Z-acceleration of waypoint, set to NaN if not being used
 @__( 5 )  float  pos_yaw;//Yaw angle, set to NaN if not being used
 @__( 5 )  float  vel_yaw;//Yaw rate, set to NaN if not being used
MAV_CMD command;//Scheduled action for each waypoint, UINT16_MAX if not being used.
}

/**
Describe a trajectory using an array of up-to 5 bezier points in the local frame.*/
@id(333) public static class TRAJECTORY_REPRESENTATION_BEZIER{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I byte  valid_points;//Number of valid points (up-to 5 waypoints are possible)
 @__( 5 )  float  pos_x;//X-coordinate of starting bezier point, set to NaN if not being used
 @__( 5 )  float  pos_y;//Y-coordinate of starting bezier point, set to NaN if not being used
 @__( 5 )  float  pos_z;//Z-coordinate of starting bezier point, set to NaN if not being used
 @__( 5 )  float  delta;//Bezier time horizon, set to NaN if velocity/acceleration should not be incorporated
 @__( 5 )  float  pos_yaw;//Yaw, set to NaN for unchanged
}

/**
Report current used cellular network status*/
@id(334) public static class CELLULAR_STATUS{
CELLULAR_NETWORK_STATUS_FLAG status;//Status bitmap
CELLULAR_NETWORK_RADIO_TYPE type;//Cellular network radio type: gsm, cdma, lte...
@I byte  quality;//Cellular network RSSI/RSRP in dBm, absolute value
@I short  mcc;//Mobile country code. If unknown, set to: UINT16_MAX
@I short  mnc;//Mobile network code. If unknown, set to: UINT16_MAX
@I short  lac;//Location area code. If unknown, set to: 0
@I  int  cid;//Cell ID. If unknown, set to: UINT32_MAX
}

/**
Status of the Iridium SBD link.*/
@id(335) public static class ISBD_LINK_STATUS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  timestamp;
/**
Timestamp of the last successful sbd session. The receiving end can infer timestamp format (since 1.1.1970
or since system boot) by checking for the magnitude the number*/
@I long  last_heartbeat;
@I short  failed_sessions;//Number of failed SBD sessions.
@I short  successful_sessions;//Number of successful SBD sessions.
/**
Signal quality equal to the number of bars displayed on the ISU signal strength indicator. Range is 0
to 5, where 0 indicates no signal and 5 indicates maximum signal strength*/
@I byte  signal_quality;
@I byte  ring_pending;//1: Ring call pending, 0: No call pending.
@I byte  tx_session_pending;//1: Transmission session pending, 0: No transmission session pending.
@I byte  rx_session_pending;//1: Receiving session pending, 0: No receiving session pending.
}

/**
The global position resulting from GPS and sensor fusion.*/
@id(340) public static class UTM_GLOBAL_POSITION{
@I long  time;//Time of applicability of position (microseconds since UNIX epoch).
@I @__( 18 )  byte  uas_id;//Unique UAS ID.
  int  lat;//Latitude (WGS84)
  int  lon;//Longitude (WGS84)
  int  alt;//Altitude (WGS84)
  int  relative_alt;//Altitude above ground
 short  vx;//Ground X speed (latitude, positive north)
 short  vy;//Ground Y speed (longitude, positive east)
 short  vz;//Ground Z speed (altitude, positive down)
@I short  h_acc;//Horizontal position uncertainty (standard deviation)
@I short  v_acc;//Altitude uncertainty (standard deviation)
@I short  vel_acc;//Speed uncertainty (standard deviation)
  int  next_lat;//Next waypoint, latitude (WGS84)
  int  next_lon;//Next waypoint, longitude (WGS84)
  int  next_alt;//Next waypoint, altitude (WGS84)
@I short  update_rate;//Time until next update. Set to 0 if unknown or in data driven mode.
UTM_FLIGHT_STATE flight_state;//Flight state
UTM_DATA_AVAIL_FLAGS flags;//Bitwise OR combination of the data available flags.
}

/**
Large debug/prototyping array. The message uses the maximum available payload for data. The array_id and
name fields are used to discriminate between messages in code and in user interfaces (respectively).
Do not use in production code*/
@id(350) public static class DEBUG_FLOAT_ARRAY{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@__(10) String  name;//Name, for human-friendly display in a Ground Control Station
@I short  array_id;//Unique ID used to discriminate between arrays
@I_()  @__( 58 )  float  data;//data
}

/**
Vehicle status report that is sent out while orbit execution is in progress (see MAV_CMD_DO_ORBIT).*/
@id(360) public static class ORBIT_EXECUTION_STATUS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
 float  radius;//Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise
MAV_FRAME frame;//The coordinate system of the fields: x, y, z.
/**
X coordinate of center point. Coordinate system depends on frame field: local = x position in meters *
1e4, global = latitude in degrees * 1e7*/
  int  x;
/**
Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters
* 1e4, global = latitude in degrees * 1e7*/
  int  y;
 float  z;//Altitude of center point. Coordinate system depends on frame field.
}

/**
Status text message (use only for important status and error messages). The full message payload can be
used for status text, but we recommend that updates be kept concise. Note: The message is intended as
a less restrictive replacement for STATUSTEXT*/
@id(365) public static class STATUSTEXT_LONG{
MAV_SEVERITY severity;//Severity of status. Relies on the definitions within RFC-5424.
@__(254) String  text;//Status text message, without null termination character.
}

/**
Smart Battery information (static/infrequent update). Use for updates from: smart battery to flight stack,
flight stack to GCS. Use instead of BATTERY_STATUS for smart batteries*/
@id(370) public static class SMART_BATTERY_INFO{
@I byte  id;//Battery ID
  int  capacity_full_specification;//Capacity when full according to manufacturer, -1: field not provided.
  int  capacity_full;//Capacity when full (accounting for battery degradation), -1: field not provided.
@I short  cycle_count;//Charge/discharge cycle count. -1: field not provided.
  int  serial_number;//Serial number. -1: field not provided.
@__(50) String  device_name;//Static device name. Encode as manufacturer and product names separated using an underscore.
@I short  weight;//Battery weight. 0: field not provided.
@I short  discharge_minimum_voltage;//Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
@I short  charging_minimum_voltage;//Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
@I short  resting_minimum_voltage;//Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
}

/**
Smart Battery information (dynamic). Use for updates from: smart battery to flight stack, flight stack
to GCS. Use instead of BATTERY_STATUS for smart batteries*/
@id(371) public static class SMART_BATTERY_STATUS{
@I short  id;//Battery ID
 short  capacity_remaining;//Remaining battery energy. Values: [0-100], -1: field not provided.
/**
Battery current (through all cells/loads). Positive if discharging, negative if charging. UINT16_MAX:
field not provided*/
 short  current;
 short  temperature;//Battery temperature. -1: field not provided.
MAV_SMART_BATTERY_FAULT fault_bitmask;//Fault/health indications.
  int  time_remaining;//Estimated remaining battery time. -1: field not provided.
/**
The cell number of the first index in the 'voltages' array field. Using this field allows you to specify
cell voltages for batteries with more than 16 cells*/
@I short  cell_offset;
/**
Individual cell voltages. Batteries with more 16 cells can use the cell_offset field to specify the cell
offset for the array specified in the current message . Index values above the valid cell count for this
battery should have the UINT16_MAX value*/
@I @__( 16 )  short  voltages;
}

/**
The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports). This message supersedes
SERVO_OUTPUT_RAW*/
@id(375) public static class ACTUATOR_OUTPUT_STATUS{
@I long  time_usec;//Timestamp (since system boot).
@I  int  active;//Active outputs
 @__( 32 )  float  actuator;//Servo / motor output array values. Zero values indicate unused channels.
}

/**
Time/duration estimates for various events and actions given the current vehicle state and position*/
@id(380) public static class TIME_ESTIMATE_TO_TARGET{
/**
Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g.
RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available*/
  int  safe_return;
/**
Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the
vehicle is landed, or that no time estimate available*/
  int  land;
  int  mission_next_item;//Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available
  int  mission_end;//Estimated time for completing the current mission. -1 means no mission active and/or no estimate available
/**
Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means
no action active and/or no estimate available*/
  int  commanded_action;
}

/**
Message for transporting "arbitrary" variable-length data from one component to another (broadcast is
not forbidden, but discouraged). The encoding of the data is usually extension specific, i.e. determined
by the source, and is usually not documented as part of the MAVLink specification*/
@id(385) public static class TUNNEL{
@I byte  target_system;//System ID (can be 0 for broadcast, but this is discouraged)
@I byte  target_component;//Component ID (can be 0 for broadcast, but this is discouraged)
/**
A code that identifies the content of the payload (0 for unknown, which is the default). If this code
is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the
MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater
than 32767 are considered local experiments and should not be checked in to any widely distributed codebase*/
MAV_TUNNEL_PAYLOAD_TYPE payload_type;
@I byte  payload_length;//Length of the data transported in payload
/**
Variable length payload. The payload length is defined by payload_length. The entire content of this block
is opaque unless you understand the encoding specified by payload_type*/
@I @__( 128 )  byte  payload;
}

/**
Hardware status sent by an onboard computer.*/
@id(390) public static class ONBOARD_COMPUTER_STATUS{
/**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number*/
@I long  time_usec;
@I  int  uptime;//Time since system boot.
/**
Type of the onboard computer: 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer
backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers*/
@I byte  type;
@I @__( 8 )  byte  cpu_cores;//CPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused
/**
Combined CPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load
that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field
is unused*/
@I @__( 10 )  byte  cpu_combined;
@I @__( 4 )  byte  gpu_cores;//GPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused
/**
Combined GPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load
that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field
is unused*/
@I @__( 10 )  byte  gpu_combined;
 byte  temperature_board;//Temperature of the board. A value of INT8_MAX implies the field is unused.
 @__( 8 )  byte  temperature_core;//Temperature of the CPU core. A value of INT8_MAX implies the field is unused.
 @__( 4 )  short  fan_speed;//Fan speeds. A value of INT16_MAX implies the field is unused.
@I  int  ram_usage;//Amount of used RAM on the component system. A value of UINT32_MAX implies the field is unused.
@I  int  ram_total;//Total amount of RAM on the component system. A value of UINT32_MAX implies the field is unused.
/**
Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable). A value of
UINT32_MAX implies the field is unused*/
@I @__( 4 )   int  storage_type;
@I @__( 4 )   int  storage_usage;//Amount of used storage space on the component system. A value of UINT32_MAX implies the field is unused
@I @__( 4 )   int  storage_total;//Total amount of storage space on the component system. A value of UINT32_MAX implies the field is unused
/**
Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh
proprietar*/
@I @__( 6 )   int  link_type;
@I @__( 6 )   int  link_tx_rate;//Network traffic from the component system. A value of UINT32_MAX implies the field is unused.
@I @__( 6 )   int  link_rx_rate;//Network traffic to the component system. A value of UINT32_MAX implies the field is unused.
@I @__( 6 )   int  link_tx_max;//Network capacity from the component system. A value of UINT32_MAX implies the field is unused.
@I @__( 6 )   int  link_rx_max;//Network capacity to the component system. A value of UINT32_MAX implies the field is unused.
}

/**
Information about a component. For camera components instead use CAMERA_INFORMATION, and for autopilots
use AUTOPILOT_VERSION. Components including GCSes should consider supporting requests of this message
via MAV_CMD_REQUEST_MESSAGE*/
@id(395) public static class COMPONENT_INFORMATION{
@I  int  time_boot_ms;//Timestamp (time since system boot).
@I @__( 32 )  byte  vendor_name;//Name of the component vendor
@I @__( 32 )  byte  model_name;//Name of the component model
@I  int  firmware_version;//0xff = Major)
@I  int  hardware_version;//0xff = Major)
COMPONENT_CAP_FLAGS capability_flags;//Bitmap of component capability flags.
@I short  component_definition_version;//Component definition version (iteration)
/**
Component definition URI (if any, otherwise only basic functions will be available). The XML format is
not yet specified and work in progress.*/
@__(140) String  component_definition_uri;
}

/**
Play vehicle tone/tune (buzzer). Supersedes message PLAY_TUNE.*/
@id(400) public static class PLAY_TUNE_V2{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
TUNE_FORMAT format;//Tune format
@__(248) String  tune;//Tune definition as a NULL-terminated string.
}

/**
Tune formats supported by vehicle. This should be emitted as response to MAV_CMD_REQUEST_MESSAGE.*/
@id(401) public static class SUPPORTED_TUNES{
@I byte  target_system;//System ID
@I byte  target_component;//Component ID
TUNE_FORMAT format;//Bitfield of supported tune formats.
}

/**
Cumulative distance traveled for each reported wheel.*/
@id(9000) public static class WHEEL_DISTANCE{
@I long  time_usec;//Timestamp (synced to UNIX time or since system boot).
@I byte  count;//Number of wheels reported.
/**
Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease
them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions
must be agreed/understood by the endpoints*/
 @__( 16 )  double  distance;
}

/**
Data for filling the OpenDroneID Basic ID message. This and the below messages are primarily meant for
feeding data to/from an OpenDroneID implementation. E.g. https://github.com/opendroneid/opendroneid-core-*/
@id(12900) public static class OPEN_DRONE_ID_BASIC_ID{
MAV_ODID_ID_TYPE id_type;//Indicates the format for the uas_id field of this message.
MAV_ODID_UA_TYPE ua_type;//Indicates the type of UA (Unmanned Aircraft).
/**
UAS (Unmanned Aircraft System) ID following the format specified by id_type. Shall be filled with nulls
in the unused portion of the field*/
@I @__( 20 )  byte  uas_id;
}

/**
Data for filling the OpenDroneID Location message. The float data types are 32-bit IEEE 754. The Location
message provides the location, altitude, direction and speed of the aircraft*/
@id(12901) public static class OPEN_DRONE_ID_LOCATION{
MAV_ODID_STATUS status;//Indicates whether the Unmanned Aircraft is on the ground or in the air.
/**
Direction over ground (not heading, but direction of movement) in degrees * 100: 0.0 - 359.99 degrees.
If unknown: 361.00 degrees*/
@I short  direction;
@I short  speed_horizontal;//Ground speed. Positive only. If unknown: 255.00 m/s. If speed is larger than 254.25 m/s, use 254.25 m/s
/**
The vertical speed. Up is positive. If unknown: 63.00 m/s. If speed is larger than 62.00 m/s, use 62.00
m/s*/
 short  speed_vertical;
  int  latitude;//Current latitude of the UA (Unmanned Aircraft). If unknown: 0 deg (both Lat/Lon).
  int  longitude;//Current longitude of the UA (Unmanned Aircraft). If unknown: 0 deg (both Lat/Lon).
/**
The altitude calculated from the barometric pressue. Reference is against 29.92inHg or 1013.2mb. If unknown:
-1000 m*/
 float  altitude_barometric;
 float  altitude_geodetic;//The geodetic altitude as defined by WGS84. If unknown: -1000 m.
MAV_ODID_HEIGHT_REF height_reference;//Indicates the reference point for the height field.
/**
The current height of the UA (Unmanned Aircraft) above the take-off location or the ground as indicated
by height_reference. If unknown: -1000 m*/
 float  height;
MAV_ODID_HOR_ACC horizontal_accuracy;//The accuracy of the horizontal position.
MAV_ODID_VER_ACC vertical_accuracy;//The accuracy of the vertical position.
MAV_ODID_VER_ACC barometer_accuracy;//The accuracy of the barometric altitude.
MAV_ODID_SPEED_ACC speed_accuracy;//The accuracy of the horizontal and vertical speed.
/**
Seconds after the full hour. Typically the GPS outputs a time of week value in milliseconds. That value
can be easily converted for this field using ((float) (time_week_ms % (60*60*1000))) / 1000*/
 float  timestamp;
MAV_ODID_TIME_ACC timestamp_accuracy;//The accuracy of the timestamps.
}

/**
Data for filling the OpenDroneID Authentication message. The Authentication Message defines a field that
can provide a means of authenticity for the identity of the UAS (Unmanned Aircraft System). The Authentication
message can have two different formats. Five data pages are supported. For data page 0, the fields PageCount,
Length and TimeStamp are present and AuthData is only 17 bytes. For data page 1 through 4, PageCount,Length
and TimeStamp are not present and the size of AuthData is 23 bytes*/
@id(12902) public static class OPEN_DRONE_ID_AUTHENTICATION{
MAV_ODID_AUTH_TYPE authentication_type;//Indicates the type of authentication.
@I byte  data_page;//Allowed range is 0 - 4.
@I byte  page_count;//This field is only present for page 0. Allowed range is 0 - 5.
/**
This field is only present for page 0. Total bytes of authentication_data from all data pages. Allowed
range is 0 - 109 (17 + 23*4)*/
@I byte  length;
@I  int  timestamp;//This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
/**
Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes.
Shall be filled with nulls in the unused portion of the field*/
@I @__( 23 )  byte  authentication_data;
}

/**
Data for filling the OpenDroneID Self ID message. The Self ID Message is an opportunity for the operator
to (optionally) declare their identity and purpose of the flight. This message can provide additional
information that could reduce the threat profile of a UA (Unmanned Aircraft) flying in a particular area
or manner*/
@id(12903) public static class OPEN_DRONE_ID_SELF_ID{
MAV_ODID_DESC_TYPE description_type;//Indicates the type of the description field.
/**
Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused
portion of the field*/
@__(23) String  description;
}

/**
Data for filling the OpenDroneID System message. The System Message contains general system information
including the operator location and possible aircraft group information*/
@id(12904) public static class OPEN_DRONE_ID_SYSTEM{
MAV_ODID_LOCATION_SRC flags;//Specifies the location source for the operator location.
  int  operator_latitude;//Latitude of the operator. If unknown: 0 deg (both Lat/Lon).
  int  operator_longitude;//Longitude of the operator. If unknown: 0 deg (both Lat/Lon).
@I short  area_count;//Number of aircraft in the area, group or formation (default 1).
@I short  area_radius;//Radius of the cylindrical area of the group or formation (default 0).
 float  area_ceiling;//Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
 float  area_floor;//Area Operations Floor relative to WGS84. If unknown: -1000 m.
}

/**
Data for filling the OpenDroneID Operator ID message, which contains the CAA (Civil Aviation Authority)
issued operator ID*/
@id(12905) public static class OPEN_DRONE_ID_OPERATOR_ID{
MAV_ODID_OPERATOR_ID_TYPE operator_id_type;//Indicates the type of the operator_id field.
/**
Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused
portion of the field*/
@__(20) String  operator_id;
}

/**
An OpenDroneID message pack is a container for multiple encoded OpenDroneID messages (i.e. not in the
format given for the above messages descriptions but after encoding into the compressed OpenDroneID byte
format). Used e.g. when transmitting on Bluetooth 5.0 Long Range/Extended Advertising or on WiFi Neighbor
Aware Networking*/
@id(12915) public static class OPEN_DRONE_ID_MESSAGE_PACK{
/**
This field must currently always be equal to 25 bytes, since all encoded OpenDroneID messages are specificed
to have this length*/
@I byte  single_message_size;
@I byte  msg_pack_size;//Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 10.
/**
Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the
field*/
@I @__( 250 )  byte  messages;
}

/**
Array test #0.*/
@id(150) public static class ARRAY_TEST_0{
@I byte  v1;//Stub field
 @__( 4 )  byte  ar_i8;//Value array
@I @__( 4 )  byte  ar_u8;//Value array
@I @__( 4 )  short  ar_u16;//Value array
@I @__( 4 )   int  ar_u32;//Value array
}

/**
Array test #1.*/
@id(151) public static class ARRAY_TEST_1{
@I @__( 4 )   int  ar_u32;//Value array
}

/**
Array test #3.*/
@id(153) public static class ARRAY_TEST_3{
@I byte  v;//Stub field
@I @__( 4 )   int  ar_u32;//Value array
}

/**
Array test #4.*/
@id(154) public static class ARRAY_TEST_4{
@I @__( 4 )   int  ar_u32;//Value array
@I byte  v;//Stub field
}

/**
Array test #5.*/
@id(155) public static class ARRAY_TEST_5{
@__(5) String  c1;//Value array
@__(5) String  c2;//Value array
}

/**
Array test #6.*/
@id(156) public static class ARRAY_TEST_6{
@I byte  v1;//Stub field
@I short  v2;//Stub field
@I  int  v3;//Stub field
@I @__( 2 )   int  ar_u32;//Value array
 @__( 2 )   int  ar_i32;//Value array
@I @__( 2 )  short  ar_u16;//Value array
 @__( 2 )  short  ar_i16;//Value array
@I @__( 2 )  byte  ar_u8;//Value array
 @__( 2 )  byte  ar_i8;//Value array
@__(32) String  ar_c;//Value array
 @__( 2 )  double  ar_d;//Value array
 @__( 2 )  float  ar_f;//Value array
}

/**
Array test #7.*/
@id(157) public static class ARRAY_TEST_7{
 @__( 2 )  double  ar_d;//Value array
 @__( 2 )  float  ar_f;//Value array
@I @__( 2 )   int  ar_u32;//Value array
 @__( 2 )   int  ar_i32;//Value array
@I @__( 2 )  short  ar_u16;//Value array
 @__( 2 )  short  ar_i16;//Value array
@I @__( 2 )  byte  ar_u8;//Value array
 @__( 2 )  byte  ar_i8;//Value array
@__(32) String  ar_c;//Value array
}

/**
Array test #8.*/
@id(158) public static class ARRAY_TEST_8{
@I  int  v3;//Stub field
 @__( 2 )  double  ar_d;//Value array
@I @__( 2 )  short  ar_u16;//Value array
}
}}
/**
Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD
for information about the structure of the MAV_CMD entrie*/
enum MAV_CMD {;
final int

/**
Navigate to waypoint.
1	Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
2	Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
3	0 to pass through the WP, if 	>	0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
4	Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_WAYPOINT = 16, 
/**
Loiter around this waypoint an unlimited amount of time
1	Empty
2	Empty
3	Radius around waypoint. If positive loiter clockwise, else counter-clockwise
4	Desired yaw angle. NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_LOITER_UNLIM = 17, 
/**
Loiter around this waypoint for X turns
1	Number of turns.
2	Empty
3	Radius around waypoint. If positive loiter clockwise, else counter-clockwise
4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle. NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_LOITER_TURNS = 18, 
/**
Loiter around this waypoint for X seconds
1	Loiter time.
2	Empty
3	Radius around waypoint. If positive loiter clockwise, else counter-clockwise.
4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle.  NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_LOITER_TIME = 19, 
/**
Return to launch location
1	Empty
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20, 
/**
Land at location.
1	Minimum target altitude if landing is aborted (0 = undefined/use system default).
2	Precision land mode.
3	Empty.
4	Desired yaw angle. NaN for unchanged.
5	Latitude.
6	Longitude.
7	Landing altitude (ground level in current frame).*/
MAV_CMD_NAV_LAND = 21, 
/**
Takeoff from ground / hand
1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
2	Empty
3	Empty
4	Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_TAKEOFF = 22, 
/**
Land at local position (local frame only)
1	Landing target number (if available)
2	Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
3	Landing descend rate
4	Desired yaw angle
5	Y-axis position
6	X-axis position
7	Z-axis / ground level position*/
MAV_CMD_NAV_LAND_LOCAL = 23, 
/**
Takeoff from local position (local frame only)
1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
2	Empty
3	Takeoff ascend rate
4	Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these
5	Y-axis position
6	X-axis position
7	Z-axis position*/
MAV_CMD_NAV_TAKEOFF_LOCAL = 24, 
/**
Vehicle following, i.e. this waypoint represents the position of a moving vehicle
1	Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
2	Ground speed of vehicle to be followed
3	Radius around waypoint. If positive loiter clockwise, else counter-clockwise
4	Desired yaw angle.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_FOLLOW = 25, 
/**
Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached
1	Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Desired altitude*/
MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30, 
/**
Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
 Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
 Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter
until heading toward the next waypoint
1	Heading Required (0 = False)
2	Radius. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
3	Empty
4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_LOITER_TO_ALT = 31, 
/**
Begin following a target
1	System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.
2	RESERVED
3	RESERVED
4	Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.
5	Altitude above home. (used if mode=2)
6	RESERVED
7	Time to land in which the MAV should go to the default position hold mode after a message RX timeout.*/
MAV_CMD_DO_FOLLOW = 32, 
/**
Reposition the MAV after a follow target command has been sent
1	Camera q1 (where 0 is on the ray from the camera to the tracking device)
2	Camera q2
3	Camera q3
4	Camera q4
5	altitude offset from target
6	X offset from target
7	Y offset from target*/
MAV_CMD_DO_FOLLOW_REPOSITION = 33, 
/**
Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results
in using defaults
1	Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise.
2	Tangential Velocity. NaN: Vehicle configuration default.
3	Yaw behavior of the vehicle.
4	Reserved (e.g. for dynamic center beacon options)
5	Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
6	Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
7	Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.*/
MAV_CMD_DO_ORBIT = 34, 
/**
Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
1	Region of interest mode.
2	Waypoint index/ target ID. (see MAV_ROI enum)
3	ROI index (allows a vehicle to manage multiple ROI's)
4	Empty
5	x the location of the fixed ROI (see MAV_FRAME)
6	y
7	z*/
MAV_CMD_NAV_ROI = 80, 
/**
Control autonomous path planning on the MAV.
1	0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
2	0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
3	Empty
4	Yaw angle at goal
5	Latitude/X of goal
6	Longitude/Y of goal
7	Altitude/Z of goal*/
MAV_CMD_NAV_PATHPLANNING = 81, 
/**
Navigate to waypoint using a spline path.
1	Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
2	Empty
3	Empty
4	Empty
5	Latitude/X of goal
6	Longitude/Y of goal
7	Altitude/Z of goal*/
MAV_CMD_NAV_SPLINE_WAYPOINT = 82, 
/**
Takeoff from ground using VTOL mode, and transition to forward flight with specified heading.
1	Empty
2	Front transition heading.
3	Empty
4	Yaw angle. NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_VTOL_TAKEOFF = 84, 
/**
Land using VTOL mode
1	Empty
2	Empty
3	Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
4	Yaw angle. NaN for unchanged.
5	Latitude
6	Longitude
7	Altitude (ground level)*/
MAV_CMD_NAV_VTOL_LAND = 85, 
/**
hand control over to an external controller
1	On / Off (	>	0.5f on)
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_NAV_GUIDED_ENABLE = 92, 
/**
Delay the next navigation command a number of seconds or until a specified time
1	Delay (-1 to enable time-of-day fields)
2	hour (24h format, UTC, -1 to ignore)
3	minute (24h format, UTC, -1 to ignore)
4	second (24h format, UTC)
5	Empty
6	Empty
7	Empty*/
MAV_CMD_NAV_DELAY = 93, 
/**
Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload
has reached the ground, and then releases the payload. If ground is not detected before the reaching
the maximum descent value (param1), the command will complete without releasing the payload
1	Maximum distance to descend.
2	Empty
3	Empty
4	Empty
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_PAYLOAD_PLACE = 94, 
/**
NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeratio
1	Empty
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_NAV_LAST = 95, 
/**
Delay mission state machine.
1	Delay
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_CONDITION_DELAY = 112, 
/**
Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
1	Descent / Ascend rate.
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Finish Altitude*/
MAV_CMD_CONDITION_CHANGE_ALT = 113, 
/**
Delay mission state machine until within desired distance of next NAV point.
1	Distance.
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_CONDITION_DISTANCE = 114, 
/**
Reach a certain target angle.
1	target angle, 0 is north
2	angular speed
3	direction: -1: counter clockwise, 1: clockwise
4	0: absolute angle, 1: relative offset
5	Empty
6	Empty
7	Empty*/
MAV_CMD_CONDITION_YAW = 115, 
/**
NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeratio
1	Empty
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_CONDITION_LAST = 159, 
/**
Set system mode.
1	Mode
2	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
3	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_MODE = 176, 
/**
Jump to the desired command in the mission list.  Repeat this action only the specified number of time
1	Sequence number
2	Repeat count
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_JUMP = 177, 
/**
Change speed and/or throttle set points.
1	Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
2	Speed (-1 indicates no change)
3	Throttle (-1 indicates no change)
4	0: absolute, 1: relative
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_CHANGE_SPEED = 178, 
/**
Changes the home location either to the current location or a specified location.
1	Use current (1=use current location, 0=use specified location)
2	Empty
3	Empty
4	Empty
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_DO_SET_HOME = 179, 
/**
Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
of the parameter
1	Parameter number
2	Parameter value
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_PARAMETER = 180, 
/**
Set a relay to a condition.
1	Relay instance number.
2	Setting. (1=on, 0=off, others possible depending on system hardware)
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_RELAY = 181, 
/**
Cycle a relay on and off for a desired number of cycles with a desired period.
1	Relay instance number.
2	Cycle count.
3	Cycle time.
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_REPEAT_RELAY = 182, 
/**
Set a servo to a desired PWM value.
1	Servo instance number.
2	Pulse Width Modulation.
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_SERVO = 183, 
/**
Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period
1	Servo instance number.
2	Pulse Width Modulation.
3	Cycle count.
4	Cycle time.
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_REPEAT_SERVO = 184, 
/**
Terminate flight immediately
1	Flight termination activated if 	>	0.5
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_FLIGHTTERMINATION = 185, 
/**
Change altitude set point.
1	Altitude.
2	Frame of new altitude.
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_CHANGE_ALTITUDE = 186, 
/**
Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
will be used to help find the closest landing sequence
1	Empty
2	Empty
3	Empty
4	Empty
5	Latitude
6	Longitude
7	Empty*/
MAV_CMD_DO_LAND_START = 189, 
/**
Mission command to perform a landing from a rally point.
1	Break altitude
2	Landing speed
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_RALLY_LAND = 190, 
/**
Mission command to safely abort an autonomous landing.
1	Altitude
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_GO_AROUND = 191, 
/**
Reposition the vehicle to a specific WGS84 global position.
1	Ground speed, less than 0 (-1) for default
2	Bitmask of option flags.
3	Reserved
4	Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
5	Latitude
6	Longitude
7	Altitude (meters)*/
MAV_CMD_DO_REPOSITION = 192, 
/**
If in a GPS controlled position mode, hold the current position or continue.
1	0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
2	Reserved
3	Reserved
4	Reserved
5	Reserved
6	Reserved
7	Reserved*/
MAV_CMD_DO_PAUSE_CONTINUE = 193, 
/**
Set moving direction to forward or reverse.
1	Direction (0=Forward, 1=Reverse)
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_REVERSE = 194, 
/**
Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system
to control the vehicle attitude and the attitude of various sensors such as cameras
1	Empty
2	Empty
3	Empty
4	Empty
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_DO_SET_ROI_LOCATION = 195, 
/**
Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This
can then be used by the vehicles control system to control the vehicle attitude and the attitude of various
sensors such as cameras
1	Empty
2	Empty
3	Empty
4	Empty
5	pitch offset from next waypoint
6	roll offset from next waypoint
7	yaw offset from next waypoint*/
MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196, 
/**
Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This
can then be used by the vehicles control system to control the vehicle attitude and the attitude of various
sensors such as cameras
1	Empty
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_ROI_NONE = 197, 
/**
Mount tracks system with specified system ID. Determination of target vehicle position may be done with
GLOBAL_POSITION_INT or any other means
1	sysid*/
MAV_CMD_DO_SET_ROI_SYSID = 198, 
/**
Control onboard camera system.
1	Camera ID (-1 for all)
2	Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
3	Transmission mode: 0: video stream, 	>	0: single images every n seconds
4	Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_CONTROL_VIDEO = 200, 
/**
Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
1	Region of interest mode.
2	Waypoint index/ target ID (depends on param 1).
3	Region of interest index. (allows a vehicle to manage multiple ROI's)
4	Empty
5	MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
6	MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
7	MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude*/
MAV_CMD_DO_SET_ROI = 201, 
/**
Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX
messages and camera definition files (see https://mavlink.io/en/services/camera_def.html )
1	Modes: P, TV, AV, M, Etc.
2	Shutter speed: Divisor number for one second.
3	Aperture: F stop number.
4	ISO number e.g. 80, 100, 200, Etc.
5	Exposure type enumerator.
6	Command Identity.
7	Main engine cut-off time before camera trigger. (0 means no cut-off)*/
MAV_CMD_DO_DIGICAM_CONFIGURE = 202, 
/**
Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX
messages and camera definition files (see https://mavlink.io/en/services/camera_def.html )
1	Session control e.g. show/hide lens
2	Zoom's absolute position
3	Zooming step value to offset zoom from the current position
4	Focus Locking, Unlocking or Re-locking
5	Shooting Command
6	Command Identity
7	Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.*/
MAV_CMD_DO_DIGICAM_CONTROL = 203, 
/**
Mission command to configure a camera or antenna mount
1	Mount operation mode
2	stabilize roll? (1 = yes, 0 = no)
3	stabilize pitch? (1 = yes, 0 = no)
4	stabilize yaw? (1 = yes, 0 = no)
5	roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
6	pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
7	yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)*/
MAV_CMD_DO_MOUNT_CONFIGURE = 204, 
/**
Mission command to control a camera or antenna mount
1	pitch depending on mount mode (degrees or degrees/second depending on pitch input).
2	roll depending on mount mode (degrees or degrees/second depending on roll input).
3	yaw depending on mount mode (degrees or degrees/second depending on yaw input).
4	altitude depending on mount mode.
5	latitude, set if appropriate mount mode.
6	longitude, set if appropriate mount mode.
7	Mount mode.*/
MAV_CMD_DO_MOUNT_CONTROL = 205, 
/**
Mission command to set camera trigger distance for this flight. The camera is triggered each time this
distance is exceeded. This command can also be used to set the shutter integration time for the camera
1	Camera trigger distance. 0 to stop triggering.
2	Camera shutter integration time. -1 or 0 to ignore
3	Trigger camera once immediately. (0 = no trigger, 1 = trigger)
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206, 
/**
Mission command to enable the geofence
1	enable? (0=disable, 1=enable, 2=disable_floor_only)
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_FENCE_ENABLE = 207, 
/**
Mission command to trigger a parachute
1	action
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_PARACHUTE = 208, 
/**
Mission command to perform motor test.
1	Motor instance number. (from 1 to max number of motors on the vehicle)
2	Throttle type.
3	Throttle.
4	Timeout.
5	Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)
6	Motor test order.
7	Empty*/
MAV_CMD_DO_MOTOR_TEST = 209, 
/**
Change to/from inverted flight.
1	Inverted flight. (0=normal, 1=inverted)
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_INVERTED_FLIGHT = 210, 
/**
Sets a desired vehicle turn angle and speed change.
1	Yaw angle to adjust steering by.
2	Speed.
3	Final angle. (0=absolute, 1=relative)
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_NAV_SET_YAW_SPEED = 213, 
/**
Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
triggered each time this interval expires. This command can also be used to set the shutter integration
time for the camera
1	Camera trigger cycle time. -1 or 0 to ignore.
2	Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214, 
/**
Mission command to control a camera or antenna mount, using a quaternion as reference.
1	quaternion param q1, w (1 in null-rotation)
2	quaternion param q2, x (0 in null-rotation)
3	quaternion param q3, y (0 in null-rotation)
4	quaternion param q4, z (0 in null-rotation)
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220, 
/**
set id of master controller
1	System ID
2	Component ID
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_GUIDED_MASTER = 221, 
/**
Set limits for external control
1	Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.
2	Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.
3	Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.
4	Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_GUIDED_LIMITS = 222, 
/**
Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
state. It is intended for vehicles with internal combustion engine
1	0: Stop engine, 1:Start Engine
2	0: Warm start, 1:Cold start. Controls use of choke where applicable
3	Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
4	Empty
5	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_ENGINE_CONTROL = 223, 
/**
Set the mission item with sequence number seq as current item. This means that the MAV will continue to
this mission item on the shortest path (not following the mission items in-between)
1	Mission sequence value to set
2	Empty
3	Empty
4	Empty
5	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_SET_MISSION_CURRENT = 224, 
/**
NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
1	Empty
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_DO_LAST = 240, 
/**
Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
Calibration, only one sensor should be set in a single message and all others should be zero
1	1: gyro calibration, 3: gyro temperature calibration
2	1: magnetometer calibration
3	1: ground pressure calibration
4	1: radio RC calibration, 2: RC trim calibration
5	1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
6	1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
7	1: ESC calibration, 3: barometer temperature calibration*/
MAV_CMD_PREFLIGHT_CALIBRATION = 241, 
/**
Set sensor offsets. This command will be only accepted if in pre-flight mode.
1	Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
2	X axis offset (or generic dimension 1), in the sensor's raw units
3	Y axis offset (or generic dimension 2), in the sensor's raw units
4	Z axis offset (or generic dimension 3), in the sensor's raw units
5	Generic dimension 4, in the sensor's raw units
6	Generic dimension 5, in the sensor's raw units
7	Generic dimension 6, in the sensor's raw units*/
MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242, 
/**
Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
1	1: Trigger actuator ID assignment and direction mapping.
2	Reserved
3	Reserved
4	Reserved
5	Reserved
6	Reserved
7	Reserved*/
MAV_CMD_PREFLIGHT_UAVCAN = 243, 
/**
Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
mode
1	Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
2	Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
3	Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, 	>	1: logging rate (e.g. set to 1000 for 1000 Hz logging)
4	Reserved
5	Empty
6	Empty
7	Empty*/
MAV_CMD_PREFLIGHT_STORAGE = 245, 
/**
Request the reboot or shutdown of system components.
1	0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
2	0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
3	WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
4	WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
5	Reserved, send 0
6	Reserved, send 0
7	WIP: ID (e.g. camera ID -1 for all IDs)*/
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246, 
/**
Override current mission with command to pause mission, pause mission and move to position, continue/resume
mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether
it holds in place or moves to another position
1	MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.
2	MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.
3	Coordinate frame of hold point.
4	Desired yaw angle.
5	Latitude / X position.
6	Longitude / Y position.
7	Altitude / Z position.*/
MAV_CMD_OVERRIDE_GOTO = 252, 
/**
start running a mission
1	first_item: the first mission item to run
2	last_item:  the last mission item to run (after this item is run, the mission ends)*/
MAV_CMD_MISSION_START = 300, 
/**
Arms / Disarms a component
1	0: disarm, 1: arm
2	0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)*/
MAV_CMD_COMPONENT_ARM_DISARM = 400, 
/**
Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external
to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system
itself, e.g. an indicator light)
1	0: Illuminators OFF, 1: Illuminators ON*/
MAV_CMD_ILLUMINATOR_ON_OFF = 405, 
/**
Request the home position from the vehicle.
1	Reserved
2	Reserved
3	Reserved
4	Reserved
5	Reserved
6	Reserved
7	Reserved*/
MAV_CMD_GET_HOME_POSITION = 410, 
/**
Starts receiver pairing.
1	0:Spektrum.
2	RC type.*/
MAV_CMD_START_RX_PAIR = 500, 
/**
Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the
command and then emit its response in a MESSAGE_INTERVAL message
1	The MAVLink message ID*/
MAV_CMD_GET_MESSAGE_INTERVAL = 510, 
/**
Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM
1	The MAVLink message ID
2	The interval between two messages. Set to -1 to disable and 0 to request default rate.
7	Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.*/
MAV_CMD_SET_MESSAGE_INTERVAL = 511, 
/**
Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version
of MAV_CMD_SET_MESSAGE_INTERVAL)
1	The MAVLink message ID of the requested message.
2	Index id (if appropriate). The use of this parameter (if any), must be defined in the requested message.
3	The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
4	The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
5	The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
6	The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
7	Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.*/
MAV_CMD_REQUEST_MESSAGE = 512, 
/**
Request MAVLink protocol version compatibility
1	1: Request supported protocol versions by all nodes on the network
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_PROTOCOL_VERSION = 519, 
/**
Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in
an AUTOPILOT_VERSION messag
1	1: Request autopilot version
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520, 
/**
Request camera information (CAMERA_INFORMATION).
1	0: No action 1: Request camera capabilities
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_CAMERA_INFORMATION = 521, 
/**
Request camera settings (CAMERA_SETTINGS).
1	0: No Action 1: Request camera settings
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_CAMERA_SETTINGS = 522, 
/**
Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific
component's storage
1	Storage ID (0 for all, 1 for first, 2 for second, etc.)
2	0: No Action 1: Request storage information
3	Reserved (all remaining params)*/
MAV_CMD_REQUEST_STORAGE_INFORMATION = 525, 
/**
Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's
target_component to target a specific component's storage
1	Storage ID (1 for first, 2 for second, etc.)
2	0: No action 1: Format storage
3	Reserved (all remaining params)*/
MAV_CMD_STORAGE_FORMAT = 526, 
/**
Request camera capture status (CAMERA_CAPTURE_STATUS)
1	0: No Action 1: Request camera capture status
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527, 
/**
Request flight information (FLIGHT_INFORMATION)
1	1: Request flight information
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528, 
/**
Reset all camera settings to Factory Default
1	0: No Action 1: Reset all settings
2	Reserved (all remaining params)*/
MAV_CMD_RESET_CAMERA_SETTINGS = 529, 
/**
Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
command after a mode change if the camera supports video streaming
1	Reserved (Set to 0)
2	Camera mode
3	Reserved (all remaining params)*/
MAV_CMD_SET_CAMERA_MODE = 530, 
/**
Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved
values
1	Zoom type
2	Zoom value. The range of valid values depend on the zoom type.
3	Reserved (all remaining params)*/
MAV_CMD_SET_CAMERA_ZOOM = 531, 
/**
Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved
values
1	Focus type
2	Focus value
3	Reserved (all remaining params)*/
MAV_CMD_SET_CAMERA_FOCUS = 532, 
/**
Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
1	Tag.*/
MAV_CMD_JUMP_TAG = 600, 
/**
Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A
mission should contain a single matching tag for each jump. If this is not the case then a jump to a
missing tag should complete the mission, and a jump where there are multiple matching tags should always
select the one with the lowest mission sequence number
1	Target tag to jump to.
2	Repeat count.*/
MAV_CMD_DO_JUMP_TAG = 601, 
/**
Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values
1	Reserved (Set to 0)
2	Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).
3	Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.
4	Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1). Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted. Use 0 to ignore it.
5	Reserved (all remaining params)*/
MAV_CMD_IMAGE_START_CAPTURE = 2000, 
/**
Stop image capture sequence Use NaN for reserved values.
1	Reserved (Set to 0)
2	Reserved (all remaining params)*/
MAV_CMD_IMAGE_STOP_CAPTURE = 2001, 
/**
Re-request a CAMERA_IMAGE_CAPTURE message. Use NaN for reserved values.
1	Sequence number for missing CAMERA_IMAGE_CAPTURE message
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002, 
/**
Enable or disable on-board camera triggering system.
1	Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
2	1 to reset the trigger sequence, -1 or 0 to ignore
3	1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore*/
MAV_CMD_DO_TRIGGER_CONTROL = 2003, 
/**
Starts video capture (recording). Use NaN for reserved values.
1	Video Stream ID (0 for all streams)
2	Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)
3	Reserved (all remaining params)*/
MAV_CMD_VIDEO_START_CAPTURE = 2500, 
/**
Stop the current video capture (recording). Use NaN for reserved values.
1	Video Stream ID (0 for all streams)
2	Reserved (all remaining params)*/
MAV_CMD_VIDEO_STOP_CAPTURE = 2501, 
/**
Start video streaming
1	Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
2	Reserved*/
MAV_CMD_VIDEO_START_STREAMING = 2502, 
/**
Stop the given video stream
1	Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
2	Reserved*/
MAV_CMD_VIDEO_STOP_STREAMING = 2503, 
/**
Request video stream information (VIDEO_STREAM_INFORMATION)
1	Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504, 
/**
Request video stream status (VIDEO_STREAM_STATUS)
1	Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
2	Reserved (all remaining params)*/
MAV_CMD_REQUEST_VIDEO_STREAM_STATUS = 2505, 
/**
Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
1	Format: 0: ULog
2	Reserved (set to 0)
3	Reserved (set to 0)
4	Reserved (set to 0)
5	Reserved (set to 0)
6	Reserved (set to 0)
7	Reserved (set to 0)*/
MAV_CMD_LOGGING_START = 2510, 
/**
Request to stop streaming log data over MAVLink
1	Reserved (set to 0)
2	Reserved (set to 0)
3	Reserved (set to 0)
4	Reserved (set to 0)
5	Reserved (set to 0)
6	Reserved (set to 0)
7	Reserved (set to 0)*/
MAV_CMD_LOGGING_STOP = 2511, 
/**

1	Landing gear ID (default: 0, -1 for all)
2	Landing gear position (Down: 0, Up: 1, NaN for no change)
3	Reserved, set to NaN
4	Reserved, set to NaN
5	Reserved, set to NaN
6	Reserved, set to NaN
7	Reserved, set to NaN*/
MAV_CMD_AIRFRAME_CONFIGURATION = 2520, 
/**
Request to start/stop transmitting over the high latency telemetry
1	Control transmission over high latency telemetry (0: stop, 1: start)
2	Empty
3	Empty
4	Empty
5	Empty
6	Empty
7	Empty*/
MAV_CMD_CONTROL_HIGH_LATENCY = 2600, 
/**
Create a panorama at the current position
1	Viewing angle horizontal of the panorama (+- 0.5 the total angle)
2	Viewing angle vertical of panorama.
3	Speed of the horizontal rotation.
4	Speed of the vertical rotation.*/
MAV_CMD_PANORAMA_CREATE = 2800, 
/**
Request VTOL transition
1	The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.*/
MAV_CMD_DO_VTOL_TRANSITION = 3000, 
/**
Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.

1	Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle*/
MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001, 
/**
This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
*/
MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000, 
/**
This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.

1	Radius of desired circle in CIRCLE_MODE
2	User defined
3	User defined
4	User defined
5	Unscaled target latitude of center of circle in CIRCLE_MODE
6	Unscaled target longitude of center of circle in CIRCLE_MODE*/
MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001, 
/**
Delay mission state machine until gate has been reached.
1	Geometry: 0: orthogonal to path between previous and next waypoint.
2	Altitude: 0: ignore altitude
3	Empty
4	Empty
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_CONDITION_GATE = 4501, 
/**
Fence return point. There can only be one fence return point.

1	Reserved
2	Reserved
3	Reserved
4	Reserved
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_FENCE_RETURN_POINT = 5000, 
/**
Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.

1	Polygon vertex count
2	Reserved
3	Reserved
4	Reserved
5	Latitude
6	Longitude
7	Reserved*/
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001, 
/**
Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.

1	Polygon vertex count
2	Reserved
3	Reserved
4	Reserved
5	Latitude
6	Longitude
7	Reserved*/
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002, 
/**
Circular fence area. The vehicle must stay inside this area.

1	Radius.
2	Reserved
3	Reserved
4	Reserved
5	Latitude
6	Longitude
7	Reserved*/
MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003, 
/**
Circular fence area. The vehicle must stay outside this area.

1	Radius.
2	Reserved
3	Reserved
4	Reserved
5	Latitude
6	Longitude
7	Reserved*/
MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004, 
/**
Rally point. You can have multiple rally points defined.

1	Reserved
2	Reserved
3	Reserved
4	Reserved
5	Latitude
6	Longitude
7	Altitude*/
MAV_CMD_NAV_RALLY_POINT = 5100, 
/**
Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
node that is online. Note that some of the response messages can be lost, which the receiver can detect
easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
received earlier; if not, this command should be sent again in order to request re-transmission of the
node information messages
1	Reserved (set to 0)
2	Reserved (set to 0)
3	Reserved (set to 0)
4	Reserved (set to 0)
5	Reserved (set to 0)
6	Reserved (set to 0)
7	Reserved (set to 0)*/
MAV_CMD_UAVCAN_GET_NODE_INFO = 5200, 
/**
Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
position and velocity
1	Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
2	Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.
3	Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
4	Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.
5	Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
6	Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
7	Altitude (MSL), in meters*/
MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001, 
/**
Control the payload deployment.
1	Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
2	Reserved
3	Reserved
4	Reserved
5	Reserved
6	Reserved
7	Reserved*/
MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002, 
/**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_WAYPOINT_USER_1 = 31000, 
/**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_WAYPOINT_USER_2 = 31001, 
/**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_WAYPOINT_USER_3 = 31002, 
/**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_WAYPOINT_USER_4 = 31003, 
/**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_WAYPOINT_USER_5 = 31004, 
/**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_SPATIAL_USER_1 = 31005, 
/**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_SPATIAL_USER_2 = 31006, 
/**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_SPATIAL_USER_3 = 31007, 
/**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_SPATIAL_USER_4 = 31008, 
/**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item
1	User defined
2	User defined
3	User defined
4	User defined
5	Latitude unscaled
6	Longitude unscaled
7	Altitude (MSL), in meters*/
MAV_CMD_SPATIAL_USER_5 = 31009, 
/**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item
1	User defined
2	User defined
3	User defined
4	User defined
5	User defined
6	User defined
7	User defined*/
MAV_CMD_USER_1 = 31010, 
/**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item
1	User defined
2	User defined
3	User defined
4	User defined
5	User defined
6	User defined
7	User defined*/
MAV_CMD_USER_2 = 31011, 
/**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item
1	User defined
2	User defined
3	User defined
4	User defined
5	User defined
6	User defined
7	User defined*/
MAV_CMD_USER_3 = 31012, 
/**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item
1	User defined
2	User defined
3	User defined
4	User defined
5	User defined
6	User defined
7	User defined*/
MAV_CMD_USER_4 = 31013, 
/**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item
1	User defined
2	User defined
3	User defined
4	User defined
5	User defined
6	User defined
7	User defined*/
MAV_CMD_USER_5 = 31014;
}

/**
Micro air vehicle / autopilot classes. This identifies the individual model.*/
enum MAV_AUTOPILOT
{ 
;

 final int
MAV_AUTOPILOT_GENERIC = 0, //Generic autopilot, full support for everything
MAV_AUTOPILOT_RESERVED = 1, //Reserved for future use.
MAV_AUTOPILOT_SLUGS = 2, //SLUGS autopilot, http://slugsuav.soe.ucsc.edu
MAV_AUTOPILOT_ARDUPILOTMEGA = 3, //ArduPilot - Plane/Copter/Rover/Sub/Tracker, http://ardupilot.org
MAV_AUTOPILOT_OPENPILOT = 4, //OpenPilot, http://openpilot.org
MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5, //Generic autopilot only supporting simple waypoints
MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6, //Generic autopilot supporting waypoints and other simple navigation commands
MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7, //Generic autopilot supporting the full mission command set
MAV_AUTOPILOT_INVALID = 8, //No valid autopilot, e.g. a GCS or other MAVLink component
MAV_AUTOPILOT_PPZ = 9, //PPZ UAV - http://nongnu.org/paparazzi
MAV_AUTOPILOT_UDB = 10, //UAV Dev Board
MAV_AUTOPILOT_FP = 11, //FlexiPilot
MAV_AUTOPILOT_PX4 = 12, //PX4 Autopilot - http://px4.io/
MAV_AUTOPILOT_SMACCMPILOT = 13, //SMACCMPilot - http://smaccmpilot.org
MAV_AUTOPILOT_AUTOQUAD = 14, //AutoQuad -- http://autoquad.org
MAV_AUTOPILOT_ARMAZILA = 15, //Armazila -- http://armazila.com
MAV_AUTOPILOT_AEROB = 16, //Aerob -- http://aerob.ru
MAV_AUTOPILOT_ASLUAV = 17, //ASLUAV autopilot -- http://www.asl.ethz.ch
MAV_AUTOPILOT_SMARTAP = 18, //SmartAP Autopilot - http://sky-drones.com
MAV_AUTOPILOT_AIRRAILS = 19;  //AirRails - http://uaventure.com
}

/**
MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle
on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate
for their type (e.g. a camera must use MAV_TYPE_CAMERA)*/
enum MAV_TYPE
{ 
;

 final int
MAV_TYPE_GENERIC = 0, //Generic micro air vehicle
MAV_TYPE_FIXED_WING = 1, //Fixed wing aircraft.
MAV_TYPE_QUADROTOR = 2, //Quadrotor
MAV_TYPE_COAXIAL = 3, //Coaxial helicopter
MAV_TYPE_HELICOPTER = 4, //Normal helicopter with tail rotor.
MAV_TYPE_ANTENNA_TRACKER = 5, //Ground installation
MAV_TYPE_GCS = 6, //Operator control unit / ground control station
MAV_TYPE_AIRSHIP = 7, //Airship, controlled
MAV_TYPE_FREE_BALLOON = 8, //Free balloon, uncontrolled
MAV_TYPE_ROCKET = 9, //Rocket
MAV_TYPE_GROUND_ROVER = 10, //Ground rover
MAV_TYPE_SURFACE_BOAT = 11, //Surface vessel, boat, ship
MAV_TYPE_SUBMARINE = 12, //Submarine
MAV_TYPE_HEXAROTOR = 13, //Hexarotor
MAV_TYPE_OCTOROTOR = 14, //Octorotor
MAV_TYPE_TRICOPTER = 15, //Tricopter
MAV_TYPE_FLAPPING_WING = 16, //Flapping wing
MAV_TYPE_KITE = 17, //Kite
MAV_TYPE_ONBOARD_CONTROLLER = 18, //Onboard companion controller
MAV_TYPE_VTOL_DUOROTOR = 19, //Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
MAV_TYPE_VTOL_QUADROTOR = 20, //Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
MAV_TYPE_VTOL_TILTROTOR = 21, //Tiltrotor VTOL
MAV_TYPE_VTOL_RESERVED2 = 22, //VTOL reserved 2
MAV_TYPE_VTOL_RESERVED3 = 23, //VTOL reserved 3
MAV_TYPE_VTOL_RESERVED4 = 24, //VTOL reserved 4
MAV_TYPE_VTOL_RESERVED5 = 25, //VTOL reserved 5
MAV_TYPE_GIMBAL = 26, //Gimbal
MAV_TYPE_ADSB = 27, //ADSB system
MAV_TYPE_PARAFOIL = 28, //Steerable, nonrigid airfoil
MAV_TYPE_DODECAROTOR = 29, //Dodecarotor
MAV_TYPE_CAMERA = 30, //Camera
MAV_TYPE_CHARGING_STATION = 31, //Charging station
MAV_TYPE_FLARM = 32, //FLARM collision avoidance system
MAV_TYPE_SERVO = 33;  //Servo
}

/**
These values define the type of firmware release.  These values indicate the first version or release
of this type.  For example the first alpha release would be 64, the second would be 65*/
enum FIRMWARE_VERSION_TYPE
{ 
;

 final int
FIRMWARE_VERSION_TYPE_DEV = 0, //development release
FIRMWARE_VERSION_TYPE_ALPHA = 64, //alpha release
FIRMWARE_VERSION_TYPE_BETA = 128, //beta release
FIRMWARE_VERSION_TYPE_RC = 192, //release candidate
FIRMWARE_VERSION_TYPE_OFFICIAL = 255;  //official stable release
}

/**
Flags to report failure cases over the high latency telemtry.*/
 @Flags enum HL_FAILURE_FLAG
{ 
;

 final int
HL_FAILURE_FLAG_GPS = 1, //GPS failure.
HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE = 2, //Differential pressure sensor failure.
HL_FAILURE_FLAG_ABSOLUTE_PRESSURE = 4, //Absolute pressure sensor failure.
HL_FAILURE_FLAG_3D_ACCEL = 8, //Accelerometer sensor failure.
HL_FAILURE_FLAG_3D_GYRO = 16, //Gyroscope sensor failure.
HL_FAILURE_FLAG_3D_MAG = 32, //Magnetometer sensor failure.
HL_FAILURE_FLAG_TERRAIN = 64, //Terrain subsystem failure.
HL_FAILURE_FLAG_BATTERY = 128, //Battery failure/critical low battery.
HL_FAILURE_FLAG_RC_RECEIVER = 256, //RC receiver failure/no rc connection.
HL_FAILURE_FLAG_OFFBOARD_LINK = 512, //Offboard link failure.
HL_FAILURE_FLAG_ENGINE = 1024, //Engine failure.
HL_FAILURE_FLAG_GEOFENCE = 2048, //Geofence violation.
HL_FAILURE_FLAG_ESTIMATOR = 4096, //Estimator failure, for example measurement rejection or large variances.
HL_FAILURE_FLAG_MISSION = 8192;  //Mission failure.
}

/**
These flags encode the MAV mode.*/
 @Flags enum MAV_MODE_FLAG
{ 
;

 final int
/**
0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
shall be used instead. The flag can still be used to report the armed state*/
MAV_MODE_FLAG_SAFETY_ARMED = 128, 
MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, //0b01000000 remote control input is enabled.
/**
0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
is full operational*/
MAV_MODE_FLAG_HIL_ENABLED = 32, 
/**
0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
control inputs to move around*/
MAV_MODE_FLAG_STABILIZE_ENABLED = 16, 
MAV_MODE_FLAG_GUIDED_ENABLED = 8, //0b00001000 guided mode enabled, system flies waypoints / mission items.
/**
0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
depends on the actual implementation*/
MAV_MODE_FLAG_AUTO_ENABLED = 4, 
/**
0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
not be used for stable implementations*/
MAV_MODE_FLAG_TEST_ENABLED = 2, 
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;  //0b00000001 Reserved for future use.
}

/**
These values encode the bit positions of the decode position. These values can be used to read the value
of a flag bit by combining the base_mode variable with AND with the flag position value. The result will
be either 0 or 1, depending on if the flag is set or not*/
 @Flags enum MAV_MODE_FLAG_DECODE_POSITION
{ 
;

 final int
MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 128, //First bit:  10000000
MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64, //Second bit: 01000000
MAV_MODE_FLAG_DECODE_POSITION_HIL = 32, //Third bit:  00100000
MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16, //Fourth bit: 00010000
MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8, //Fifth bit:  00001000
MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4, //Sixth bit:   00000100
MAV_MODE_FLAG_DECODE_POSITION_TEST = 2, //Seventh bit: 00000010
MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1;  //Eighth bit: 00000001
}

/**
Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.*/
enum MAV_GOTO
{ 
;

 final int
MAV_GOTO_DO_HOLD = 0, //Hold at the current position.
MAV_GOTO_DO_CONTINUE = 1, //Continue with the next item in mission execution.
MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2, //Hold at the current position of the system
MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3;  //Hold at the position specified in the parameters of the DO_HOLD action
}

/**
These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.*/
enum MAV_MODE
{ 
;

 final int
MAV_MODE_PREFLIGHT = 0, //System is not ready to fly, booting, calibrating, etc. No flag is set.
MAV_MODE_STABILIZE_DISARMED = 80, //System is allowed to be active, under assisted RC control.
MAV_MODE_STABILIZE_ARMED = 208, //System is allowed to be active, under assisted RC control.
MAV_MODE_MANUAL_DISARMED = 64, //System is allowed to be active, under manual (RC) control, no stabilization
MAV_MODE_MANUAL_ARMED = 192, //System is allowed to be active, under manual (RC) control, no stabilization
MAV_MODE_GUIDED_DISARMED = 88, //System is allowed to be active, under autonomous control, manual setpoint
MAV_MODE_GUIDED_ARMED = 216, //System is allowed to be active, under autonomous control, manual setpoint
/**
System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
and not pre-programmed by waypoints*/
MAV_MODE_AUTO_DISARMED = 92, 
/**
System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
and not pre-programmed by waypoints*/
MAV_MODE_AUTO_ARMED = 220, 
MAV_MODE_TEST_DISARMED = 66, //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
MAV_MODE_TEST_ARMED = 194;  //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
}
enum MAV_STATE
{ 
MAV_STATE_BOOT, //System is booting up.
MAV_STATE_CALIBRATING, //System is calibrating and not flight-ready.
MAV_STATE_STANDBY, //System is grounded and on standby. It can be launched any time.
MAV_STATE_ACTIVE, //System is active and might be already airborne. Motors are engaged.
MAV_STATE_CRITICAL, //System is in a non-normal flight mode. It can however still navigate.
/**
System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
mayday and going down*/
MAV_STATE_EMERGENCY, 
MAV_STATE_POWEROFF, //System just initialized its power-down sequence, will shut down now.
MAV_STATE_FLIGHT_TERMINATION;  //System is terminating itself.
 final int
MAV_STATE_UNINIT = 0;  //Uninitialized system, state is unknown.
}

/**
When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should
be allocated sequential values. An appropriate number of values should be left free after these components
to allow the number of instances to be expanded*/
enum MAV_COMPONENT
{ 
;

 final int
/**
Target id (target_component) used to broadcast messages to all components of the receiving system. Components
should attempt to process messages with this component ID and forward to components on any other interfaces.
Note: This is not a valid *source* component id for a message*/
MAV_COMP_ID_ALL = 0, 
MAV_COMP_ID_AUTOPILOT1 = 1, //System flight controller component ("autopilot"). Only one autopilot is expected in a particular system
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER1 = 25, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER2 = 26, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER3 = 27, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER4 = 28, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER5 = 29, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER6 = 30, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER7 = 31, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER8 = 32, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER9 = 33, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER10 = 34, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER11 = 35, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER12 = 36, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER13 = 37, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER14 = 38, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER15 = 39, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USE16 = 40, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER17 = 41, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER18 = 42, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER19 = 43, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER20 = 44, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER21 = 45, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER22 = 46, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER23 = 47, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER24 = 48, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER25 = 49, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER26 = 50, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER27 = 51, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER28 = 52, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER29 = 53, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER30 = 54, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER31 = 55, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER32 = 56, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER33 = 57, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER34 = 58, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER35 = 59, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER36 = 60, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER37 = 61, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER38 = 62, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER39 = 63, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER40 = 64, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER41 = 65, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER42 = 66, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER43 = 67, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER44 = 68, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER45 = 69, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER46 = 70, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER47 = 71, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER48 = 72, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER49 = 73, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER50 = 74, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER51 = 75, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER52 = 76, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER53 = 77, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER54 = 78, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER55 = 79, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER56 = 80, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER57 = 81, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER58 = 82, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER59 = 83, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER60 = 84, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER61 = 85, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER62 = 86, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER63 = 87, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER64 = 88, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER65 = 89, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER66 = 90, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER67 = 91, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER68 = 92, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER69 = 93, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER70 = 94, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER71 = 95, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER72 = 96, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER73 = 97, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER74 = 98, 
/**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network*/
MAV_COMP_ID_USER75 = 99, 
MAV_COMP_ID_CAMERA = 100, //Camera #1.
MAV_COMP_ID_CAMERA2 = 101, //Camera #2.
MAV_COMP_ID_CAMERA3 = 102, //Camera #3.
MAV_COMP_ID_CAMERA4 = 103, //Camera #4.
MAV_COMP_ID_CAMERA5 = 104, //Camera #5.
MAV_COMP_ID_CAMERA6 = 105, //Camera #6.
MAV_COMP_ID_SERVO1 = 140, //Servo #1.
MAV_COMP_ID_SERVO2 = 141, //Servo #2.
MAV_COMP_ID_SERVO3 = 142, //Servo #3.
MAV_COMP_ID_SERVO4 = 143, //Servo #4.
MAV_COMP_ID_SERVO5 = 144, //Servo #5.
MAV_COMP_ID_SERVO6 = 145, //Servo #6.
MAV_COMP_ID_SERVO7 = 146, //Servo #7.
MAV_COMP_ID_SERVO8 = 147, //Servo #8.
MAV_COMP_ID_SERVO9 = 148, //Servo #9.
MAV_COMP_ID_SERVO10 = 149, //Servo #10.
MAV_COMP_ID_SERVO11 = 150, //Servo #11.
MAV_COMP_ID_SERVO12 = 151, //Servo #12.
MAV_COMP_ID_SERVO13 = 152, //Servo #13.
MAV_COMP_ID_SERVO14 = 153, //Servo #14.
MAV_COMP_ID_GIMBAL = 154, //Gimbal #1.
MAV_COMP_ID_LOG = 155, //Logging component.
MAV_COMP_ID_ADSB = 156, //Automatic Dependent Surveillance-Broadcast (ADS-B) component.
MAV_COMP_ID_OSD = 157, //On Screen Display (OSD) devices for video links.
MAV_COMP_ID_PERIPHERAL = 158, //Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice
MAV_COMP_ID_QX1_GIMBAL = 159, //Gimbal ID for QX1.
MAV_COMP_ID_FLARM = 160, //FLARM collision alert component.
MAV_COMP_ID_GIMBAL2 = 171, //Gimbal #2.
MAV_COMP_ID_GIMBAL3 = 172, //Gimbal #3.
MAV_COMP_ID_GIMBAL4 = 173, //Gimbal #4
MAV_COMP_ID_GIMBAL5 = 174, //Gimbal #5.
MAV_COMP_ID_GIMBAL6 = 175, //Gimbal #6.
MAV_COMP_ID_MISSIONPLANNER = 190, //Component that can generate/supply a mission flight plan (e.g. GCS or developer API).
/**
Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap,
shortest path, cost, etc.)*/
MAV_COMP_ID_PATHPLANNER = 195, 
MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196, //Component that plans a collision free path between two points.
MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197, //Component that provides position estimates using VIO techniques.
MAV_COMP_ID_PAIRING_MANAGER = 198, //Component that manages pairing of vehicle and GCS.
MAV_COMP_ID_IMU = 200, //Inertial Measurement Unit (IMU) #1.
MAV_COMP_ID_IMU_2 = 201, //Inertial Measurement Unit (IMU) #2.
MAV_COMP_ID_IMU_3 = 202, //Inertial Measurement Unit (IMU) #3.
MAV_COMP_ID_GPS = 220, //GPS #1.
MAV_COMP_ID_GPS2 = 221, //GPS #2.
MAV_COMP_ID_UDP_BRIDGE = 240, //Component to bridge MAVLink to UDP (i.e. from a UART).
MAV_COMP_ID_UART_BRIDGE = 241, //Component to bridge to UART (i.e. from UDP).
MAV_COMP_ID_TUNNEL_NODE = 242, //Component handling TUNNEL messages (e.g. vendor specific GUI of a component).
MAV_COMP_ID_SYSTEM_CONTROL = 250;  //Component for handling system messages (e.g. to ARM, takeoff, etc.).
}

/**
These encode the sensors whose status is sent as part of the SYS_STATUS message.*/
 @Flags enum MAV_SYS_STATUS_SENSOR
{ 
;

 final int
MAV_SYS_STATUS_SENSOR_3D_GYRO = 1, //0x01 3D gyro
MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2, //0x02 3D accelerometer
MAV_SYS_STATUS_SENSOR_3D_MAG = 4, //0x04 3D magnetometer
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8, //0x08 absolute pressure
MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16, //0x10 differential pressure
MAV_SYS_STATUS_SENSOR_GPS = 32, //0x20 GPS
MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64, //0x40 optical flow
MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128, //0x80 computer vision position
MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256, //0x100 laser based position
MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512, //0x200 external ground truth (Vicon or Leica)
MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024, //0x400 3D angular rate control
MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048, //0x800 attitude stabilization
MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096, //0x1000 yaw position
MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192, //0x2000 z/altitude control
MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384, //0x4000 x/y position control
MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768, //0x8000 motor outputs / control
MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536, //0x10000 rc receiver
MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072, //0x20000 2nd 3D gyro
MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144, //0x40000 2nd 3D accelerometer
MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288, //0x80000 2nd 3D magnetometer
MAV_SYS_STATUS_GEOFENCE = 1048576, //0x100000 geofence
MAV_SYS_STATUS_AHRS = 2097152, //0x200000 AHRS subsystem health
MAV_SYS_STATUS_TERRAIN = 4194304, //0x400000 Terrain subsystem health
MAV_SYS_STATUS_REVERSE_MOTOR = 8388608, //0x800000 Motors are reversed
MAV_SYS_STATUS_LOGGING = 16777216, //0x1000000 Logging
MAV_SYS_STATUS_SENSOR_BATTERY = 33554432, //0x2000000 Battery
MAV_SYS_STATUS_SENSOR_PROXIMITY = 67108864, //0x4000000 Proximity
MAV_SYS_STATUS_SENSOR_SATCOM = 134217728;  //0x8000000 Satellite Communication 
}
enum MAV_FRAME
{ 
;

 final int
/**
Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude,
third value / z: positive altitude over mean sea level (MSL)*/
MAV_FRAME_GLOBAL = 0, 
MAV_FRAME_LOCAL_NED = 1, //Local coordinate frame, Z-down (x: north, y: east, z: down).
MAV_FRAME_MISSION = 2, //NOT a coordinate frame, indicates a mission command.
/**
Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second
value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location*/
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, 
MAV_FRAME_LOCAL_ENU = 4, //Local coordinate frame, Z-up (x: east, y: north, z: up).
/**
Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7,
second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level
(MSL)*/
MAV_FRAME_GLOBAL_INT = 5, 
/**
Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude
in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with
0 being at the altitude of the home location*/
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, 
/**
Offset to the current local frame. Anything expressed in this frame should be added to the current local
frame position*/
MAV_FRAME_LOCAL_OFFSET_NED = 7, 
/**
Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to
command 2 m/s^2 acceleration to the right*/
MAV_FRAME_BODY_NED = 8, 
/**
Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an
obstacle - e.g. useful to command 2 m/s^2 acceleration to the east*/
MAV_FRAME_BODY_OFFSET_NED = 9, 
/**
Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude
in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with
0 being at ground level in terrain model*/
MAV_FRAME_GLOBAL_TERRAIN_ALT = 10, 
/**
Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value /
x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive
altitude in meters with 0 being at ground level in terrain model*/
MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11, 
MAV_FRAME_BODY_FRD = 12, //Body fixed frame of reference, Z-down (x: forward, y: right, z: down).
MAV_FRAME_BODY_FLU = 13, //Body fixed frame of reference, Z-up (x: forward, y: left, z: up).
/**
Odometry local coordinate frame of data given by a motion capture system, Z-down (x: north, y: east, z:
down)*/
MAV_FRAME_MOCAP_NED = 14, 
/**
Odometry local coordinate frame of data given by a motion capture system, Z-up (x: east, y: north, z:
up)*/
MAV_FRAME_MOCAP_ENU = 15, 
/**
Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east,
z: down)*/
MAV_FRAME_VISION_NED = 16, 
/**
Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: east, y: north,
z: up)*/
MAV_FRAME_VISION_ENU = 17, 
/**
Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x:
north, y: east, z: down)*/
MAV_FRAME_ESTIM_NED = 18, 
/**
Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: east,
y: noth, z: up)*/
MAV_FRAME_ESTIM_ENU = 19, 
/**
Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e.
not aligned with NED/earth frame)*/
MAV_FRAME_LOCAL_FRD = 20, 
/**
Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e.
not aligned with ENU/earth frame)*/
MAV_FRAME_LOCAL_FLU = 21;  
}
enum MAVLINK_DATA_STREAM_TYPE
{ 
MAVLINK_DATA_STREAM_IMG_JPEG, 
MAVLINK_DATA_STREAM_IMG_BMP, 
MAVLINK_DATA_STREAM_IMG_RAW8U, 
MAVLINK_DATA_STREAM_IMG_RAW32U, 
MAVLINK_DATA_STREAM_IMG_PGM, 
MAVLINK_DATA_STREAM_IMG_PNG;  
}
enum FENCE_ACTION
{ 
;

 final int
FENCE_ACTION_NONE = 0, //Disable fenced mode
FENCE_ACTION_GUIDED = 1, //Switched to guided mode to return point (fence point 0)
FENCE_ACTION_REPORT = 2, //Report fence breach, but don't take action
FENCE_ACTION_GUIDED_THR_PASS = 3, //Switched to guided mode to return point (fence point 0) with manual throttle control
FENCE_ACTION_RTL = 4;  //Switch to RTL (return to launch) mode and head for the return point.
}
enum FENCE_BREACH
{ 
;

 final int
FENCE_BREACH_NONE = 0, //No last fence breach
FENCE_BREACH_MINALT = 1, //Breached minimum altitude
FENCE_BREACH_MAXALT = 2, //Breached maximum altitude
FENCE_BREACH_BOUNDARY = 3;  //Breached fence boundary
}

/**
Actions being taken to mitigate/prevent fence breach*/
 @Flags enum FENCE_MITIGATE
{ 
;

 final int
FENCE_MITIGATE_UNKNOWN = 0, //Unknown
FENCE_MITIGATE_NONE = 1, //No actions being taken
FENCE_MITIGATE_VEL_LIMIT = 2;  //Velocity limiting active to prevent breach
}

/**
Enumeration of possible mount operation modes*/
enum MAV_MOUNT_MODE
{ 
;

 final int
MAV_MOUNT_MODE_RETRACT = 0, //Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
MAV_MOUNT_MODE_NEUTRAL = 1, //Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
MAV_MOUNT_MODE_MAVLINK_TARGETING = 2, //Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
MAV_MOUNT_MODE_RC_TARGETING = 3, //Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
MAV_MOUNT_MODE_GPS_POINT = 4, //Load neutral position and start to point to Lat,Lon,Alt
MAV_MOUNT_MODE_SYSID_TARGET = 5;  //Gimbal tracks system with specified system ID
}

/**
Generalized UAVCAN node health*/
enum UAVCAN_NODE_HEALTH
{ 
;

 final int
UAVCAN_NODE_HEALTH_OK = 0, //The node is functioning properly.
UAVCAN_NODE_HEALTH_WARNING = 1, //A critical parameter went out of range or the node has encountered a minor failure.
UAVCAN_NODE_HEALTH_ERROR = 2, //The node has encountered a major failure.
UAVCAN_NODE_HEALTH_CRITICAL = 3;  //The node has suffered a fatal malfunction.
}

/**
Generalized UAVCAN node mode*/
enum UAVCAN_NODE_MODE
{ 
;

 final int
UAVCAN_NODE_MODE_OPERATIONAL = 0, //The node is performing its primary functions.
UAVCAN_NODE_MODE_INITIALIZATION = 1, //The node is initializing; this mode is entered immediately after startup.
UAVCAN_NODE_MODE_MAINTENANCE = 2, //The node is under maintenance.
UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3, //The node is in the process of updating its software.
UAVCAN_NODE_MODE_OFFLINE = 7;  //The node is no longer available online.
}

/**
Flags to indicate the status of camera storage.*/
enum STORAGE_STATUS
{ 
;

 final int
STORAGE_STATUS_EMPTY = 0, //Storage is missing (no microSD card loaded for example.)
STORAGE_STATUS_UNFORMATTED = 1, //Storage present but unformatted.
STORAGE_STATUS_READY = 2, //Storage present and ready.
/**
Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields
will be ignored*/
STORAGE_STATUS_NOT_SUPPORTED = 3;  
}

/**
Yaw behaviour during orbit flight.*/
enum ORBIT_YAW_BEHAVIOUR
{ 
;

 final int
ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0, //Vehicle front points to the center (default).
ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1, //Vehicle front holds heading when message received.
ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2, //Yaw uncontrolled.
ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3, //Vehicle front follows flight path (tangential to circle).
ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4;  //Yaw controlled by RC input.
}

/**
the recommended messages.*/
enum MAV_DATA_STREAM
{ 
;

 final int
MAV_DATA_STREAM_ALL = 0, //Enable all data streams
MAV_DATA_STREAM_RAW_SENSORS = 1, //Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
MAV_DATA_STREAM_EXTENDED_STATUS = 2, //Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
MAV_DATA_STREAM_RC_CHANNELS = 3, //Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
MAV_DATA_STREAM_RAW_CONTROLLER = 4, //Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
MAV_DATA_STREAM_POSITION = 6, //Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
MAV_DATA_STREAM_EXTRA1 = 10, //Dependent on the autopilot
MAV_DATA_STREAM_EXTRA2 = 11, //Dependent on the autopilot
MAV_DATA_STREAM_EXTRA3 = 12;  //Dependent on the autopilot
}

/**
MAV_CMD_NAV_ROI).*/
enum MAV_ROI
{ 
;

 final int
MAV_ROI_NONE = 0, //No region of interest.
MAV_ROI_WPNEXT = 1, //Point toward next waypoint, with optional pitch/roll/yaw offset.
MAV_ROI_WPINDEX = 2, //Point toward given waypoint.
MAV_ROI_LOCATION = 3, //Point toward fixed location.
MAV_ROI_TARGET = 4;  //Point toward of given id.
}

/**
ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.*/
enum MAV_CMD_ACK
{ 
MAV_CMD_ACK_OK, //Command / mission item is ok.
MAV_CMD_ACK_ERR_FAIL, //Generic error message if none of the other reasons fails or if no detailed error reporting is implemented
MAV_CMD_ACK_ERR_ACCESS_DENIED, //The system is refusing to accept this command from this source / communication partner.
MAV_CMD_ACK_ERR_NOT_SUPPORTED, //Command or mission item is not supported, other commands would be accepted.
MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED, //The coordinate frame of this command / mission item is not supported.
/**
The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this
system. This is a generic error, please use the more specific error messages below if possible*/
MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE, 
MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE, //The X or latitude value is out of range.
MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE, //The Y or longitude value is out of range.
MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE;  //The Z or altitude value is out of range.
}

/**
Specifies the datatype of a MAVLink parameter.*/
enum MAV_PARAM_TYPE
{ 
;

 final int
MAV_PARAM_TYPE_UINT8 = 1, //8-bit unsigned integer
MAV_PARAM_TYPE_INT8 = 2, //8-bit signed integer
MAV_PARAM_TYPE_UINT16 = 3, //16-bit unsigned integer
MAV_PARAM_TYPE_INT16 = 4, //16-bit signed integer
MAV_PARAM_TYPE_UINT32 = 5, //32-bit unsigned integer
MAV_PARAM_TYPE_INT32 = 6, //32-bit signed integer
MAV_PARAM_TYPE_UINT64 = 7, //64-bit unsigned integer
MAV_PARAM_TYPE_INT64 = 8, //64-bit signed integer
MAV_PARAM_TYPE_REAL32 = 9, //32-bit floating-point
MAV_PARAM_TYPE_REAL64 = 10;  //64-bit floating-point
}

/**
Specifies the datatype of a MAVLink extended parameter.*/
enum MAV_PARAM_EXT_TYPE
{ 
;

 final int
MAV_PARAM_EXT_TYPE_UINT8 = 1, //8-bit unsigned integer
MAV_PARAM_EXT_TYPE_INT8 = 2, //8-bit signed integer
MAV_PARAM_EXT_TYPE_UINT16 = 3, //16-bit unsigned integer
MAV_PARAM_EXT_TYPE_INT16 = 4, //16-bit signed integer
MAV_PARAM_EXT_TYPE_UINT32 = 5, //32-bit unsigned integer
MAV_PARAM_EXT_TYPE_INT32 = 6, //32-bit signed integer
MAV_PARAM_EXT_TYPE_UINT64 = 7, //64-bit unsigned integer
MAV_PARAM_EXT_TYPE_INT64 = 8, //64-bit signed integer
MAV_PARAM_EXT_TYPE_REAL32 = 9, //32-bit floating-point
MAV_PARAM_EXT_TYPE_REAL64 = 10, //64-bit floating-point
MAV_PARAM_EXT_TYPE_CUSTOM = 11;  //Custom Type
}

/**
Result from a MAVLink command (MAV_CMD)*/
enum MAV_RESULT
{ 
;

 final int
MAV_RESULT_ACCEPTED = 0, //Command is valid (is supported and has valid parameters), and was executed.
/**
Command is valid, but cannot be executed at this time. This is used to indicate a problem that should
be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.).
Retrying later should work*/
MAV_RESULT_TEMPORARILY_REJECTED = 1, 
/**
Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will
not work*/
MAV_RESULT_DENIED = 2, 
MAV_RESULT_UNSUPPORTED = 3, //Command is not supported (unknown).
/**
Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem,
i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting
to write a file when out of memory, attempting to arm when sensors are not calibrated, etc*/
MAV_RESULT_FAILED = 4, 
/**
Command is valid and is being executed. This will be followed by further progress updates, i.e. the component
may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation),
and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress
field can be used to indicate the progress of the operation. There is no need for the sender to retry
the command, but if done during execution, the component will return MAV_RESULT_IN_PROGRESS with an updated
progress*/
MAV_RESULT_IN_PROGRESS = 5;  
}

/**
Result of mission operation (in a MISSION_ACK message).*/
enum MAV_MISSION_RESULT
{ 
;

 final int
MAV_MISSION_ACCEPTED = 0, //mission accepted OK
MAV_MISSION_ERROR = 1, //Generic error / not accepting mission commands at all right now.
MAV_MISSION_UNSUPPORTED_FRAME = 2, //Coordinate frame is not supported.
MAV_MISSION_UNSUPPORTED = 3, //Command is not supported.
MAV_MISSION_NO_SPACE = 4, //Mission item exceeds storage space.
MAV_MISSION_INVALID = 5, //One of the parameters has an invalid value.
MAV_MISSION_INVALID_PARAM1 = 6, //param1 has an invalid value.
MAV_MISSION_INVALID_PARAM2 = 7, //param2 has an invalid value.
MAV_MISSION_INVALID_PARAM3 = 8, //param3 has an invalid value.
MAV_MISSION_INVALID_PARAM4 = 9, //param4 has an invalid value.
MAV_MISSION_INVALID_PARAM5_X = 10, //x / param5 has an invalid value.
MAV_MISSION_INVALID_PARAM6_Y = 11, //y / param6 has an invalid value.
MAV_MISSION_INVALID_PARAM7 = 12, //z / param7 has an invalid value.
MAV_MISSION_INVALID_SEQUENCE = 13, //Mission item received out of sequence
MAV_MISSION_DENIED = 14, //Not accepting any mission commands from this communication partner.
MAV_MISSION_OPERATION_CANCELLED = 15;  //Current mission operation cancelled (e.g. mission upload, mission download).
}

/**
Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/*/
enum MAV_SEVERITY
{ 
;

 final int
MAV_SEVERITY_EMERGENCY = 0, //System is unusable. This is a "panic" condition.
MAV_SEVERITY_ALERT = 1, //Action should be taken immediately. Indicates error in non-critical systems.
MAV_SEVERITY_CRITICAL = 2, //Action must be taken immediately. Indicates failure in a primary system.
MAV_SEVERITY_ERROR = 3, //Indicates an error in secondary/redundant systems.
/**
Indicates about a possible future error if this is not resolved within a given timeframe. Example would
be a low battery warning*/
MAV_SEVERITY_WARNING = 4, 
/**
An unusual event has occurred, though not an error condition. This should be investigated for the root
cause*/
MAV_SEVERITY_NOTICE = 5, 
MAV_SEVERITY_INFO = 6, //Normal operational messages. Useful for logging. No action is required for these messages.
MAV_SEVERITY_DEBUG = 7;  //Useful non-operational messages that can assist in debugging. These should not occur during normal operation
}

/**
Power supply status flags (bitmask)*/
 @Flags enum MAV_POWER_STATUS
{ 
;

 final int
MAV_POWER_STATUS_BRICK_VALID = 1, //main brick power supply valid
MAV_POWER_STATUS_SERVO_VALID = 2, //main servo power supply valid for FMU
MAV_POWER_STATUS_USB_CONNECTED = 4, //USB power is connected
MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8, //peripheral supply is in over-current state
MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16, //hi-power peripheral supply is in over-current state
MAV_POWER_STATUS_CHANGED = 32;  //Power status has changed since boot
}

/**
SERIAL_CONTROL device types*/
enum SERIAL_CONTROL_DEV
{ 
;

 final int
SERIAL_CONTROL_DEV_TELEM1 = 0, //First telemetry port
SERIAL_CONTROL_DEV_TELEM2 = 1, //Second telemetry port
SERIAL_CONTROL_DEV_GPS1 = 2, //First GPS port
SERIAL_CONTROL_DEV_GPS2 = 3, //Second GPS port
SERIAL_CONTROL_DEV_SHELL = 10, //system shell
SERIAL_CONTROL_SERIAL0 = 100, //SERIAL0
SERIAL_CONTROL_SERIAL1 = 101, //SERIAL1
SERIAL_CONTROL_SERIAL2 = 102, //SERIAL2
SERIAL_CONTROL_SERIAL3 = 103, //SERIAL3
SERIAL_CONTROL_SERIAL4 = 104, //SERIAL4
SERIAL_CONTROL_SERIAL5 = 105, //SERIAL5
SERIAL_CONTROL_SERIAL6 = 106, //SERIAL6
SERIAL_CONTROL_SERIAL7 = 107, //SERIAL7
SERIAL_CONTROL_SERIAL8 = 108, //SERIAL8
SERIAL_CONTROL_SERIAL9 = 109;  //SERIAL9
}

/**
SERIAL_CONTROL flags (bitmask)*/
 @Flags enum SERIAL_CONTROL_FLAG
{ 
;

 final int
SERIAL_CONTROL_FLAG_REPLY = 1, //Set if this is a reply
SERIAL_CONTROL_FLAG_RESPOND = 2, //Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
/**
Set if access to the serial port should be removed from whatever driver is currently using it, giving
exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
this flag se*/
SERIAL_CONTROL_FLAG_EXCLUSIVE = 4, 
SERIAL_CONTROL_FLAG_BLOCKING = 8, //Block on writes to the serial port
SERIAL_CONTROL_FLAG_MULTI = 16;  //Send multiple replies until port is drained
}

/**
Enumeration of distance sensor types*/
enum MAV_DISTANCE_SENSOR
{ 
;

 final int
MAV_DISTANCE_SENSOR_LASER = 0, //Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
MAV_DISTANCE_SENSOR_ULTRASOUND = 1, //Ultrasound rangefinder, e.g. MaxBotix units
MAV_DISTANCE_SENSOR_INFRARED = 2, //Infrared rangefinder, e.g. Sharp units
MAV_DISTANCE_SENSOR_RADAR = 3, //Radar type, e.g. uLanding units
MAV_DISTANCE_SENSOR_UNKNOWN = 4;  //Broken or unknown type, e.g. analog units
}

/**
Enumeration of sensor orientation, according to its rotations*/
enum MAV_SENSOR_ORIENTATION
{ 
;

 final int
MAV_SENSOR_ROTATION_NONE = 0, //Roll: 0, Pitch: 0, Yaw: 0
MAV_SENSOR_ROTATION_YAW_45 = 1, //Roll: 0, Pitch: 0, Yaw: 45
MAV_SENSOR_ROTATION_YAW_90 = 2, //Roll: 0, Pitch: 0, Yaw: 90
MAV_SENSOR_ROTATION_YAW_135 = 3, //Roll: 0, Pitch: 0, Yaw: 135
MAV_SENSOR_ROTATION_YAW_180 = 4, //Roll: 0, Pitch: 0, Yaw: 180
MAV_SENSOR_ROTATION_YAW_225 = 5, //Roll: 0, Pitch: 0, Yaw: 225
MAV_SENSOR_ROTATION_YAW_270 = 6, //Roll: 0, Pitch: 0, Yaw: 270
MAV_SENSOR_ROTATION_YAW_315 = 7, //Roll: 0, Pitch: 0, Yaw: 315
MAV_SENSOR_ROTATION_ROLL_180 = 8, //Roll: 180, Pitch: 0, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9, //Roll: 180, Pitch: 0, Yaw: 45
MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10, //Roll: 180, Pitch: 0, Yaw: 90
MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11, //Roll: 180, Pitch: 0, Yaw: 135
MAV_SENSOR_ROTATION_PITCH_180 = 12, //Roll: 0, Pitch: 180, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13, //Roll: 180, Pitch: 0, Yaw: 225
MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14, //Roll: 180, Pitch: 0, Yaw: 270
MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15, //Roll: 180, Pitch: 0, Yaw: 315
MAV_SENSOR_ROTATION_ROLL_90 = 16, //Roll: 90, Pitch: 0, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17, //Roll: 90, Pitch: 0, Yaw: 45
MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18, //Roll: 90, Pitch: 0, Yaw: 90
MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19, //Roll: 90, Pitch: 0, Yaw: 135
MAV_SENSOR_ROTATION_ROLL_270 = 20, //Roll: 270, Pitch: 0, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21, //Roll: 270, Pitch: 0, Yaw: 45
MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22, //Roll: 270, Pitch: 0, Yaw: 90
MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23, //Roll: 270, Pitch: 0, Yaw: 135
MAV_SENSOR_ROTATION_PITCH_90 = 24, //Roll: 0, Pitch: 90, Yaw: 0
MAV_SENSOR_ROTATION_PITCH_270 = 25, //Roll: 0, Pitch: 270, Yaw: 0
MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26, //Roll: 0, Pitch: 180, Yaw: 90
MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27, //Roll: 0, Pitch: 180, Yaw: 270
MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28, //Roll: 90, Pitch: 90, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29, //Roll: 180, Pitch: 90, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30, //Roll: 270, Pitch: 90, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31, //Roll: 90, Pitch: 180, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32, //Roll: 270, Pitch: 180, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33, //Roll: 90, Pitch: 270, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34, //Roll: 180, Pitch: 270, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35, //Roll: 270, Pitch: 270, Yaw: 0
MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36, //Roll: 90, Pitch: 180, Yaw: 90
MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37, //Roll: 90, Pitch: 0, Yaw: 270
MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 = 38, //Roll: 90, Pitch: 68, Yaw: 293
MAV_SENSOR_ROTATION_PITCH_315 = 39, //Pitch: 315
MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 = 40, //Roll: 90, Pitch: 315
MAV_SENSOR_ROTATION_CUSTOM = 100;  //Custom orientation
}

/**
Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability*/
 @Flags enum MAV_PROTOCOL_CAPABILITY
{ 
;

 final int
MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1, //Autopilot supports MISSION float message type.
MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2, //Autopilot supports the new param float message type.
MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4, //Autopilot supports MISSION_INT scaled integer message type.
MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8, //Autopilot supports COMMAND_INT scaled integer message type.
MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16, //Autopilot supports the new param union message type.
MAV_PROTOCOL_CAPABILITY_FTP = 32, //Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64, //Autopilot supports commanding attitude offboard.
MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128, //Autopilot supports commanding position and velocity targets in local NED frame.
MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256, //Autopilot supports commanding position and velocity targets in global scaled integers.
MAV_PROTOCOL_CAPABILITY_TERRAIN = 512, //Autopilot supports terrain protocol / data handling.
MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024, //Autopilot supports direct actuator control.
MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048, //Autopilot supports the flight termination command.
MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096, //Autopilot supports onboard compass calibration.
MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192, //Autopilot supports MAVLink version 2.
MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384, //Autopilot supports mission fence protocol.
MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768, //Autopilot supports mission rally point protocol.
MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536;  //Autopilot supports the flight information protocol.
}

/**
Type of mission items being requested/sent in mission protocol.*/
enum MAV_MISSION_TYPE
{ 
;

 final int
MAV_MISSION_TYPE_MISSION = 0, //Items are mission commands for main mission.
MAV_MISSION_TYPE_FENCE = 1, //Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
/**
Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT
rally point items*/
MAV_MISSION_TYPE_RALLY = 2, 
MAV_MISSION_TYPE_ALL = 255;  //Only used in MISSION_CLEAR_ALL to clear all mission types.
}

/**
Enumeration of estimator types*/
enum MAV_ESTIMATOR_TYPE
{ 
;

 final int
MAV_ESTIMATOR_TYPE_UNKNOWN = 0, //Unknown type of the estimator.
MAV_ESTIMATOR_TYPE_NAIVE = 1, //This is a naive estimator without any real covariance feedback.
MAV_ESTIMATOR_TYPE_VISION = 2, //Computer vision based estimate. Might be up to scale.
MAV_ESTIMATOR_TYPE_VIO = 3, //Visual-inertial estimate.
MAV_ESTIMATOR_TYPE_GPS = 4, //Plain GPS estimate.
MAV_ESTIMATOR_TYPE_GPS_INS = 5, //Estimator integrating GPS and inertial sensing.
MAV_ESTIMATOR_TYPE_MOCAP = 6, //Estimate from external motion capturing system.
MAV_ESTIMATOR_TYPE_LIDAR = 7, //Estimator based on lidar sensor input.
MAV_ESTIMATOR_TYPE_AUTOPILOT = 8;  //Estimator on autopilot.
}

/**
Enumeration of battery types*/
enum MAV_BATTERY_TYPE
{ 
;

 final int
MAV_BATTERY_TYPE_UNKNOWN = 0, //Not specified.
MAV_BATTERY_TYPE_LIPO = 1, //Lithium polymer battery
MAV_BATTERY_TYPE_LIFE = 2, //Lithium-iron-phosphate battery
MAV_BATTERY_TYPE_LION = 3, //Lithium-ION battery
MAV_BATTERY_TYPE_NIMH = 4;  //Nickel metal hydride battery
}

/**
Enumeration of battery functions*/
enum MAV_BATTERY_FUNCTION
{ 
;

 final int
MAV_BATTERY_FUNCTION_UNKNOWN = 0, //Battery function is unknown
MAV_BATTERY_FUNCTION_ALL = 1, //Battery supports all flight systems
MAV_BATTERY_FUNCTION_PROPULSION = 2, //Battery for the propulsion system
MAV_BATTERY_FUNCTION_AVIONICS = 3, //Avionics battery
MAV_BATTERY_TYPE_PAYLOAD = 4;  //Payload battery
}

/**
Enumeration for battery charge states.*/
enum MAV_BATTERY_CHARGE_STATE
{ 
;

 final int
MAV_BATTERY_CHARGE_STATE_UNDEFINED = 0, //Low battery state is not provided
MAV_BATTERY_CHARGE_STATE_OK = 1, //Battery is not in low state. Normal operation.
MAV_BATTERY_CHARGE_STATE_LOW = 2, //Battery state is low, warn and monitor close.
MAV_BATTERY_CHARGE_STATE_CRITICAL = 3, //Battery state is critical, return or abort immediately.
/**
Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent
damage*/
MAV_BATTERY_CHARGE_STATE_EMERGENCY = 4, 
MAV_BATTERY_CHARGE_STATE_FAILED = 5, //Battery failed, damage unavoidable.
MAV_BATTERY_CHARGE_STATE_UNHEALTHY = 6, //Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited.
MAV_BATTERY_CHARGE_STATE_CHARGING = 7;  //Battery is charging.
}

/**
Smart battery supply status/fault flags (bitmask) for health indication.*/
 @Flags enum MAV_SMART_BATTERY_FAULT
{ 
;

 final int
MAV_SMART_BATTERY_FAULT_DEEP_DISCHARGE = 1, //Battery has deep discharged.
MAV_SMART_BATTERY_FAULT_SPIKES = 2, //Voltage spikes.
MAV_SMART_BATTERY_FAULT_SINGLE_CELL_FAIL = 4, //Single cell has failed.
MAV_SMART_BATTERY_FAULT_OVER_CURRENT = 8, //Over-current fault.
MAV_SMART_BATTERY_FAULT_OVER_TEMPERATURE = 16, //Over-temperature fault.
MAV_SMART_BATTERY_FAULT_UNDER_TEMPERATURE = 32;  //Under-temperature fault.
}

/**
Enumeration of VTOL states*/
enum MAV_VTOL_STATE
{ 
;

 final int
MAV_VTOL_STATE_UNDEFINED = 0, //MAV is not configured as VTOL
MAV_VTOL_STATE_TRANSITION_TO_FW = 1, //VTOL is in transition from multicopter to fixed-wing
MAV_VTOL_STATE_TRANSITION_TO_MC = 2, //VTOL is in transition from fixed-wing to multicopter
MAV_VTOL_STATE_MC = 3, //VTOL is in multicopter state
MAV_VTOL_STATE_FW = 4;  //VTOL is in fixed-wing state
}

/**
Enumeration of landed detector states*/
enum MAV_LANDED_STATE
{ 
;

 final int
MAV_LANDED_STATE_UNDEFINED = 0, //MAV landed state is unknown
MAV_LANDED_STATE_ON_GROUND = 1, //MAV is landed (on ground)
MAV_LANDED_STATE_IN_AIR = 2, //MAV is in air
MAV_LANDED_STATE_TAKEOFF = 3, //MAV currently taking off
MAV_LANDED_STATE_LANDING = 4;  //MAV currently landing
}

/**
Enumeration of the ADSB altimeter types*/
enum ADSB_ALTITUDE_TYPE
{ 
;

 final int
ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0, //Altitude reported from a Baro source using QNH reference
ADSB_ALTITUDE_TYPE_GEOMETRIC = 1;  //Altitude reported from a GNSS source
}

/**
ADSB classification for the type of vehicle emitting the transponder signal*/
enum ADSB_EMITTER_TYPE
{ 
;

 final int
ADSB_EMITTER_TYPE_NO_INFO = 0, 
ADSB_EMITTER_TYPE_LIGHT = 1, 
ADSB_EMITTER_TYPE_SMALL = 2, 
ADSB_EMITTER_TYPE_LARGE = 3, 
ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4, 
ADSB_EMITTER_TYPE_HEAVY = 5, 
ADSB_EMITTER_TYPE_HIGHLY_MANUV = 6, 
ADSB_EMITTER_TYPE_ROTOCRAFT = 7, 
ADSB_EMITTER_TYPE_UNASSIGNED = 8, 
ADSB_EMITTER_TYPE_GLIDER = 9, 
ADSB_EMITTER_TYPE_LIGHTER_AIR = 10, 
ADSB_EMITTER_TYPE_PARACHUTE = 11, 
ADSB_EMITTER_TYPE_ULTRA_LIGHT = 12, 
ADSB_EMITTER_TYPE_UNASSIGNED2 = 13, 
ADSB_EMITTER_TYPE_UAV = 14, 
ADSB_EMITTER_TYPE_SPACE = 15, 
ADSB_EMITTER_TYPE_UNASSGINED3 = 16, 
ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17, 
ADSB_EMITTER_TYPE_SERVICE_SURFACE = 18, 
ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19;  
}

/**
These flags indicate status such as data validity of each data source. Set = data valid*/
 @Flags enum ADSB_FLAGS
{ 
;

 final int
ADSB_FLAGS_VALID_COORDS = 1, 
ADSB_FLAGS_VALID_ALTITUDE = 2, 
ADSB_FLAGS_VALID_HEADING = 4, 
ADSB_FLAGS_VALID_VELOCITY = 8, 
ADSB_FLAGS_VALID_CALLSIGN = 16, 
ADSB_FLAGS_VALID_SQUAWK = 32, 
ADSB_FLAGS_SIMULATED = 64, 
ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128, 
ADSB_FLAGS_BARO_VALID = 256, 
ADSB_FLAGS_SOURCE_UAT = 32768;  
}

/**
Bitmap of options for the MAV_CMD_DO_REPOSITION*/
enum MAV_DO_REPOSITION_FLAGS
{ 
;

 final int
MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1;  //The aircraft should immediately transition into guided. This should not be set for follow me application
}

/**
Flags in EKF_STATUS message*/
 @Flags enum ESTIMATOR_STATUS_FLAGS
{ 
;

 final int
ESTIMATOR_ATTITUDE = 1, //True if the attitude estimate is good
ESTIMATOR_VELOCITY_HORIZ = 2, //True if the horizontal velocity estimate is good
ESTIMATOR_VELOCITY_VERT = 4, //True if the  vertical velocity estimate is good
ESTIMATOR_POS_HORIZ_REL = 8, //True if the horizontal position (relative) estimate is good
ESTIMATOR_POS_HORIZ_ABS = 16, //True if the horizontal position (absolute) estimate is good
ESTIMATOR_POS_VERT_ABS = 32, //True if the vertical position (absolute) estimate is good
ESTIMATOR_POS_VERT_AGL = 64, //True if the vertical position (above ground) estimate is good
/**
True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
flow*/
ESTIMATOR_CONST_POS_MODE = 128, 
ESTIMATOR_PRED_POS_HORIZ_REL = 256, //True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
ESTIMATOR_PRED_POS_HORIZ_ABS = 512, //True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
ESTIMATOR_GPS_GLITCH = 1024, //True if the EKF has detected a GPS glitch
ESTIMATOR_ACCEL_ERROR = 2048;  //True if the EKF has detected bad accelerometer data
}
 @Flags enum MOTOR_TEST_ORDER
{ 
;

 final int
MOTOR_TEST_ORDER_DEFAULT = 0, //default autopilot motor test method
MOTOR_TEST_ORDER_SEQUENCE = 1, //motor numbers are specified as their index in a predefined vehicle-specific sequence
MOTOR_TEST_ORDER_BOARD = 2;  //motor numbers are specified as the output as labeled on the board
}
enum MOTOR_TEST_THROTTLE_TYPE
{ 
;

 final int
MOTOR_TEST_THROTTLE_PERCENT = 0, //throttle as a percentage from 0 ~ 100
MOTOR_TEST_THROTTLE_PWM = 1, //throttle as an absolute PWM value (normally in range of 1000~2000)
MOTOR_TEST_THROTTLE_PILOT = 2, //throttle pass-through from pilot's transmitter
MOTOR_TEST_COMPASS_CAL = 3;  //per-motor compass calibration test
}
 @Flags enum GPS_INPUT_IGNORE_FLAGS
{ 
;

 final int
GPS_INPUT_IGNORE_FLAG_ALT = 1, //ignore altitude field
GPS_INPUT_IGNORE_FLAG_HDOP = 2, //ignore hdop field
GPS_INPUT_IGNORE_FLAG_VDOP = 4, //ignore vdop field
GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8, //ignore horizontal velocity field (vn and ve)
GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16, //ignore vertical velocity field (vd)
GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32, //ignore speed accuracy field
GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64, //ignore horizontal accuracy field
GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128;  //ignore vertical accuracy field
}

/**
Possible actions an aircraft can take to avoid a collision.*/
enum MAV_COLLISION_ACTION
{ 
;

 final int
MAV_COLLISION_ACTION_NONE = 0, //Ignore any potential collisions
MAV_COLLISION_ACTION_REPORT = 1, //Report potential collision
MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2, //Ascend or Descend to avoid threat
MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3, //Move horizontally to avoid threat
MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4, //Aircraft to move perpendicular to the collision's velocity vector
MAV_COLLISION_ACTION_RTL = 5, //Aircraft to fly directly back to its launch point
MAV_COLLISION_ACTION_HOVER = 6;  //Aircraft to stop in place
}

/**
Aircraft-rated danger from this threat.*/
 @Flags enum MAV_COLLISION_THREAT_LEVEL
{ 
;

 final int
MAV_COLLISION_THREAT_LEVEL_NONE = 0, //Not a threat
MAV_COLLISION_THREAT_LEVEL_LOW = 1, //Craft is mildly concerned about this threat
MAV_COLLISION_THREAT_LEVEL_HIGH = 2;  //Craft is panicking, and may take actions to avoid threat
}

/**
Source of information about this collision.*/
enum MAV_COLLISION_SRC
{ 
;

 final int
MAV_COLLISION_SRC_ADSB = 0, //ID field references ADSB_VEHICLE packets
MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1;  //ID field references MAVLink SRC ID
}

/**
Type of GPS fix*/
enum GPS_FIX_TYPE
{ 
;

 final int
GPS_FIX_TYPE_NO_GPS = 0, //No GPS connected
GPS_FIX_TYPE_NO_FIX = 1, //No position information, GPS is connected
GPS_FIX_TYPE_2D_FIX = 2, //2D position
GPS_FIX_TYPE_3D_FIX = 3, //3D position
GPS_FIX_TYPE_DGPS = 4, //DGPS/SBAS aided 3D position
GPS_FIX_TYPE_RTK_FLOAT = 5, //RTK float, 3D position
GPS_FIX_TYPE_RTK_FIXED = 6, //RTK Fixed, 3D position
GPS_FIX_TYPE_STATIC = 7, //Static fixed, typically used for base stations
GPS_FIX_TYPE_PPP = 8;  //PPP, 3D position.
}

/**
RTK GPS baseline coordinate system, used for RTK corrections*/
enum RTK_BASELINE_COORDINATE_SYSTEM
{ 
;

 final int
RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0, //Earth-centered, Earth-fixed
RTK_BASELINE_COORDINATE_SYSTEM_NED = 1;  //RTK basestation centered, north, east, down
}

/**
Type of landing target*/
enum LANDING_TARGET_TYPE
{ 
;

 final int
LANDING_TARGET_TYPE_LIGHT_BEACON = 0, //Landing target signaled by light beacon (ex: IR-LOCK)
LANDING_TARGET_TYPE_RADIO_BEACON = 1, //Landing target signaled by radio beacon (ex: ILS, NDB)
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2, //Landing target represented by a fiducial marker (ex: ARTag)
LANDING_TARGET_TYPE_VISION_OTHER = 3;  //Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
}

/**
Direction of VTOL transition*/
enum VTOL_TRANSITION_HEADING
{ 
;

 final int
VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT = 0, //Respect the heading configuration of the vehicle.
VTOL_TRANSITION_HEADING_NEXT_WAYPOINT = 1, //Use the heading pointing towards the next waypoint.
VTOL_TRANSITION_HEADING_TAKEOFF = 2, //Use the heading on takeoff (while sitting on the ground).
VTOL_TRANSITION_HEADING_SPECIFIED = 3, //Use the specified heading in parameter 4.
/**
Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning
is active)*/
VTOL_TRANSITION_HEADING_ANY = 4;  
}

/**
Camera capability flags (Bitmap)*/
 @Flags enum CAMERA_CAP_FLAGS
{ 
;

 final int
CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1, //Camera is able to record video
CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2, //Camera is able to capture images
CAMERA_CAP_FLAGS_HAS_MODES = 4, //Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8, //Camera can capture images while in video mode
CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16, //Camera can capture videos while in Photo/Image mode
CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32, //Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM = 64, //Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS = 128, //Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
/**
Camera has video streaming capabilities (use MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION for video streaming
info*/
CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM = 256;  
}

/**
Stream status flags (Bitmap)*/
enum VIDEO_STREAM_STATUS_FLAGS
{ 
;

 final int
VIDEO_STREAM_STATUS_FLAGS_RUNNING = 1, //Stream is active (running)
VIDEO_STREAM_STATUS_FLAGS_THERMAL = 2;  //Stream is thermal imaging
}

/**
Video stream types*/
enum VIDEO_STREAM_TYPE
{ 
;

 final int
VIDEO_STREAM_TYPE_RTSP = 0, //Stream is RTSP
VIDEO_STREAM_TYPE_RTPUDP = 1, //Stream is RTP UDP (URI gives the port number)
VIDEO_STREAM_TYPE_TCP_MPEG = 2, //Stream is MPEG on TCP
VIDEO_STREAM_TYPE_MPEG_TS_H264 = 3;  //Stream is h.264 on MPEG TS (URI gives the port number)
}

/**
Zoom types for MAV_CMD_SET_CAMERA_ZOOM*/
enum CAMERA_ZOOM_TYPE
{ 
;

 final int
ZOOM_TYPE_STEP = 0, //Zoom one step increment (-1 for wide, 1 for tele)
ZOOM_TYPE_CONTINUOUS = 1, //Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)
ZOOM_TYPE_RANGE = 2, //Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
/**
Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom range
of the camera, so this can type can only be used for cameras where the zoom range is known (implying
that this cannot reliably be used in a GCS for an arbitrary camera*/
ZOOM_TYPE_FOCAL_LENGTH = 3;  
}

/**
Focus types for MAV_CMD_SET_CAMERA_FOCUS*/
enum SET_FOCUS_TYPE
{ 
;

 final int
FOCUS_TYPE_STEP = 0, //Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
/**
Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to
stop focusing*/
FOCUS_TYPE_CONTINUOUS = 1, 
FOCUS_TYPE_RANGE = 2, //Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
/**
Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this
can type can only be used for cameras where the range is known (implying that this cannot reliably be
used in a GCS for an arbitrary camera)*/
FOCUS_TYPE_METERS = 3;  
}

/**
Result from a PARAM_EXT_SET message.*/
enum PARAM_ACK
{ 
;

 final int
PARAM_ACK_ACCEPTED = 0, //Parameter value ACCEPTED and SET
PARAM_ACK_VALUE_UNSUPPORTED = 1, //Parameter value UNKNOWN/UNSUPPORTED
PARAM_ACK_FAILED = 2, //Parameter failed to set
/**
Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
is completed with the actual result. These are for parameters that may take longer to set. Instead of
waiting for an ACK and potentially timing out, you will immediately receive this response to let you
know it was received*/
PARAM_ACK_IN_PROGRESS = 3;  
}

/**
Camera Modes.*/
 @Flags enum CAMERA_MODE
{ 
;

 final int
CAMERA_MODE_IMAGE = 0, //Camera is in image/photo capture mode.
CAMERA_MODE_VIDEO = 1, //Camera is in video capture mode.
CAMERA_MODE_IMAGE_SURVEY = 2;  //Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
}
enum MAV_ARM_AUTH_DENIED_REASON
{ 
;

 final int
MAV_ARM_AUTH_DENIED_REASON_GENERIC = 0, //Not a specific reason
MAV_ARM_AUTH_DENIED_REASON_NONE = 1, //Authorizer will send the error as string to GCS
MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2, //At least one waypoint have a invalid value
MAV_ARM_AUTH_DENIED_REASON_TIMEOUT = 3, //Timeout in the authorizer process(in case it depends on network)
/**
Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that
caused it to be denied*/
MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4, 
MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5;  //Weather is not good to fly
}

/**
RC type*/
enum RC_TYPE
{ 
;

 final int
RC_TYPE_SPEKTRUM_DSM2 = 0, //Spektrum DSM2
RC_TYPE_SPEKTRUM_DSMX = 1;  //Spektrum DSMX
}

/**
Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set
the floats afx afy afz should be interpreted as force instead of acceleration*/
 @Flags enum POSITION_TARGET_TYPEMASK
{ 
;

 final int
POSITION_TARGET_TYPEMASK_X_IGNORE = 1, //Ignore position x
POSITION_TARGET_TYPEMASK_Y_IGNORE = 2, //Ignore position y
POSITION_TARGET_TYPEMASK_Z_IGNORE = 4, //Ignore position z
POSITION_TARGET_TYPEMASK_VX_IGNORE = 8, //Ignore velocity x
POSITION_TARGET_TYPEMASK_VY_IGNORE = 16, //Ignore velocity y
POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32, //Ignore velocity z
POSITION_TARGET_TYPEMASK_AX_IGNORE = 64, //Ignore acceleration x
POSITION_TARGET_TYPEMASK_AY_IGNORE = 128, //Ignore acceleration y
POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256, //Ignore acceleration z
POSITION_TARGET_TYPEMASK_FORCE_SET = 512, //Use force instead of acceleration
POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024, //Ignore yaw
POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048;  //Ignore yaw rate
}

/**
Airborne status of UAS.*/
enum UTM_FLIGHT_STATE
{ 
;

 final int
UTM_FLIGHT_STATE_UNKNOWN = 1, //The flight state can't be determined.
UTM_FLIGHT_STATE_GROUND = 2, //UAS on ground.
UTM_FLIGHT_STATE_AIRBORNE = 3, //UAS airborne.
UTM_FLIGHT_STATE_EMERGENCY = 16, //UAS is in an emergency flight state.
UTM_FLIGHT_STATE_NOCTRL = 32;  //UAS has no active controls.
}

/**
Flags for the global position report.*/
 @Flags enum UTM_DATA_AVAIL_FLAGS
{ 
;

 final int
UTM_DATA_AVAIL_FLAGS_TIME_VALID = 1, //The field time contains valid data.
UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE = 2, //The field uas_id contains valid data.
UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE = 4, //The fields lat, lon and h_acc contain valid data.
UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE = 8, //The fields alt and v_acc contain valid data.
UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE = 16, //The field relative_alt contains valid data.
UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE = 32, //The fields vx and vy contain valid data.
UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE = 64, //The field vz contains valid data.
UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE = 128;  //The fields next_lat, next_lon and next_alt contain valid data.
}

/**
Cellular network radio type*/
enum CELLULAR_NETWORK_RADIO_TYPE
{ 
;

 final int
CELLULAR_NETWORK_RADIO_TYPE_NONE = 0, 
CELLULAR_NETWORK_RADIO_TYPE_GSM = 1, 
CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2, 
CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3, 
CELLULAR_NETWORK_RADIO_TYPE_LTE = 4;  
}

/**
These flags encode the cellular network status*/
enum CELLULAR_NETWORK_STATUS_FLAG
{ 
;

 final int
CELLULAR_NETWORK_STATUS_FLAG_ROAMING = 1;  //Roaming is active
}

/**
Precision land modes (used in MAV_CMD_NAV_LAND).*/
 @Flags enum PRECISION_LAND_MODE
{ 
;

 final int
PRECISION_LAND_MODE_DISABLED = 0, //Normal (non-precision) landing.
PRECISION_LAND_MODE_OPPORTUNISTIC = 1, //Use precision landing if beacon detected when land command accepted, otherwise land normally.
/**
Use precision landing, searching for beacon if not found when land command accepted (land normally if
beacon cannot be found)*/
PRECISION_LAND_MODE_REQUIRED = 2;  
}
 @Flags enum PARACHUTE_ACTION
{ 
;

 final int
PARACHUTE_DISABLE = 0, //Disable parachute release.
PARACHUTE_ENABLE = 1, //Enable parachute release.
PARACHUTE_RELEASE = 2;  //Release parachute.
}
enum MAV_TUNNEL_PAYLOAD_TYPE
{ 
;

 final int
MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN = 0, //Encoding of payload unknown.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208, //Registered for STorM32 gimbal controller.
MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209;  //Registered for STorM32 gimbal controller.
}
enum MAV_ODID_ID_TYPE
{ 
;

 final int
MAV_ODID_ID_TYPE_NONE = 0, //No type defined.
MAV_ODID_ID_TYPE_SERIAL_NUMBER = 1, //Manufacturer Serial Number (ANSI/CTA-2063 format).
MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID = 2, //.
MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID = 3;  //UTM (Unmanned Traffic Management) assigned UUID (RFC4122).
}
enum MAV_ODID_UA_TYPE
{ 
;

 final int
MAV_ODID_UA_TYPE_NONE = 0, //No UA (Unmanned Aircraft) type defined.
MAV_ODID_UA_TYPE_AEROPLANE = 1, //Aeroplane/Airplane. Fixed wing.
MAV_ODID_UA_TYPE_ROTORCRAFT = 2, //Rotorcraft (including Multirotor).
MAV_ODID_UA_TYPE_GYROPLANE = 3, //Gyroplane.
MAV_ODID_UA_TYPE_VTOL = 4, //VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
MAV_ODID_UA_TYPE_ORNITHOPTER = 5, //Ornithopter.
MAV_ODID_UA_TYPE_GLIDER = 6, //Glider.
MAV_ODID_UA_TYPE_KITE = 7, //Kite.
MAV_ODID_UA_TYPE_FREE_BALLOON = 8, //Free Balloon.
MAV_ODID_UA_TYPE_CAPTIVE_BALLOON = 9, //Captive Balloon.
MAV_ODID_UA_TYPE_AIRSHIP = 10, //Airship. E.g. a blimp.
MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE = 11, //Free Fall/Parachute.
MAV_ODID_UA_TYPE_ROCKET = 12, //Rocket.
MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT = 13, //Tethered powered aircraft.
MAV_ODID_UA_TYPE_GROUND_OBSTACLE = 14, //Ground Obstacle.
MAV_ODID_UA_TYPE_OTHER = 15;  //Other type of aircraft not listed earlier.
}
 @Flags enum MAV_ODID_STATUS
{ 
;

 final int
MAV_ODID_STATUS_UNDECLARED = 0, //The status of the (UA) Unmanned Aircraft is undefined.
MAV_ODID_STATUS_GROUND = 1, //The UA is on the ground.
MAV_ODID_STATUS_AIRBORNE = 2;  //The UA is in the air.
}
enum MAV_ODID_HEIGHT_REF
{ 
;

 final int
MAV_ODID_HEIGHT_REF_OVER_TAKEOFF = 0, //The height field is relative to the take-off location.
MAV_ODID_HEIGHT_REF_OVER_GROUND = 1;  //The height field is relative to ground.
}
enum MAV_ODID_HOR_ACC
{ 
;

 final int
MAV_ODID_HOR_ACC_UNKNOWN = 0, //The horizontal accuracy is unknown.
MAV_ODID_HOR_ACC_10NM = 1, //The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km.
MAV_ODID_HOR_ACC_4NM = 2, //The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km.
MAV_ODID_HOR_ACC_2NM = 3, //The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km.
MAV_ODID_HOR_ACC_1NM = 4, //The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km.
MAV_ODID_HOR_ACC_0_5NM = 5, //The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m.
MAV_ODID_HOR_ACC_0_3NM = 6, //The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m.
MAV_ODID_HOR_ACC_0_1NM = 7, //The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m.
MAV_ODID_HOR_ACC_0_05NM = 8, //The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m.
MAV_ODID_HOR_ACC_30_METER = 9, //The horizontal accuracy is smaller than 30 meter.
MAV_ODID_HOR_ACC_10_METER = 10, //The horizontal accuracy is smaller than 10 meter.
MAV_ODID_HOR_ACC_3_METER = 11, //The horizontal accuracy is smaller than 3 meter.
MAV_ODID_HOR_ACC_1_METER = 12;  //The horizontal accuracy is smaller than 1 meter.
}
enum MAV_ODID_VER_ACC
{ 
;

 final int
MAV_ODID_VER_ACC_UNKNOWN = 0, //The vertical accuracy is unknown.
MAV_ODID_VER_ACC_150_METER = 1, //The vertical accuracy is smaller than 150 meter.
MAV_ODID_VER_ACC_45_METER = 2, //The vertical accuracy is smaller than 45 meter.
MAV_ODID_VER_ACC_25_METER = 3, //The vertical accuracy is smaller than 25 meter.
MAV_ODID_VER_ACC_10_METER = 4, //The vertical accuracy is smaller than 10 meter.
MAV_ODID_VER_ACC_3_METER = 5, //The vertical accuracy is smaller than 3 meter.
MAV_ODID_VER_ACC_1_METER = 6;  //The vertical accuracy is smaller than 1 meter.
}
enum MAV_ODID_SPEED_ACC
{ 
;

 final int
MAV_ODID_SPEED_ACC_UNKNOWN = 0, //The speed accuracy is unknown.
MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND = 1, //The speed accuracy is smaller than 10 meters per second.
MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND = 2, //The speed accuracy is smaller than 3 meters per second.
MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND = 3, //The speed accuracy is smaller than 1 meters per second.
MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND = 4;  //The speed accuracy is smaller than 0.3 meters per second.
}
enum MAV_ODID_TIME_ACC
{ 
;

 final int
MAV_ODID_TIME_ACC_UNKNOWN = 0, //The timestamp accuracy is unknown.
MAV_ODID_TIME_ACC_0_1_SECOND = 1, //The timestamp accuracy is smaller than 0.1 second.
MAV_ODID_TIME_ACC_0_2_SECOND = 2, //The timestamp accuracy is smaller than 0.2 second.
MAV_ODID_TIME_ACC_0_3_SECOND = 3, //The timestamp accuracy is smaller than 0.3 second.
MAV_ODID_TIME_ACC_0_4_SECOND = 4, //The timestamp accuracy is smaller than 0.4 second.
MAV_ODID_TIME_ACC_0_5_SECOND = 5, //The timestamp accuracy is smaller than 0.5 second.
MAV_ODID_TIME_ACC_0_6_SECOND = 6, //The timestamp accuracy is smaller than 0.6 second.
MAV_ODID_TIME_ACC_0_7_SECOND = 7, //The timestamp accuracy is smaller than 0.7 second.
MAV_ODID_TIME_ACC_0_8_SECOND = 8, //The timestamp accuracy is smaller than 0.8 second.
MAV_ODID_TIME_ACC_0_9_SECOND = 9, //The timestamp accuracy is smaller than 0.9 second.
MAV_ODID_TIME_ACC_1_0_SECOND = 10, //The timestamp accuracy is smaller than 1.0 second.
MAV_ODID_TIME_ACC_1_1_SECOND = 11, //The timestamp accuracy is smaller than 1.1 second.
MAV_ODID_TIME_ACC_1_2_SECOND = 12, //The timestamp accuracy is smaller than 1.2 second.
MAV_ODID_TIME_ACC_1_3_SECOND = 13, //The timestamp accuracy is smaller than 1.3 second.
MAV_ODID_TIME_ACC_1_4_SECOND = 14, //The timestamp accuracy is smaller than 1.4 second.
MAV_ODID_TIME_ACC_1_5_SECOND = 15;  //The timestamp accuracy is smaller than 1.5 second.
}
enum MAV_ODID_AUTH_TYPE
{ 
;

 final int
MAV_ODID_AUTH_TYPE_NONE = 0, //No authentication type is specified.
MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE = 1, //Signature for the UAS (Unmanned Aircraft System) ID.
MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE = 2, //Signature for the Operator ID.
MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE = 3, //Signature for the entire message set.
MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID = 4;  //Authentication is provided by Network Remote ID.
}
enum MAV_ODID_DESC_TYPE
{ 
;

 final int
MAV_ODID_DESC_TYPE_TEXT = 0;  //Free-form text description of the purpose of the flight.
}
 @Flags enum MAV_ODID_LOCATION_SRC
{ 
;

 final int
MAV_ODID_LOCATION_SRC_TAKEOFF = 0, //The location of the operator is the same as the take-off location.
MAV_ODID_LOCATION_SRC_LIVE_GNSS = 1, //The location of the operator is based on live GNSS data.
MAV_ODID_LOCATION_SRC_FIXED = 2;  //The location of the operator is a fixed location.
}
enum MAV_ODID_OPERATOR_ID_TYPE
{ 
;

 final int
MAV_ODID_OPERATOR_ID_TYPE_CAA = 0;  //CAA (Civil Aviation Authority) registered operator ID.
}

/**
Tune formats (used for vehicle buzzer/tone generation).*/
enum TUNE_FORMAT
{ 
;

 final int
TUNE_FORMAT_QBASIC1_1 = 1, //Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm.
TUNE_FORMAT_MML_MODERN = 2;  //Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML
}

/**
Component capability flags (Bitmap)*/
enum COMPONENT_CAP_FLAGS
{ 
;

 final int
COMPONENT_CAP_FLAGS_PARAM = 1, //Component has parameters, and supports the parameter protocol (PARAM messages).
COMPONENT_CAP_FLAGS_PARAM_EXT = 2;  //Component has parameters, and supports the extended parameter protocol (PARAM_EXT messages).
}

/**
Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html*/
enum AIS_TYPE
{ 
;

 final int
AIS_TYPE_UNKNOWN = 0, //Not available (default).
AIS_TYPE_RESERVED_1 = 1, 
AIS_TYPE_RESERVED_2 = 2, 
AIS_TYPE_RESERVED_3 = 3, 
AIS_TYPE_RESERVED_4 = 4, 
AIS_TYPE_RESERVED_5 = 5, 
AIS_TYPE_RESERVED_6 = 6, 
AIS_TYPE_RESERVED_7 = 7, 
AIS_TYPE_RESERVED_8 = 8, 
AIS_TYPE_RESERVED_9 = 9, 
AIS_TYPE_RESERVED_10 = 10, 
AIS_TYPE_RESERVED_11 = 11, 
AIS_TYPE_RESERVED_12 = 12, 
AIS_TYPE_RESERVED_13 = 13, 
AIS_TYPE_RESERVED_14 = 14, 
AIS_TYPE_RESERVED_15 = 15, 
AIS_TYPE_RESERVED_16 = 16, 
AIS_TYPE_RESERVED_17 = 17, 
AIS_TYPE_RESERVED_18 = 18, 
AIS_TYPE_RESERVED_19 = 19, 
AIS_TYPE_WIG = 20, //Wing In Ground effect.
AIS_TYPE_WIG_HAZARDOUS_A = 21, 
AIS_TYPE_WIG_HAZARDOUS_B = 22, 
AIS_TYPE_WIG_HAZARDOUS_C = 23, 
AIS_TYPE_WIG_HAZARDOUS_D = 24, 
AIS_TYPE_WIG_RESERVED_1 = 25, 
AIS_TYPE_WIG_RESERVED_2 = 26, 
AIS_TYPE_WIG_RESERVED_3 = 27, 
AIS_TYPE_WIG_RESERVED_4 = 28, 
AIS_TYPE_WIG_RESERVED_5 = 29, 
AIS_TYPE_FISHING = 30, 
AIS_TYPE_TOWING = 31, 
AIS_TYPE_TOWING_LARGE = 32, //Towing: length exceeds 200m or breadth exceeds 25m.
AIS_TYPE_DREDGING = 33, //Dredging or other underwater ops.
AIS_TYPE_DIVING = 34, 
AIS_TYPE_MILITARY = 35, 
AIS_TYPE_SAILING = 36, 
AIS_TYPE_PLEASURE = 37, 
AIS_TYPE_RESERVED_20 = 38, 
AIS_TYPE_RESERVED_21 = 39, 
AIS_TYPE_HSC = 40, //High Speed Craft.
AIS_TYPE_HSC_HAZARDOUS_A = 41, 
AIS_TYPE_HSC_HAZARDOUS_B = 42, 
AIS_TYPE_HSC_HAZARDOUS_C = 43, 
AIS_TYPE_HSC_HAZARDOUS_D = 44, 
AIS_TYPE_HSC_RESERVED_1 = 45, 
AIS_TYPE_HSC_RESERVED_2 = 46, 
AIS_TYPE_HSC_RESERVED_3 = 47, 
AIS_TYPE_HSC_RESERVED_4 = 48, 
AIS_TYPE_HSC_UNKNOWN = 49, 
AIS_TYPE_PILOT = 50, 
AIS_TYPE_SAR = 51, //Search And Rescue vessel.
AIS_TYPE_TUG = 52, 
AIS_TYPE_PORT_TENDER = 53, 
AIS_TYPE_ANTI_POLLUTION = 54, //Anti-pollution equipment.
AIS_TYPE_LAW_ENFORCEMENT = 55, 
AIS_TYPE_SPARE_LOCAL_1 = 56, 
AIS_TYPE_SPARE_LOCAL_2 = 57, 
AIS_TYPE_MEDICAL_TRANSPORT = 58, 
AIS_TYPE_NONECOMBATANT = 59, //Noncombatant ship according to RR Resolution No. 18.
AIS_TYPE_PASSENGER = 60, 
AIS_TYPE_PASSENGER_HAZARDOUS_A = 61, 
AIS_TYPE_PASSENGER_HAZARDOUS_B = 62, 
AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C = 63, 
AIS_TYPE_PASSENGER_HAZARDOUS_D = 64, 
AIS_TYPE_PASSENGER_RESERVED_1 = 65, 
AIS_TYPE_PASSENGER_RESERVED_2 = 66, 
AIS_TYPE_PASSENGER_RESERVED_3 = 67, 
AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4 = 68, 
AIS_TYPE_PASSENGER_UNKNOWN = 69, 
AIS_TYPE_CARGO = 70, 
AIS_TYPE_CARGO_HAZARDOUS_A = 71, 
AIS_TYPE_CARGO_HAZARDOUS_B = 72, 
AIS_TYPE_CARGO_HAZARDOUS_C = 73, 
AIS_TYPE_CARGO_HAZARDOUS_D = 74, 
AIS_TYPE_CARGO_RESERVED_1 = 75, 
AIS_TYPE_CARGO_RESERVED_2 = 76, 
AIS_TYPE_CARGO_RESERVED_3 = 77, 
AIS_TYPE_CARGO_RESERVED_4 = 78, 
AIS_TYPE_CARGO_UNKNOWN = 79, 
AIS_TYPE_TANKER = 80, 
AIS_TYPE_TANKER_HAZARDOUS_A = 81, 
AIS_TYPE_TANKER_HAZARDOUS_B = 82, 
AIS_TYPE_TANKER_HAZARDOUS_C = 83, 
AIS_TYPE_TANKER_HAZARDOUS_D = 84, 
AIS_TYPE_TANKER_RESERVED_1 = 85, 
AIS_TYPE_TANKER_RESERVED_2 = 86, 
AIS_TYPE_TANKER_RESERVED_3 = 87, 
AIS_TYPE_TANKER_RESERVED_4 = 88, 
AIS_TYPE_TANKER_UNKNOWN = 89, 
AIS_TYPE_OTHER = 90, 
AIS_TYPE_OTHER_HAZARDOUS_A = 91, 
AIS_TYPE_OTHER_HAZARDOUS_B = 92, 
AIS_TYPE_OTHER_HAZARDOUS_C = 93, 
AIS_TYPE_OTHER_HAZARDOUS_D = 94, 
AIS_TYPE_OTHER_RESERVED_1 = 95, 
AIS_TYPE_OTHER_RESERVED_2 = 96, 
AIS_TYPE_OTHER_RESERVED_3 = 97, 
AIS_TYPE_OTHER_RESERVED_4 = 98, 
AIS_TYPE_OTHER_UNKNOWN = 99;  
}

/**
Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.htm*/
enum AIS_NAV_STATUS
{ 
;

 final int
UNDER_WAY = 0, //Under way using engine.
AIS_NAV_ANCHORED = 1, 
AIS_NAV_UN_COMMANDED = 2, 
AIS_NAV_RESTRICTED_MANOEUVERABILITY = 3, 
AIS_NAV_DRAUGHT_CONSTRAINED = 4, 
AIS_NAV_MOORED = 5, 
AIS_NAV_AGROUND = 6, 
AIS_NAV_FISHING = 7, 
AIS_NAV_SAILING = 8, 
AIS_NAV_RESERVED_HSC = 9, 
AIS_NAV_RESERVED_WIG = 10, 
AIS_NAV_RESERVED_1 = 11, 
AIS_NAV_RESERVED_2 = 12, 
AIS_NAV_RESERVED_3 = 13, 
AIS_NAV_AIS_SART = 14, //Search And Rescue Transponder.
AIS_NAV_UNKNOWN = 15;  //Not available (default).
}

/**
These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message
fields. When set, the data is valid*/
 @Flags enum AIS_FLAGS
{ 
;

 final int
AIS_FLAGS_POSITION_ACCURACY = 1, //1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m.
AIS_FLAGS_VALID_COG = 2, 
AIS_FLAGS_VALID_VELOCITY = 4, 
AIS_FLAGS_HIGH_VELOCITY = 8, //1 = Velocity over 52.5765m/s (102.2 knots)
AIS_FLAGS_VALID_TURN_RATE = 16, 
AIS_FLAGS_TURN_RATE_SIGN_ONLY = 32, //Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30
AIS_FLAGS_VALID_DIMENSIONS = 64, 
AIS_FLAGS_LARGE_BOW_DIMENSION = 128, //Distance to bow is larger than 511m
AIS_FLAGS_LARGE_STERN_DIMENSION = 256, //Distance to stern is larger than 511m
AIS_FLAGS_LARGE_PORT_DIMENSION = 512, //Distance to port side is larger than 63m
AIS_FLAGS_LARGE_STARBOARD_DIMENSION = 1024, //Distance to starboard side is larger than 63m
AIS_FLAGS_VALID_CALLSIGN = 2048, 
AIS_FLAGS_VALID_NAME = 4096;  
}

 }