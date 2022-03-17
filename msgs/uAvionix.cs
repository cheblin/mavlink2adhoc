using System;
using org.unirail.Meta;

namespace org.mavlink
{
    public interface uAvionix
    {
        interface CommunicationChannel : GroundControl.CommunicationInterface, MicroAirVehicle.CommunicationInterface { }

        class GroundControl : InJAVA, InCS
        {
            public interface CommunicationInterface
            {
                /**
Micro air vehicle / autopilot classes. This identifies the individual model.
*/
                enum MAV_AUTOPILOT
                {
                    /**
Generic autopilot, full support for everything
*/
                    MAV_AUTOPILOT_GENERIC = 0,

                    /**
Reserved for future use.
*/
                    MAV_AUTOPILOT_RESERVED = 1,

                    /**
SLUGS autopilot, http://slugsuav.soe.ucsc.edu
*/
                    MAV_AUTOPILOT_SLUGS = 2,

                    /**
ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org
*/
                    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,

                    /**
OpenPilot, http://openpilot.org
*/
                    MAV_AUTOPILOT_OPENPILOT = 4,

                    /**
Generic autopilot only supporting simple waypoints
*/
                    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,

                    /**
Generic autopilot supporting waypoints and other simple navigation commands
*/
                    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,

                    /**
Generic autopilot supporting the full mission command set
*/
                    MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,

                    /**
No valid autopilot, e.g. a GCS or other MAVLink component
*/
                    MAV_AUTOPILOT_INVALID = 8,

                    /**
PPZ UAV - http://nongnu.org/paparazzi
*/
                    MAV_AUTOPILOT_PPZ = 9,

                    /**
UAV Dev Board
*/
                    MAV_AUTOPILOT_UDB = 10,

                    /**
FlexiPilot
*/
                    MAV_AUTOPILOT_FP = 11,

                    /**
PX4 Autopilot - http://px4.io/
*/
                    MAV_AUTOPILOT_PX4 = 12,

                    /**
SMACCMPilot - http://smaccmpilot.org
*/
                    MAV_AUTOPILOT_SMACCMPILOT = 13,

                    /**
AutoQuad -- http://autoquad.org
*/
                    MAV_AUTOPILOT_AUTOQUAD = 14,

                    /**
Armazila -- http://armazila.com
*/
                    MAV_AUTOPILOT_ARMAZILA = 15,

                    /**
Aerob -- http://aerob.ru
*/
                    MAV_AUTOPILOT_AEROB = 16,

                    /**
ASLUAV autopilot -- http://www.asl.ethz.ch
*/
                    MAV_AUTOPILOT_ASLUAV = 17,

                    /**
SmartAP Autopilot - http://sky-drones.com
*/
                    MAV_AUTOPILOT_SMARTAP = 18,

                    /**
AirRails - http://uaventure.com
*/
                    MAV_AUTOPILOT_AIRRAILS = 19,

                    /**
Fusion Reflex - https://fusion.engineering
*/
                    MAV_AUTOPILOT_REFLEX = 20,
                }

                /**
MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle
on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate
for their type (e.g. a camera must use MAV_TYPE_CAMERA).
*/
                enum MAV_TYPE
                {
                    /**
Generic micro air vehicle
*/
                    MAV_TYPE_GENERIC = 0,

                    /**
Fixed wing aircraft.
*/
                    MAV_TYPE_FIXED_WING = 1,

                    /**
Quadrotor
*/
                    MAV_TYPE_QUADROTOR = 2,

                    /**
Coaxial helicopter
*/
                    MAV_TYPE_COAXIAL = 3,

                    /**
Normal helicopter with tail rotor.
*/
                    MAV_TYPE_HELICOPTER = 4,

                    /**
Ground installation
*/
                    MAV_TYPE_ANTENNA_TRACKER = 5,

                    /**
Operator control unit / ground control station
*/
                    MAV_TYPE_GCS = 6,

                    /**
Airship, controlled
*/
                    MAV_TYPE_AIRSHIP = 7,

                    /**
Free balloon, uncontrolled
*/
                    MAV_TYPE_FREE_BALLOON = 8,

                    /**
Rocket
*/
                    MAV_TYPE_ROCKET = 9,

                    /**
Ground rover
*/
                    MAV_TYPE_GROUND_ROVER = 10,

                    /**
Surface vessel, boat, ship
*/
                    MAV_TYPE_SURFACE_BOAT = 11,

                    /**
Submarine
*/
                    MAV_TYPE_SUBMARINE = 12,

                    /**
Hexarotor
*/
                    MAV_TYPE_HEXAROTOR = 13,

                    /**
Octorotor
*/
                    MAV_TYPE_OCTOROTOR = 14,

                    /**
Tricopter
*/
                    MAV_TYPE_TRICOPTER = 15,

                    /**
Flapping wing
*/
                    MAV_TYPE_FLAPPING_WING = 16,

                    /**
Kite
*/
                    MAV_TYPE_KITE = 17,

                    /**
Onboard companion controller
*/
                    MAV_TYPE_ONBOARD_CONTROLLER = 18,

                    /**
Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
*/
                    MAV_TYPE_VTOL_DUOROTOR = 19,

                    /**
Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
*/
                    MAV_TYPE_VTOL_QUADROTOR = 20,

                    /**
Tiltrotor VTOL
*/
                    MAV_TYPE_VTOL_TILTROTOR = 21,

                    /**
VTOL reserved 2
*/
                    MAV_TYPE_VTOL_RESERVED2 = 22,

                    /**
VTOL reserved 3
*/
                    MAV_TYPE_VTOL_RESERVED3 = 23,

                    /**
VTOL reserved 4
*/
                    MAV_TYPE_VTOL_RESERVED4 = 24,

                    /**
VTOL reserved 5
*/
                    MAV_TYPE_VTOL_RESERVED5 = 25,

                    /**
Gimbal
*/
                    MAV_TYPE_GIMBAL = 26,

                    /**
ADSB system
*/
                    MAV_TYPE_ADSB = 27,

                    /**
Steerable, nonrigid airfoil
*/
                    MAV_TYPE_PARAFOIL = 28,

                    /**
Dodecarotor
*/
                    MAV_TYPE_DODECAROTOR = 29,

                    /**
Camera
*/
                    MAV_TYPE_CAMERA = 30,

                    /**
Charging station
*/
                    MAV_TYPE_CHARGING_STATION = 31,

                    /**
FLARM collision avoidance system
*/
                    MAV_TYPE_FLARM = 32,

                    /**
Servo
*/
                    MAV_TYPE_SERVO = 33,

                    /**
Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.
*/
                    MAV_TYPE_ODID = 34,

                    /**
Decarotor
*/
                    MAV_TYPE_DECAROTOR = 35,

                    /**
Battery
*/
                    MAV_TYPE_BATTERY = 36,

                    /**
Parachute
*/
                    MAV_TYPE_PARACHUTE = 37,

                    /**
Log
*/
                    MAV_TYPE_LOG = 38,

                    /**
OSD
*/
                    MAV_TYPE_OSD = 39,

                    /**
IMU
*/
                    MAV_TYPE_IMU = 40,

                    /**
GPS
*/
                    MAV_TYPE_GPS = 41,

                    /**
Winch
*/
                    MAV_TYPE_WINCH = 42,
                }

                /**
These flags encode the MAV mode.
*/
                enum MAV_MODE_FLAG
                {
                    /**
0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
shall be used instead. The flag can still be used to report the armed state.
*/
                    MAV_MODE_FLAG_SAFETY_ARMED = 128,

                    /**
0b01000000 remote control input is enabled.
*/
                    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,

                    /**
0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
is full operational.
*/
                    MAV_MODE_FLAG_HIL_ENABLED = 32,

                    /**
0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
control inputs to move around.
*/
                    MAV_MODE_FLAG_STABILIZE_ENABLED = 16,

                    /**
0b00001000 guided mode enabled, system flies waypoints / mission items.
*/
                    MAV_MODE_FLAG_GUIDED_ENABLED = 8,

                    /**
0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
depends on the actual implementation.
*/
                    MAV_MODE_FLAG_AUTO_ENABLED = 4,

                    /**
0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
not be used for stable implementations.
*/
                    MAV_MODE_FLAG_TEST_ENABLED = 2,

                    /**
0b00000001 Reserved for future use.
*/
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
                }

                /**
These values encode the bit positions of the decode position. These values can be used to read the value
of a flag bit by combining the base_mode variable with AND with the flag position value. The result will
be either 0 or 1, depending on if the flag is set or not.
*/
                enum MAV_MODE_FLAG_DECODE_POSITION
                {
                    /**
First bit:  10000000
*/
                    MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 128,

                    /**
Second bit: 01000000
*/
                    MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64,

                    /**
Third bit:  00100000
*/
                    MAV_MODE_FLAG_DECODE_POSITION_HIL = 32,

                    /**
Fourth bit: 00010000
*/
                    MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16,

                    /**
Fifth bit:  00001000
*/
                    MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8,

                    /**
Sixth bit:   00000100
*/
                    MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4,

                    /**
Seventh bit: 00000010
*/
                    MAV_MODE_FLAG_DECODE_POSITION_TEST = 2,

                    /**
Eighth bit: 00000001
*/
                    MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1,
                }

                enum MAV_STATE
                {
                    /**
Uninitialized system, state is unknown.
*/
                    MAV_STATE_UNINIT = 0,

                    /**
System is booting up.
*/
                    MAV_STATE_BOOT = 1,

                    /**
System is calibrating and not flight-ready.
*/
                    MAV_STATE_CALIBRATING = 2,

                    /**
System is grounded and on standby. It can be launched any time.
*/
                    MAV_STATE_STANDBY = 3,

                    /**
System is active and might be already airborne. Motors are engaged.
*/
                    MAV_STATE_ACTIVE = 4,

                    /**
System is in a non-normal flight mode. It can however still navigate.
*/
                    MAV_STATE_CRITICAL = 5,

                    /**
System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
mayday and going down.
*/
                    MAV_STATE_EMERGENCY = 6,

                    /**
System just initialized its power-down sequence, will shut down now.
*/
                    MAV_STATE_POWEROFF = 7,

                    /**
System is terminating itself.
*/
                    MAV_STATE_FLIGHT_TERMINATION = 8,
                }

                /**
When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should
be allocated sequential values. An appropriate number of values should be left free after these components
to allow the number of instances to be expanded.
*/
                enum MAV_COMPONENT
                {
                    /**
Target id (target_component) used to broadcast messages to all components of the receiving system. Components
should attempt to process messages with this component ID and forward to components on any other interfaces.
Note: This is not a valid *source* component id for a message.
*/
                    MAV_COMP_ID_ALL = 0,

                    /**
System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.
*/
                    MAV_COMP_ID_AUTOPILOT1 = 1,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER1 = 25,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER2 = 26,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER3 = 27,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER4 = 28,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER5 = 29,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER6 = 30,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER7 = 31,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER8 = 32,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER9 = 33,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER10 = 34,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER11 = 35,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER12 = 36,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER13 = 37,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER14 = 38,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER15 = 39,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER16 = 40,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER17 = 41,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER18 = 42,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER19 = 43,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER20 = 44,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER21 = 45,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER22 = 46,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER23 = 47,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER24 = 48,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER25 = 49,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER26 = 50,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER27 = 51,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER28 = 52,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER29 = 53,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER30 = 54,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER31 = 55,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER32 = 56,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER33 = 57,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER34 = 58,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER35 = 59,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER36 = 60,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER37 = 61,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER38 = 62,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER39 = 63,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER40 = 64,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER41 = 65,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER42 = 66,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER43 = 67,

                    /**
Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages).
*/
                    MAV_COMP_ID_TELEMETRY_RADIO = 68,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER45 = 69,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER46 = 70,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER47 = 71,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER48 = 72,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER49 = 73,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER50 = 74,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER51 = 75,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER52 = 76,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER53 = 77,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER54 = 78,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER55 = 79,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER56 = 80,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER57 = 81,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER58 = 82,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER59 = 83,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER60 = 84,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER61 = 85,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER62 = 86,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER63 = 87,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER64 = 88,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER65 = 89,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER66 = 90,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER67 = 91,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER68 = 92,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER69 = 93,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER70 = 94,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER71 = 95,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER72 = 96,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER73 = 97,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER74 = 98,

                    /**
Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published
by components outside of the private network.
*/
                    MAV_COMP_ID_USER75 = 99,

                    /**
Camera #1.
*/
                    MAV_COMP_ID_CAMERA = 100,

                    /**
Camera #2.
*/
                    MAV_COMP_ID_CAMERA2 = 101,

                    /**
Camera #3.
*/
                    MAV_COMP_ID_CAMERA3 = 102,

                    /**
Camera #4.
*/
                    MAV_COMP_ID_CAMERA4 = 103,

                    /**
Camera #5.
*/
                    MAV_COMP_ID_CAMERA5 = 104,

                    /**
Camera #6.
*/
                    MAV_COMP_ID_CAMERA6 = 105,

                    /**
Servo #1.
*/
                    MAV_COMP_ID_SERVO1 = 140,

                    /**
Servo #2.
*/
                    MAV_COMP_ID_SERVO2 = 141,

                    /**
Servo #3.
*/
                    MAV_COMP_ID_SERVO3 = 142,

                    /**
Servo #4.
*/
                    MAV_COMP_ID_SERVO4 = 143,

                    /**
Servo #5.
*/
                    MAV_COMP_ID_SERVO5 = 144,

                    /**
Servo #6.
*/
                    MAV_COMP_ID_SERVO6 = 145,

                    /**
Servo #7.
*/
                    MAV_COMP_ID_SERVO7 = 146,

                    /**
Servo #8.
*/
                    MAV_COMP_ID_SERVO8 = 147,

                    /**
Servo #9.
*/
                    MAV_COMP_ID_SERVO9 = 148,

                    /**
Servo #10.
*/
                    MAV_COMP_ID_SERVO10 = 149,

                    /**
Servo #11.
*/
                    MAV_COMP_ID_SERVO11 = 150,

                    /**
Servo #12.
*/
                    MAV_COMP_ID_SERVO12 = 151,

                    /**
Servo #13.
*/
                    MAV_COMP_ID_SERVO13 = 152,

                    /**
Servo #14.
*/
                    MAV_COMP_ID_SERVO14 = 153,

                    /**
Gimbal #1.
*/
                    MAV_COMP_ID_GIMBAL = 154,

                    /**
Logging component.
*/
                    MAV_COMP_ID_LOG = 155,

                    /**
Automatic Dependent Surveillance-Broadcast (ADS-B) component.
*/
                    MAV_COMP_ID_ADSB = 156,

                    /**
On Screen Display (OSD) devices for video links.
*/
                    MAV_COMP_ID_OSD = 157,

                    /**
Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.
*/
                    MAV_COMP_ID_PERIPHERAL = 158,

                    /**
Gimbal ID for QX1.
*/
                    MAV_COMP_ID_QX1_GIMBAL = 159,

                    /**
FLARM collision alert component.
*/
                    MAV_COMP_ID_FLARM = 160,

                    /**
Parachute component.
*/
                    MAV_COMP_ID_PARACHUTE = 161,

                    /**
Gimbal #2.
*/
                    MAV_COMP_ID_GIMBAL2 = 171,

                    /**
Gimbal #3.
*/
                    MAV_COMP_ID_GIMBAL3 = 172,

                    /**
Gimbal #4
*/
                    MAV_COMP_ID_GIMBAL4 = 173,

                    /**
Gimbal #5.
*/
                    MAV_COMP_ID_GIMBAL5 = 174,

                    /**
Gimbal #6.
*/
                    MAV_COMP_ID_GIMBAL6 = 175,

                    /**
Battery #1.
*/
                    MAV_COMP_ID_BATTERY = 180,

                    /**
Battery #2.
*/
                    MAV_COMP_ID_BATTERY2 = 181,

                    /**
CAN over MAVLink client.
*/
                    MAV_COMP_ID_MAVCAN = 189,

                    /**
Component that can generate/supply a mission flight plan (e.g. GCS or developer API).
*/
                    MAV_COMP_ID_MISSIONPLANNER = 190,

                    /**
Component that lives on the onboard computer (companion computer) and has some generic functionalities,
such as settings system parameters and monitoring the status of some processes that don't directly speak
mavlink and so on.
*/
                    MAV_COMP_ID_ONBOARD_COMPUTER = 191,

                    /**
Component that lives on the onboard computer (companion computer) and has some generic functionalities,
such as settings system parameters and monitoring the status of some processes that don't directly speak
mavlink and so on.
*/
                    MAV_COMP_ID_ONBOARD_COMPUTER2 = 192,

                    /**
Component that lives on the onboard computer (companion computer) and has some generic functionalities,
such as settings system parameters and monitoring the status of some processes that don't directly speak
mavlink and so on.
*/
                    MAV_COMP_ID_ONBOARD_COMPUTER3 = 193,

                    /**
Component that lives on the onboard computer (companion computer) and has some generic functionalities,
such as settings system parameters and monitoring the status of some processes that don't directly speak
mavlink and so on.
*/
                    MAV_COMP_ID_ONBOARD_COMPUTER4 = 194,

                    /**
Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap,
shortest path, cost, etc.).
*/
                    MAV_COMP_ID_PATHPLANNER = 195,

                    /**
Component that plans a collision free path between two points.
*/
                    MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196,

                    /**
Component that provides position estimates using VIO techniques.
*/
                    MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197,

                    /**
Component that manages pairing of vehicle and GCS.
*/
                    MAV_COMP_ID_PAIRING_MANAGER = 198,

                    /**
Inertial Measurement Unit (IMU) #1.
*/
                    MAV_COMP_ID_IMU = 200,

                    /**
Inertial Measurement Unit (IMU) #2.
*/
                    MAV_COMP_ID_IMU_2 = 201,

                    /**
Inertial Measurement Unit (IMU) #3.
*/
                    MAV_COMP_ID_IMU_3 = 202,

                    /**
GPS #1.
*/
                    MAV_COMP_ID_GPS = 220,

                    /**
GPS #2.
*/
                    MAV_COMP_ID_GPS2 = 221,

                    /**
Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
*/
                    MAV_COMP_ID_ODID_TXRX_1 = 236,

                    /**
Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
*/
                    MAV_COMP_ID_ODID_TXRX_2 = 237,

                    /**
Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
*/
                    MAV_COMP_ID_ODID_TXRX_3 = 238,

                    /**
Component to bridge MAVLink to UDP (i.e. from a UART).
*/
                    MAV_COMP_ID_UDP_BRIDGE = 240,

                    /**
Component to bridge to UART (i.e. from UDP).
*/
                    MAV_COMP_ID_UART_BRIDGE = 241,

                    /**
Component handling TUNNEL messages (e.g. vendor specific GUI of a component).
*/
                    MAV_COMP_ID_TUNNEL_NODE = 242,

                    /**
Component for handling system messages (e.g. to ARM, takeoff, etc.).
*/
                    MAV_COMP_ID_SYSTEM_CONTROL = 250,
                }

                /**
The heartbeat message shows that a system or component is present and responding. The type and autopilot
fields (along with the message component id), allow the receiving system to treat further messages from
this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice
is documented at https://mavlink.io/en/services/heartbeat.html
*/
                class HEARTBEAT
                {
                    /**
Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter,
etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference
to component id for identifying the component type.
*/
                    MAV_TYPE Typ;

                    /**
Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
*/
                    MAV_AUTOPILOT autopilot;

                    /**
System mode bitmap.
*/
                    MAV_MODE_FLAG base_mode;

                    /**
A bitfield for use for autopilot-specific flags
*/
                    uint custom_mode;

                    /**
System status flag.
*/
                    MAV_STATE system_status;

                    /**
MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
*/
                    sbyte mavlink_version;
                }

                /**
Version and capability of protocol version. This message can be requested with MAV_CMD_REQUEST_MESSAGE
and is used as part of the handshaking to establish which MAVLink version should be used on the network.
Every node should respond to a request for PROTOCOL_VERSION to enable the handshaking. Library implementers
should consider adding this into the default decoding state machine to allow the protocol core to respond
directly.
*/
                class PROTOCOL_VERSION
                {
                    /**
Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
*/
                    ushort version;

                    /**
Minimum MAVLink version supported
*/
                    ushort min_version;

                    /**
Maximum MAVLink version supported (set to the same value as version by default)
*/
                    ushort max_version;

                    /**
The first 8 bytes (not characters printed in hex!) of the git hash.
*/
                    [Dims(+8)] byte spec_version_hash;

                    /**
The first 8 bytes (not characters printed in hex!) of the git hash.
*/
                    [Dims(+8)] byte library_version_hash;
                }

                /**
These values define the type of firmware release.  These values indicate the first version or release
of this type.  For example the first alpha release would be 64, the second would be 65.
*/
                enum FIRMWARE_VERSION_TYPE
                {
                    /**
development release
*/
                    FIRMWARE_VERSION_TYPE_DEV = 0,

                    /**
alpha release
*/
                    FIRMWARE_VERSION_TYPE_ALPHA = 64,

                    /**
beta release
*/
                    FIRMWARE_VERSION_TYPE_BETA = 128,

                    /**
release candidate
*/
                    FIRMWARE_VERSION_TYPE_RC = 192,

                    /**
official stable release
*/
                    FIRMWARE_VERSION_TYPE_OFFICIAL = 255,
                }

                /**
Flags to report failure cases over the high latency telemtry.
*/
                enum HL_FAILURE_FLAG
                {
                    /**
GPS failure.
*/
                    HL_FAILURE_FLAG_GPS = 1,

                    /**
Differential pressure sensor failure.
*/
                    HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE = 2,

                    /**
Absolute pressure sensor failure.
*/
                    HL_FAILURE_FLAG_ABSOLUTE_PRESSURE = 4,

                    /**
Accelerometer sensor failure.
*/
                    HL_FAILURE_FLAG_3D_ACCEL = 8,

                    /**
Gyroscope sensor failure.
*/
                    HL_FAILURE_FLAG_3D_GYRO = 16,

                    /**
Magnetometer sensor failure.
*/
                    HL_FAILURE_FLAG_3D_MAG = 32,

                    /**
Terrain subsystem failure.
*/
                    HL_FAILURE_FLAG_TERRAIN = 64,

                    /**
Battery failure/critical low battery.
*/
                    HL_FAILURE_FLAG_BATTERY = 128,

                    /**
RC receiver failure/no rc connection.
*/
                    HL_FAILURE_FLAG_RC_RECEIVER = 256,

                    /**
Offboard link failure.
*/
                    HL_FAILURE_FLAG_OFFBOARD_LINK = 512,

                    /**
Engine failure.
*/
                    HL_FAILURE_FLAG_ENGINE = 1024,

                    /**
Geofence violation.
*/
                    HL_FAILURE_FLAG_GEOFENCE = 2048,

                    /**
Estimator failure, for example measurement rejection or large variances.
*/
                    HL_FAILURE_FLAG_ESTIMATOR = 4096,

                    /**
Mission failure.
*/
                    HL_FAILURE_FLAG_MISSION = 8192,
                }

                /**
Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
*/
                enum MAV_GOTO
                {
                    /**
Hold at the current position.
*/
                    MAV_GOTO_DO_HOLD = 0,

                    /**
Continue with the next item in mission execution.
*/
                    MAV_GOTO_DO_CONTINUE = 1,

                    /**
Hold at the current position of the system
*/
                    MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2,

                    /**
Hold at the position specified in the parameters of the DO_HOLD action
*/
                    MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3,
                }

                /**
These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but
it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes
as a safety override.
*/
                enum MAV_MODE
                {
                    /**
System is not ready to fly, booting, calibrating, etc. No flag is set.
*/
                    MAV_MODE_PREFLIGHT = 0,

                    /**
System is allowed to be active, under assisted RC control.
*/
                    MAV_MODE_STABILIZE_DISARMED = 80,

                    /**
System is allowed to be active, under assisted RC control.
*/
                    MAV_MODE_STABILIZE_ARMED = 208,

                    /**
System is allowed to be active, under manual (RC) control, no stabilization
*/
                    MAV_MODE_MANUAL_DISARMED = 64,

                    /**
System is allowed to be active, under manual (RC) control, no stabilization
*/
                    MAV_MODE_MANUAL_ARMED = 192,

                    /**
System is allowed to be active, under autonomous control, manual setpoint
*/
                    MAV_MODE_GUIDED_DISARMED = 88,

                    /**
System is allowed to be active, under autonomous control, manual setpoint
*/
                    MAV_MODE_GUIDED_ARMED = 216,

                    /**
System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
and not pre-programmed by waypoints)
*/
                    MAV_MODE_AUTO_DISARMED = 92,

                    /**
System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
and not pre-programmed by waypoints)
*/
                    MAV_MODE_AUTO_ARMED = 220,

                    /**
UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
*/
                    MAV_MODE_TEST_DISARMED = 66,

                    /**
UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
*/
                    MAV_MODE_TEST_ARMED = 194,
                }

                /**
These encode the sensors whose status is sent as part of the SYS_STATUS message.
*/
                enum MAV_SYS_STATUS_SENSOR : long
                {
                    /**
0x01 3D gyro
*/
                    MAV_SYS_STATUS_SENSOR_3D_GYRO = 1,

                    /**
0x02 3D accelerometer
*/
                    MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2,

                    /**
0x04 3D magnetometer
*/
                    MAV_SYS_STATUS_SENSOR_3D_MAG = 4,

                    /**
0x08 absolute pressure
*/
                    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8,

                    /**
0x10 differential pressure
*/
                    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16,

                    /**
0x20 GPS
*/
                    MAV_SYS_STATUS_SENSOR_GPS = 32,

                    /**
0x40 optical flow
*/
                    MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64,

                    /**
0x80 computer vision position
*/
                    MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128,

                    /**
0x100 laser based position
*/
                    MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256,

                    /**
0x200 external ground truth (Vicon or Leica)
*/
                    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512,

                    /**
0x400 3D angular rate control
*/
                    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024,

                    /**
0x800 attitude stabilization
*/
                    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,

                    /**
0x1000 yaw position
*/
                    MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096,

                    /**
0x2000 z/altitude control
*/
                    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192,

                    /**
0x4000 x/y position control
*/
                    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384,

                    /**
0x8000 motor outputs / control
*/
                    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768,

                    /**
0x10000 rc receiver
*/
                    MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536,

                    /**
0x20000 2nd 3D gyro
*/
                    MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072,

                    /**
0x40000 2nd 3D accelerometer
*/
                    MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144,

                    /**
0x80000 2nd 3D magnetometer
*/
                    MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288,

                    /**
0x100000 geofence
*/
                    MAV_SYS_STATUS_GEOFENCE = 1048576,

                    /**
0x200000 AHRS subsystem health
*/
                    MAV_SYS_STATUS_AHRS = 2097152,

                    /**
0x400000 Terrain subsystem health
*/
                    MAV_SYS_STATUS_TERRAIN = 4194304,

                    /**
0x800000 Motors are reversed
*/
                    MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,

                    /**
0x1000000 Logging
*/
                    MAV_SYS_STATUS_LOGGING = 16777216,

                    /**
0x2000000 Battery
*/
                    MAV_SYS_STATUS_SENSOR_BATTERY = 33554432,

                    /**
0x4000000 Proximity
*/
                    MAV_SYS_STATUS_SENSOR_PROXIMITY = 67108864,

                    /**
0x8000000 Satellite Communication 
*/
                    MAV_SYS_STATUS_SENSOR_SATCOM = 134217728,

                    /**
0x10000000 pre-arm check status. Always healthy when armed
*/
                    MAV_SYS_STATUS_PREARM_CHECK = 268435456,

                    /**
0x20000000 Avoidance/collision prevention
*/
                    MAV_SYS_STATUS_OBSTACLE_AVOIDANCE = 536870912,

                    /**
0x40000000 propulsion (actuator, esc, motor or propellor)
*/
                    MAV_SYS_STATUS_SENSOR_PROPULSION = 1073741824,

                    /**
0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present
only)
*/
                    MAV_SYS_STATUS_EXTENSION_USED = 2147483648,
                }

                /**
These encode the sensors whose status is sent as part of the SYS_STATUS message in the extended fields.
*/
                enum MAV_SYS_STATUS_SENSOR_EXTENDED
                {
                    /**
0x01 Recovery system (parachute, balloon, retracts etc)
*/
                    MAV_SYS_STATUS_RECOVERY_SYSTEM = 1,
                }

                /**
Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).


*/
                enum MAV_FRAME
                {
                    /**
Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude,
third value / z: positive altitude over mean sea level (MSL).
*/
                    MAV_FRAME_GLOBAL = 0,

                    /**
NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
*/
                    MAV_FRAME_LOCAL_NED = 1,

                    /**
NOT a coordinate frame, indicates a mission command.
*/
                    MAV_FRAME_MISSION = 2,

                    /**
Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second
value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
*/
                    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,

                    /**
ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
*/
                    MAV_FRAME_LOCAL_ENU = 4,

                    /**
Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second
value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).
*/
                    MAV_FRAME_GLOBAL_INT = 5,

                    /**
Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude
in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0
being at the altitude of the home location.
*/
                    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,

                    /**
NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
*/
                    MAV_FRAME_LOCAL_OFFSET_NED = 7,

                    /**
Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used
with velocity/accelaration values.
*/
                    MAV_FRAME_BODY_NED = 8,

                    /**
This is the same as MAV_FRAME_BODY_FRD.
*/
                    MAV_FRAME_BODY_OFFSET_NED = 9,

                    /**
Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude
in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with
0 being at ground level in terrain model.
*/
                    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,

                    /**
Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value /
x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude
in meters with 0 being at ground level in terrain model.
*/
                    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11,

                    /**
FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The forward
axis is aligned to the front of the vehicle in the horizontal plane.
*/
                    MAV_FRAME_BODY_FRD = 12,

                    /**
MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).
*/
                    MAV_FRAME_RESERVED_13 = 13,

                    /**
MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down
(x: North, y: East, z: Down).
*/
                    MAV_FRAME_RESERVED_14 = 14,

                    /**
MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x:
East, y: North, z: Up).
*/
                    MAV_FRAME_RESERVED_15 = 15,

                    /**
MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down
(x: North, y: East, z: Down).
*/
                    MAV_FRAME_RESERVED_16 = 16,

                    /**
MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up
(x: East, y: North, z: Up).
*/
                    MAV_FRAME_RESERVED_17 = 17,

                    /**
MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the
vehicle, Z-down (x: North, y: East, z: Down).
*/
                    MAV_FRAME_RESERVED_18 = 18,

                    /**
MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the
vehicle, Z-up (x: East, y: North, z: Up).
*/
                    MAV_FRAME_RESERVED_19 = 19,

                    /**
FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward
axis is aligned to the front of the vehicle in the horizontal plane.
*/
                    MAV_FRAME_LOCAL_FRD = 20,

                    /**
FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward
axis is aligned to the front of the vehicle in the horizontal plane.
*/
                    MAV_FRAME_LOCAL_FLU = 21,
                }

                enum MAVLINK_DATA_STREAM_TYPE
                {
                    MAVLINK_DATA_STREAM_IMG_JPEG   = 0,
                    MAVLINK_DATA_STREAM_IMG_BMP    = 1,
                    MAVLINK_DATA_STREAM_IMG_RAW8U  = 2,
                    MAVLINK_DATA_STREAM_IMG_RAW32U = 3,
                    MAVLINK_DATA_STREAM_IMG_PGM    = 4,
                    MAVLINK_DATA_STREAM_IMG_PNG    = 5,
                }

                /**
Actions following geofence breach.
*/
                enum FENCE_ACTION
                {
                    /**
Disable fenced mode. If used in a plan this would mean the next fence is disabled.
*/
                    FENCE_ACTION_NONE = 0,

                    /**
Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT in GUIDED mode. Note: This action is only supported by
ArduPlane, and may not be supported in all versions.
*/
                    FENCE_ACTION_GUIDED = 1,

                    /**
Report fence breach, but don't take action
*/
                    FENCE_ACTION_REPORT = 2,

                    /**
Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT with manual throttle control in GUIDED mode. Note: This
action is only supported by ArduPlane, and may not be supported in all versions.
*/
                    FENCE_ACTION_GUIDED_THR_PASS = 3,

                    /**
Return/RTL mode.
*/
                    FENCE_ACTION_RTL = 4,

                    /**
Hold at current location.
*/
                    FENCE_ACTION_HOLD = 5,

                    /**
Termination failsafe. Motors are shut down (some flight stacks may trigger other failsafe actions).
*/
                    FENCE_ACTION_TERMINATE = 6,

                    /**
Land at current location.
*/
                    FENCE_ACTION_LAND = 7,
                }

                enum FENCE_BREACH
                {
                    /**
No last fence breach
*/
                    FENCE_BREACH_NONE = 0,

                    /**
Breached minimum altitude
*/
                    FENCE_BREACH_MINALT = 1,

                    /**
Breached maximum altitude
*/
                    FENCE_BREACH_MAXALT = 2,

                    /**
Breached fence boundary
*/
                    FENCE_BREACH_BOUNDARY = 3,
                }

                /**
Actions being taken to mitigate/prevent fence breach
*/
                enum FENCE_MITIGATE
                {
                    /**
Unknown
*/
                    FENCE_MITIGATE_UNKNOWN = 0,

                    /**
No actions being taken
*/
                    FENCE_MITIGATE_NONE = 1,

                    /**
Velocity limiting active to prevent breach
*/
                    FENCE_MITIGATE_VEL_LIMIT = 2,
                }

                /**
Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages.
*/
                enum MAV_MOUNT_MODE
                {
                    /**
Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
*/
                    MAV_MOUNT_MODE_RETRACT = 0,

                    /**
Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
*/
                    MAV_MOUNT_MODE_NEUTRAL = 1,

                    /**
Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
*/
                    MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,

                    /**
Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
*/
                    MAV_MOUNT_MODE_RC_TARGETING = 3,

                    /**
Load neutral position and start to point to Lat,Lon,Alt
*/
                    MAV_MOUNT_MODE_GPS_POINT = 4,

                    /**
Gimbal tracks system with specified system ID
*/
                    MAV_MOUNT_MODE_SYSID_TARGET = 5,

                    /**
Gimbal tracks home location
*/
                    MAV_MOUNT_MODE_HOME_LOCATION = 6,
                }

                /**
Gimbal device (low level) capability flags (bitmap)
*/
                enum GIMBAL_DEVICE_CAP_FLAGS
                {
                    /**
Gimbal device supports a retracted position
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1,

                    /**
Gimbal device supports a horizontal, forward looking position, stabilized
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2,

                    /**
Gimbal device supports rotating around roll axis.
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4,

                    /**
Gimbal device supports to follow a roll angle relative to the vehicle
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8,

                    /**
Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16,

                    /**
Gimbal device supports rotating around pitch axis.
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32,

                    /**
Gimbal device supports to follow a pitch angle relative to the vehicle
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64,

                    /**
Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128,

                    /**
Gimbal device supports rotating around yaw axis.
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256,

                    /**
Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512,

                    /**
Gimbal device supports locking to an absolute heading (often this is an option available)
*/
                    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024,

                    /**
Gimbal device supports yawing/panning infinetely (e.g. using slip disk).
*/
                    GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048,
                }

                /**
Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS.
However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the
capabilities and thus add flags.
*/
                enum GIMBAL_MANAGER_CAP_FLAGS
                {
                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024,

                    /**
Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048,

                    /**
Gimbal manager supports to point to a local position.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536,

                    /**
Gimbal manager supports to point to a global latitude, longitude, altitude position.
*/
                    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072,
                }

                /**
Flags for gimbal device (lower level) operation.
*/
                enum GIMBAL_DEVICE_FLAGS
                {
                    /**
Set to retracted safe position (no stabilization), takes presedence over all other flags.
*/
                    GIMBAL_DEVICE_FLAGS_RETRACT = 1,

                    /**
Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly
forward-facing and horizontal (pitch=yaw=0) but may be any orientation.
*/
                    GIMBAL_DEVICE_FLAGS_NEUTRAL = 2,

                    /**
Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default
with a stabilizing gimbal.
*/
                    GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4,

                    /**
Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the
default.
*/
                    GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8,

                    /**
Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion
is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion
frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).
*/
                    GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16,
                }

                /**
Flags for high level gimbal manager operation The first 16 bits are identical to the GIMBAL_DEVICE_FLAGS.
*/
                enum GIMBAL_MANAGER_FLAGS
                {
                    /**
Based on GIMBAL_DEVICE_FLAGS_RETRACT
*/
                    GIMBAL_MANAGER_FLAGS_RETRACT = 1,

                    /**
Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
*/
                    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2,

                    /**
Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
*/
                    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4,

                    /**
Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
*/
                    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8,

                    /**
Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
*/
                    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16,
                }

                /**
Gimbal device (low level) error flags (bitmap, 0 means no error)
*/
                enum GIMBAL_DEVICE_ERROR_FLAGS
                {
                    /**
Gimbal device is limited by hardware roll limit.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1,

                    /**
Gimbal device is limited by hardware pitch limit.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2,

                    /**
Gimbal device is limited by hardware yaw limit.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4,

                    /**
There is an error with the gimbal encoders.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8,

                    /**
There is an error with the gimbal power source.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16,

                    /**
There is an error with the gimbal motor's.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32,

                    /**
There is an error with the gimbal's software.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64,

                    /**
There is an error with the gimbal's communication.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128,

                    /**
Gimbal is currently calibrating.
*/
                    GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256,
                }

                /**
Gripper actions.
*/
                enum GRIPPER_ACTIONS
                {
                    /**
Gripper release cargo.
*/
                    GRIPPER_ACTION_RELEASE = 0,

                    /**
Gripper grab onto cargo.
*/
                    GRIPPER_ACTION_GRAB = 1,
                }

                /**
Winch actions.
*/
                enum WINCH_ACTIONS
                {
                    /**
Allow motor to freewheel.
*/
                    WINCH_RELAXED = 0,

                    /**
Wind or unwind specified length of line, optionally using specified rate.
*/
                    WINCH_RELATIVE_LENGTH_CONTROL = 1,

                    /**
Wind or unwind line at specified rate.
*/
                    WINCH_RATE_CONTROL = 2,

                    /**
Perform the locking sequence to relieve motor while in the fully retracted position. Only action and instance
command parameters are used, others are ignored.
*/
                    WINCH_LOCK = 3,

                    /**
Sequence of drop, slow down, touch down, reel up, lock. Only action and instance command parameters are
used, others are ignored.
*/
                    WINCH_DELIVER = 4,

                    /**
Engage motor and hold current position. Only action and instance command parameters are used, others are
ignored.
*/
                    WINCH_HOLD = 5,

                    /**
Return the reel to the fully retracted position. Only action and instance command parameters are used,
others are ignored.
*/
                    WINCH_RETRACT = 6,

                    /**
Load the reel with line. The winch will calculate the total loaded length and stop when the tension exceeds
a threshold. Only action and instance command parameters are used, others are ignored.
*/
                    WINCH_LOAD_LINE = 7,

                    /**
Spool out the entire length of the line. Only action and instance command parameters are used, others
are ignored.
*/
                    WINCH_ABANDON_LINE = 8,
                }

                /**
Generalized UAVCAN node health
*/
                enum UAVCAN_NODE_HEALTH
                {
                    /**
The node is functioning properly.
*/
                    UAVCAN_NODE_HEALTH_OK = 0,

                    /**
A critical parameter went out of range or the node has encountered a minor failure.
*/
                    UAVCAN_NODE_HEALTH_WARNING = 1,

                    /**
The node has encountered a major failure.
*/
                    UAVCAN_NODE_HEALTH_ERROR = 2,

                    /**
The node has suffered a fatal malfunction.
*/
                    UAVCAN_NODE_HEALTH_CRITICAL = 3,
                }

                /**
Generalized UAVCAN node mode
*/
                enum UAVCAN_NODE_MODE
                {
                    /**
The node is performing its primary functions.
*/
                    UAVCAN_NODE_MODE_OPERATIONAL = 0,

                    /**
The node is initializing; this mode is entered immediately after startup.
*/
                    UAVCAN_NODE_MODE_INITIALIZATION = 1,

                    /**
The node is under maintenance.
*/
                    UAVCAN_NODE_MODE_MAINTENANCE = 2,

                    /**
The node is in the process of updating its software.
*/
                    UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,

                    /**
The node is no longer available online.
*/
                    UAVCAN_NODE_MODE_OFFLINE = 7,
                }

                /**
Indicates the ESC connection type.
*/
                enum ESC_CONNECTION_TYPE
                {
                    /**
Traditional PPM ESC.
*/
                    ESC_CONNECTION_TYPE_PPM = 0,

                    /**
Serial Bus connected ESC.
*/
                    ESC_CONNECTION_TYPE_SERIAL = 1,

                    /**
One Shot PPM ESC.
*/
                    ESC_CONNECTION_TYPE_ONESHOT = 2,

                    /**
I2C ESC.
*/
                    ESC_CONNECTION_TYPE_I2C = 3,

                    /**
CAN-Bus ESC.
*/
                    ESC_CONNECTION_TYPE_CAN = 4,

                    /**
DShot ESC.
*/
                    ESC_CONNECTION_TYPE_DSHOT = 5,
                }

                /**
Flags to report ESC failures.
*/
                enum ESC_FAILURE_FLAGS
                {
                    /**
No ESC failure.
*/
                    ESC_FAILURE_NONE = 0,

                    /**
Over current failure.
*/
                    ESC_FAILURE_OVER_CURRENT = 1,

                    /**
Over voltage failure.
*/
                    ESC_FAILURE_OVER_VOLTAGE = 2,

                    /**
Over temperature failure.
*/
                    ESC_FAILURE_OVER_TEMPERATURE = 4,

                    /**
Over RPM failure.
*/
                    ESC_FAILURE_OVER_RPM = 8,

                    /**
Inconsistent command failure i.e. out of bounds.
*/
                    ESC_FAILURE_INCONSISTENT_CMD = 16,

                    /**
Motor stuck failure.
*/
                    ESC_FAILURE_MOTOR_STUCK = 32,

                    /**
Generic ESC failure.
*/
                    ESC_FAILURE_GENERIC = 64,
                }

                /**
Flags to indicate the status of camera storage.
*/
                enum STORAGE_STATUS
                {
                    /**
Storage is missing (no microSD card loaded for example.)
*/
                    STORAGE_STATUS_EMPTY = 0,

                    /**
Storage present but unformatted.
*/
                    STORAGE_STATUS_UNFORMATTED = 1,

                    /**
Storage present and ready.
*/
                    STORAGE_STATUS_READY = 2,

                    /**
Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields
will be ignored.
*/
                    STORAGE_STATUS_NOT_SUPPORTED = 3,
                }

                /**
Flags to indicate the type of storage.
*/
                enum STORAGE_TYPE
                {
                    /**
Storage type is not known.
*/
                    STORAGE_TYPE_UNKNOWN = 0,

                    /**
Storage type is USB device.
*/
                    STORAGE_TYPE_USB_STICK = 1,

                    /**
Storage type is SD card.
*/
                    STORAGE_TYPE_SD = 2,

                    /**
Storage type is microSD card.
*/
                    STORAGE_TYPE_MICROSD = 3,

                    /**
Storage type is CFast.
*/
                    STORAGE_TYPE_CF = 4,

                    /**
Storage type is CFexpress.
*/
                    STORAGE_TYPE_CFE = 5,

                    /**
Storage type is XQD.
*/
                    STORAGE_TYPE_XQD = 6,

                    /**
Storage type is HD mass storage type.
*/
                    STORAGE_TYPE_HD = 7,

                    /**
Storage type is other, not listed type.
*/
                    STORAGE_TYPE_OTHER = 254,
                }

                /**
Flags to indicate usage for a particular storage (see STORAGE_INFORMATION.storage_usage and MAV_CMD_SET_STORAGE_USAGE).
*/
                enum STORAGE_USAGE_FLAG
                {
                    /**
Always set to 1 (indicates STORAGE_INFORMATION.storage_usage is supported).
*/
                    STORAGE_USAGE_FLAG_SET = 1,

                    /**
Storage for saving photos.
*/
                    STORAGE_USAGE_FLAG_PHOTO = 2,

                    /**
Storage for saving videos.
*/
                    STORAGE_USAGE_FLAG_VIDEO = 4,

                    /**
Storage for saving logs.
*/
                    STORAGE_USAGE_FLAG_LOGS = 8,
                }

                /**
Yaw behaviour during orbit flight.
*/
                enum ORBIT_YAW_BEHAVIOUR
                {
                    /**
Vehicle front points to the center (default).
*/
                    ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0,

                    /**
Vehicle front holds heading when message received.
*/
                    ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1,

                    /**
Yaw uncontrolled.
*/
                    ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2,

                    /**
Vehicle front follows flight path (tangential to circle).
*/
                    ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3,

                    /**
Yaw controlled by RC input.
*/
                    ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4,
                }

                /**
Possible responses from a WIFI_CONFIG_AP message.
*/
                enum WIFI_CONFIG_AP_RESPONSE
                {
                    /**
Undefined response. Likely an indicative of a system that doesn't support this request.
*/
                    WIFI_CONFIG_AP_RESPONSE_UNDEFINED = 0,

                    /**
Changes accepted.
*/
                    WIFI_CONFIG_AP_RESPONSE_ACCEPTED = 1,

                    /**
Changes rejected.
*/
                    WIFI_CONFIG_AP_RESPONSE_REJECTED = 2,

                    /**
Invalid Mode.
*/
                    WIFI_CONFIG_AP_RESPONSE_MODE_ERROR = 3,

                    /**
Invalid SSID.
*/
                    WIFI_CONFIG_AP_RESPONSE_SSID_ERROR = 4,

                    /**
Invalid Password.
*/
                    WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR = 5,
                }

                /**
Possible responses from a CELLULAR_CONFIG message.
*/
                enum CELLULAR_CONFIG_RESPONSE
                {
                    /**
Changes accepted.
*/
                    CELLULAR_CONFIG_RESPONSE_ACCEPTED = 0,

                    /**
Invalid APN.
*/
                    CELLULAR_CONFIG_RESPONSE_APN_ERROR = 1,

                    /**
Invalid PIN.
*/
                    CELLULAR_CONFIG_RESPONSE_PIN_ERROR = 2,

                    /**
Changes rejected.
*/
                    CELLULAR_CONFIG_RESPONSE_REJECTED = 3,

                    /**
PUK is required to unblock SIM card.
*/
                    CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED = 4,
                }

                /**
WiFi Mode.
*/
                enum WIFI_CONFIG_AP_MODE
                {
                    /**
WiFi mode is undefined.
*/
                    WIFI_CONFIG_AP_MODE_UNDEFINED = 0,

                    /**
WiFi configured as an access point.
*/
                    WIFI_CONFIG_AP_MODE_AP = 1,

                    /**
WiFi configured as a station connected to an existing local WiFi network.
*/
                    WIFI_CONFIG_AP_MODE_STATION = 2,

                    /**
WiFi disabled.
*/
                    WIFI_CONFIG_AP_MODE_DISABLED = 3,
                }

                /**
Supported component metadata types. These are used in the "general" metadata file returned by COMPONENT_INFORMATION
to provide information about supported metadata types. The types are not used directly in MAVLink messages.
*/
                enum COMP_METADATA_TYPE
                {
                    /**
General information about the component. General metadata includes information about other metadata types
supported by the component. Files of this type must be supported, and must be downloadable from vehicle
using a MAVLink FTP URI.
*/
                    COMP_METADATA_TYPE_GENERAL = 0,

                    /**
Parameter meta data.
*/
                    COMP_METADATA_TYPE_PARAMETER = 1,

                    /**
Meta data that specifies which commands and command parameters the vehicle supports. (WIP)
*/
                    COMP_METADATA_TYPE_COMMANDS = 2,

                    /**
Meta data that specifies external non-MAVLink peripherals.
*/
                    COMP_METADATA_TYPE_PERIPHERALS = 3,

                    /**
Meta data for the events interface.
*/
                    COMP_METADATA_TYPE_EVENTS = 4,

                    /**
Meta data for actuator configuration (motors, servos and vehicle geometry) and testing.
*/
                    COMP_METADATA_TYPE_ACTUATORS = 5,
                }

                /**
Actuator configuration, used to change a setting on an actuator. Component information metadata can be
used to know which outputs support which commands.
*/
                enum ACTUATOR_CONFIGURATION
                {
                    /**
Do nothing.
*/
                    ACTUATOR_CONFIGURATION_NONE = 0,

                    /**
Command the actuator to beep now.
*/
                    ACTUATOR_CONFIGURATION_BEEP = 1,

                    /**
Permanently set the actuator (ESC) to 3D mode (reversible thrust).
*/
                    ACTUATOR_CONFIGURATION_3D_MODE_ON = 2,

                    /**
Permanently set the actuator (ESC) to non 3D mode (non-reversible thrust).
*/
                    ACTUATOR_CONFIGURATION_3D_MODE_OFF = 3,

                    /**
Permanently set the actuator (ESC) to spin direction 1 (which can be clockwise or counter-clockwise).
*/
                    ACTUATOR_CONFIGURATION_SPIN_DIRECTION1 = 4,

                    /**
Permanently set the actuator (ESC) to spin direction 2 (opposite of direction 1).
*/
                    ACTUATOR_CONFIGURATION_SPIN_DIRECTION2 = 5,
                }

                /**
Actuator output function. Values greater or equal to 1000 are autopilot-specific.
*/
                enum ACTUATOR_OUTPUT_FUNCTION
                {
                    /**
No function (disabled).
*/
                    ACTUATOR_OUTPUT_FUNCTION_NONE = 0,

                    /**
Motor 1
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR1 = 1,

                    /**
Motor 2
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR2 = 2,

                    /**
Motor 3
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR3 = 3,

                    /**
Motor 4
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR4 = 4,

                    /**
Motor 5
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR5 = 5,

                    /**
Motor 6
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR6 = 6,

                    /**
Motor 7
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR7 = 7,

                    /**
Motor 8
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR8 = 8,

                    /**
Motor 9
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR9 = 9,

                    /**
Motor 10
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR10 = 10,

                    /**
Motor 11
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR11 = 11,

                    /**
Motor 12
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR12 = 12,

                    /**
Motor 13
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR13 = 13,

                    /**
Motor 14
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR14 = 14,

                    /**
Motor 15
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR15 = 15,

                    /**
Motor 16
*/
                    ACTUATOR_OUTPUT_FUNCTION_MOTOR16 = 16,

                    /**
Servo 1
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO1 = 33,

                    /**
Servo 2
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO2 = 34,

                    /**
Servo 3
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO3 = 35,

                    /**
Servo 4
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO4 = 36,

                    /**
Servo 5
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO5 = 37,

                    /**
Servo 6
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO6 = 38,

                    /**
Servo 7
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO7 = 39,

                    /**
Servo 8
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO8 = 40,

                    /**
Servo 9
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO9 = 41,

                    /**
Servo 10
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO10 = 42,

                    /**
Servo 11
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO11 = 43,

                    /**
Servo 12
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO12 = 44,

                    /**
Servo 13
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO13 = 45,

                    /**
Servo 14
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO14 = 46,

                    /**
Servo 15
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO15 = 47,

                    /**
Servo 16
*/
                    ACTUATOR_OUTPUT_FUNCTION_SERVO16 = 48,
                }

                /**
Enable axes that will be tuned via autotuning. Used in MAV_CMD_DO_AUTOTUNE_ENABLE.
*/
                enum AUTOTUNE_AXIS
                {
                    /**
Flight stack tunes axis according to its default settings.
*/
                    AUTOTUNE_AXIS_DEFAULT = 0,

                    /**
Autotune roll axis.
*/
                    AUTOTUNE_AXIS_ROLL = 1,

                    /**
Autotune pitch axis.
*/
                    AUTOTUNE_AXIS_PITCH = 2,

                    /**
Autotune yaw axis.
*/
                    AUTOTUNE_AXIS_YAW = 4,
                }

                /**
the recommended messages.
*/
                enum MAV_DATA_STREAM
                {
                    /**
Enable all data streams
*/
                    MAV_DATA_STREAM_ALL = 0,

                    /**
Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
*/
                    MAV_DATA_STREAM_RAW_SENSORS = 1,

                    /**
Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
*/
                    MAV_DATA_STREAM_EXTENDED_STATUS = 2,

                    /**
Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
*/
                    MAV_DATA_STREAM_RC_CHANNELS = 3,

                    /**
Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
*/
                    MAV_DATA_STREAM_RAW_CONTROLLER = 4,

                    /**
Enable LOCAL_POSITION, GLOBAL_POSITION_INT messages.
*/
                    MAV_DATA_STREAM_POSITION = 6,

                    /**
Dependent on the autopilot
*/
                    MAV_DATA_STREAM_EXTRA1 = 10,

                    /**
Dependent on the autopilot
*/
                    MAV_DATA_STREAM_EXTRA2 = 11,

                    /**
Dependent on the autopilot
*/
                    MAV_DATA_STREAM_EXTRA3 = 12,
                }

                /**
MAV_CMD_NAV_ROI).
*/
                enum MAV_ROI
                {
                    /**
No region of interest.
*/
                    MAV_ROI_NONE = 0,

                    /**
Point toward next waypoint, with optional pitch/roll/yaw offset.
*/
                    MAV_ROI_WPNEXT = 1,

                    /**
Point toward given waypoint.
*/
                    MAV_ROI_WPINDEX = 2,

                    /**
Point toward fixed location.
*/
                    MAV_ROI_LOCATION = 3,

                    /**
Point toward of given id.
*/
                    MAV_ROI_TARGET = 4,
                }

                /**
ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
*/
                enum MAV_CMD_ACK
                {
                    /**
Command / mission item is ok.
*/
                    MAV_CMD_ACK_OK = 0,

                    /**
Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
*/
                    MAV_CMD_ACK_ERR_FAIL = 1,

                    /**
The system is refusing to accept this command from this source / communication partner.
*/
                    MAV_CMD_ACK_ERR_ACCESS_DENIED = 2,

                    /**
Command or mission item is not supported, other commands would be accepted.
*/
                    MAV_CMD_ACK_ERR_NOT_SUPPORTED = 3,

                    /**
The coordinate frame of this command / mission item is not supported.
*/
                    MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4,

                    /**
The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this
system. This is a generic error, please use the more specific error messages below if possible.
*/
                    MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE = 5,

                    /**
The X or latitude value is out of range.
*/
                    MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE = 6,

                    /**
The Y or longitude value is out of range.
*/
                    MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE = 7,

                    /**
The Z or altitude value is out of range.
*/
                    MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE = 8,
                }

                /**
Specifies the datatype of a MAVLink parameter.
*/
                enum MAV_PARAM_TYPE
                {
                    /**
8-bit unsigned integer
*/
                    MAV_PARAM_TYPE_UINT8 = 1,

                    /**
8-bit signed integer
*/
                    MAV_PARAM_TYPE_INT8 = 2,

                    /**
16-bit unsigned integer
*/
                    MAV_PARAM_TYPE_UINT16 = 3,

                    /**
16-bit signed integer
*/
                    MAV_PARAM_TYPE_INT16 = 4,

                    /**
32-bit unsigned integer
*/
                    MAV_PARAM_TYPE_UINT32 = 5,

                    /**
32-bit signed integer
*/
                    MAV_PARAM_TYPE_INT32 = 6,

                    /**
64-bit unsigned integer
*/
                    MAV_PARAM_TYPE_UINT64 = 7,

                    /**
64-bit signed integer
*/
                    MAV_PARAM_TYPE_INT64 = 8,

                    /**
32-bit floating-point
*/
                    MAV_PARAM_TYPE_REAL32 = 9,

                    /**
64-bit floating-point
*/
                    MAV_PARAM_TYPE_REAL64 = 10,
                }

                /**
Specifies the datatype of a MAVLink extended parameter.
*/
                enum MAV_PARAM_EXT_TYPE
                {
                    /**
8-bit unsigned integer
*/
                    MAV_PARAM_EXT_TYPE_UINT8 = 1,

                    /**
8-bit signed integer
*/
                    MAV_PARAM_EXT_TYPE_INT8 = 2,

                    /**
16-bit unsigned integer
*/
                    MAV_PARAM_EXT_TYPE_UINT16 = 3,

                    /**
16-bit signed integer
*/
                    MAV_PARAM_EXT_TYPE_INT16 = 4,

                    /**
32-bit unsigned integer
*/
                    MAV_PARAM_EXT_TYPE_UINT32 = 5,

                    /**
32-bit signed integer
*/
                    MAV_PARAM_EXT_TYPE_INT32 = 6,

                    /**
64-bit unsigned integer
*/
                    MAV_PARAM_EXT_TYPE_UINT64 = 7,

                    /**
64-bit signed integer
*/
                    MAV_PARAM_EXT_TYPE_INT64 = 8,

                    /**
32-bit floating-point
*/
                    MAV_PARAM_EXT_TYPE_REAL32 = 9,

                    /**
64-bit floating-point
*/
                    MAV_PARAM_EXT_TYPE_REAL64 = 10,

                    /**
Custom Type
*/
                    MAV_PARAM_EXT_TYPE_CUSTOM = 11,
                }

                /**
Result from a MAVLink command (MAV_CMD)
*/
                enum MAV_RESULT
                {
                    /**
Command is valid (is supported and has valid parameters), and was executed.
*/
                    MAV_RESULT_ACCEPTED = 0,

                    /**
Command is valid, but cannot be executed at this time. This is used to indicate a problem that should
be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.).
Retrying later should work.
*/
                    MAV_RESULT_TEMPORARILY_REJECTED = 1,

                    /**
Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will
not work.
*/
                    MAV_RESULT_DENIED = 2,

                    /**
Command is not supported (unknown).
*/
                    MAV_RESULT_UNSUPPORTED = 3,

                    /**
Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem,
i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting
to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.
*/
                    MAV_RESULT_FAILED = 4,

                    /**
Command is valid and is being executed. This will be followed by further progress updates, i.e. the component
may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation),
and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress
field can be used to indicate the progress of the operation.
*/
                    MAV_RESULT_IN_PROGRESS = 5,

                    /**
Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).
*/
                    MAV_RESULT_CANCELLED = 6,
                }

                /**
Result of mission operation (in a MISSION_ACK message).
*/
                enum MAV_MISSION_RESULT
                {
                    /**
mission accepted OK
*/
                    MAV_MISSION_ACCEPTED = 0,

                    /**
Generic error / not accepting mission commands at all right now.
*/
                    MAV_MISSION_ERROR = 1,

                    /**
Coordinate frame is not supported.
*/
                    MAV_MISSION_UNSUPPORTED_FRAME = 2,

                    /**
Command is not supported.
*/
                    MAV_MISSION_UNSUPPORTED = 3,

                    /**
Mission items exceed storage space.
*/
                    MAV_MISSION_NO_SPACE = 4,

                    /**
One of the parameters has an invalid value.
*/
                    MAV_MISSION_INVALID = 5,

                    /**
param1 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM1 = 6,

                    /**
param2 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM2 = 7,

                    /**
param3 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM3 = 8,

                    /**
param4 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM4 = 9,

                    /**
x / param5 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM5_X = 10,

                    /**
y / param6 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM6_Y = 11,

                    /**
z / param7 has an invalid value.
*/
                    MAV_MISSION_INVALID_PARAM7 = 12,

                    /**
Mission item received out of sequence
*/
                    MAV_MISSION_INVALID_SEQUENCE = 13,

                    /**
Not accepting any mission commands from this communication partner.
*/
                    MAV_MISSION_DENIED = 14,

                    /**
Current mission operation cancelled (e.g. mission upload, mission download).
*/
                    MAV_MISSION_OPERATION_CANCELLED = 15,
                }

                /**
Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
*/
                enum MAV_SEVERITY
                {
                    /**
System is unusable. This is a "panic" condition.
*/
                    MAV_SEVERITY_EMERGENCY = 0,

                    /**
Action should be taken immediately. Indicates error in non-critical systems.
*/
                    MAV_SEVERITY_ALERT = 1,

                    /**
Action must be taken immediately. Indicates failure in a primary system.
*/
                    MAV_SEVERITY_CRITICAL = 2,

                    /**
Indicates an error in secondary/redundant systems.
*/
                    MAV_SEVERITY_ERROR = 3,

                    /**
Indicates about a possible future error if this is not resolved within a given timeframe. Example would
be a low battery warning.
*/
                    MAV_SEVERITY_WARNING = 4,

                    /**
An unusual event has occurred, though not an error condition. This should be investigated for the root
cause.
*/
                    MAV_SEVERITY_NOTICE = 5,

                    /**
Normal operational messages. Useful for logging. No action is required for these messages.
*/
                    MAV_SEVERITY_INFO = 6,

                    /**
Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
*/
                    MAV_SEVERITY_DEBUG = 7,
                }

                /**
Power supply status flags (bitmask)
*/
                enum MAV_POWER_STATUS
                {
                    /**
main brick power supply valid
*/
                    MAV_POWER_STATUS_BRICK_VALID = 1,

                    /**
main servo power supply valid for FMU
*/
                    MAV_POWER_STATUS_SERVO_VALID = 2,

                    /**
USB power is connected
*/
                    MAV_POWER_STATUS_USB_CONNECTED = 4,

                    /**
peripheral supply is in over-current state
*/
                    MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,

                    /**
hi-power peripheral supply is in over-current state
*/
                    MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,

                    /**
Power status has changed since boot
*/
                    MAV_POWER_STATUS_CHANGED = 32,
                }

                /**
SERIAL_CONTROL device types
*/
                enum SERIAL_CONTROL_DEV
                {
                    /**
First telemetry port
*/
                    SERIAL_CONTROL_DEV_TELEM1 = 0,

                    /**
Second telemetry port
*/
                    SERIAL_CONTROL_DEV_TELEM2 = 1,

                    /**
First GPS port
*/
                    SERIAL_CONTROL_DEV_GPS1 = 2,

                    /**
Second GPS port
*/
                    SERIAL_CONTROL_DEV_GPS2 = 3,

                    /**
system shell
*/
                    SERIAL_CONTROL_DEV_SHELL = 10,

                    /**
SERIAL0
*/
                    SERIAL_CONTROL_SERIAL0 = 100,

                    /**
SERIAL1
*/
                    SERIAL_CONTROL_SERIAL1 = 101,

                    /**
SERIAL2
*/
                    SERIAL_CONTROL_SERIAL2 = 102,

                    /**
SERIAL3
*/
                    SERIAL_CONTROL_SERIAL3 = 103,

                    /**
SERIAL4
*/
                    SERIAL_CONTROL_SERIAL4 = 104,

                    /**
SERIAL5
*/
                    SERIAL_CONTROL_SERIAL5 = 105,

                    /**
SERIAL6
*/
                    SERIAL_CONTROL_SERIAL6 = 106,

                    /**
SERIAL7
*/
                    SERIAL_CONTROL_SERIAL7 = 107,

                    /**
SERIAL8
*/
                    SERIAL_CONTROL_SERIAL8 = 108,

                    /**
SERIAL9
*/
                    SERIAL_CONTROL_SERIAL9 = 109,
                }

                /**
SERIAL_CONTROL flags (bitmask)
*/
                enum SERIAL_CONTROL_FLAG
                {
                    /**
Set if this is a reply
*/
                    SERIAL_CONTROL_FLAG_REPLY = 1,

                    /**
Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
*/
                    SERIAL_CONTROL_FLAG_RESPOND = 2,

                    /**
Set if access to the serial port should be removed from whatever driver is currently using it, giving
exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
this flag set
*/
                    SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,

                    /**
Block on writes to the serial port
*/
                    SERIAL_CONTROL_FLAG_BLOCKING = 8,

                    /**
Send multiple replies until port is drained
*/
                    SERIAL_CONTROL_FLAG_MULTI = 16,
                }

                /**
Enumeration of distance sensor types
*/
                enum MAV_DISTANCE_SENSOR
                {
                    /**
Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
*/
                    MAV_DISTANCE_SENSOR_LASER = 0,

                    /**
Ultrasound rangefinder, e.g. MaxBotix units
*/
                    MAV_DISTANCE_SENSOR_ULTRASOUND = 1,

                    /**
Infrared rangefinder, e.g. Sharp units
*/
                    MAV_DISTANCE_SENSOR_INFRARED = 2,

                    /**
Radar type, e.g. uLanding units
*/
                    MAV_DISTANCE_SENSOR_RADAR = 3,

                    /**
Broken or unknown type, e.g. analog units
*/
                    MAV_DISTANCE_SENSOR_UNKNOWN = 4,
                }

                /**
Enumeration of sensor orientation, according to its rotations
*/
                enum MAV_SENSOR_ORIENTATION
                {
                    /**
Roll: 0, Pitch: 0, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_NONE = 0,

                    /**
Roll: 0, Pitch: 0, Yaw: 45
*/
                    MAV_SENSOR_ROTATION_YAW_45 = 1,

                    /**
Roll: 0, Pitch: 0, Yaw: 90
*/
                    MAV_SENSOR_ROTATION_YAW_90 = 2,

                    /**
Roll: 0, Pitch: 0, Yaw: 135
*/
                    MAV_SENSOR_ROTATION_YAW_135 = 3,

                    /**
Roll: 0, Pitch: 0, Yaw: 180
*/
                    MAV_SENSOR_ROTATION_YAW_180 = 4,

                    /**
Roll: 0, Pitch: 0, Yaw: 225
*/
                    MAV_SENSOR_ROTATION_YAW_225 = 5,

                    /**
Roll: 0, Pitch: 0, Yaw: 270
*/
                    MAV_SENSOR_ROTATION_YAW_270 = 6,

                    /**
Roll: 0, Pitch: 0, Yaw: 315
*/
                    MAV_SENSOR_ROTATION_YAW_315 = 7,

                    /**
Roll: 180, Pitch: 0, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_180 = 8,

                    /**
Roll: 180, Pitch: 0, Yaw: 45
*/
                    MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,

                    /**
Roll: 180, Pitch: 0, Yaw: 90
*/
                    MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,

                    /**
Roll: 180, Pitch: 0, Yaw: 135
*/
                    MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,

                    /**
Roll: 0, Pitch: 180, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_PITCH_180 = 12,

                    /**
Roll: 180, Pitch: 0, Yaw: 225
*/
                    MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,

                    /**
Roll: 180, Pitch: 0, Yaw: 270
*/
                    MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,

                    /**
Roll: 180, Pitch: 0, Yaw: 315
*/
                    MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,

                    /**
Roll: 90, Pitch: 0, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_90 = 16,

                    /**
Roll: 90, Pitch: 0, Yaw: 45
*/
                    MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,

                    /**
Roll: 90, Pitch: 0, Yaw: 90
*/
                    MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,

                    /**
Roll: 90, Pitch: 0, Yaw: 135
*/
                    MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,

                    /**
Roll: 270, Pitch: 0, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_270 = 20,

                    /**
Roll: 270, Pitch: 0, Yaw: 45
*/
                    MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,

                    /**
Roll: 270, Pitch: 0, Yaw: 90
*/
                    MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,

                    /**
Roll: 270, Pitch: 0, Yaw: 135
*/
                    MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,

                    /**
Roll: 0, Pitch: 90, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_PITCH_90 = 24,

                    /**
Roll: 0, Pitch: 270, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_PITCH_270 = 25,

                    /**
Roll: 0, Pitch: 180, Yaw: 90
*/
                    MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,

                    /**
Roll: 0, Pitch: 180, Yaw: 270
*/
                    MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,

                    /**
Roll: 90, Pitch: 90, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,

                    /**
Roll: 180, Pitch: 90, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,

                    /**
Roll: 270, Pitch: 90, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,

                    /**
Roll: 90, Pitch: 180, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,

                    /**
Roll: 270, Pitch: 180, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,

                    /**
Roll: 90, Pitch: 270, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,

                    /**
Roll: 180, Pitch: 270, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,

                    /**
Roll: 270, Pitch: 270, Yaw: 0
*/
                    MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,

                    /**
Roll: 90, Pitch: 180, Yaw: 90
*/
                    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,

                    /**
Roll: 90, Pitch: 0, Yaw: 270
*/
                    MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,

                    /**
Roll: 90, Pitch: 68, Yaw: 293
*/
                    MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,

                    /**
Pitch: 315
*/
                    MAV_SENSOR_ROTATION_PITCH_315 = 39,

                    /**
Roll: 90, Pitch: 315
*/
                    MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 = 40,

                    /**
Custom orientation
*/
                    MAV_SENSOR_ROTATION_CUSTOM = 100,
                }

                /**
Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
*/
                enum MAV_PROTOCOL_CAPABILITY
                {
                    /**
Autopilot supports the MISSION_ITEM float message type.
          Note that MISSION_ITEM is deprecated,
and autopilots should use MISSION_INT instead.
*/
                    MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,

                    /**
Parameter protocol uses C-cast of parameter values to set the param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.

         Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if
the parameter protocol is supported.
*/
                    MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST = 2,

                    /**
Autopilot supports MISSION_ITEM_INT scaled integer message type.
          Note that this flag must always
be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM,
which is deprecated).
*/
                    MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,

                    /**
Autopilot supports COMMAND_INT scaled integer message type.
*/
                    MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,

                    /**
Parameter protocol uses byte-wise encoding of parameter values into param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.

         Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if
the parameter protocol is supported.
*/
                    MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE = 16,

                    /**
Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html.
*/
                    MAV_PROTOCOL_CAPABILITY_FTP = 32,

                    /**
Autopilot supports commanding attitude offboard.
*/
                    MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,

                    /**
Autopilot supports commanding position and velocity targets in local NED frame.
*/
                    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,

                    /**
Autopilot supports commanding position and velocity targets in global scaled integers.
*/
                    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,

                    /**
Autopilot supports terrain protocol / data handling.
*/
                    MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,

                    /**
Autopilot supports direct actuator control.
*/
                    MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024,

                    /**
Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination).
*/
                    MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,

                    /**
Autopilot supports onboard compass calibration.
*/
                    MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,

                    /**
Autopilot supports MAVLink version 2.
*/
                    MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192,

                    /**
Autopilot supports mission fence protocol.
*/
                    MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384,

                    /**
Autopilot supports mission rally point protocol.
*/
                    MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768,

                    /**
Reserved for future use.
*/
                    MAV_PROTOCOL_CAPABILITY_RESERVED2 = 65536,
                }

                /**
Type of mission items being requested/sent in mission protocol.
*/
                enum MAV_MISSION_TYPE
                {
                    /**
Items are mission commands for main mission.
*/
                    MAV_MISSION_TYPE_MISSION = 0,

                    /**
Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
*/
                    MAV_MISSION_TYPE_FENCE = 1,

                    /**
Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT
rally point items.
*/
                    MAV_MISSION_TYPE_RALLY = 2,

                    /**
Only used in MISSION_CLEAR_ALL to clear all mission types.
*/
                    MAV_MISSION_TYPE_ALL = 255,
                }

                /**
Enumeration of estimator types
*/
                enum MAV_ESTIMATOR_TYPE
                {
                    /**
Unknown type of the estimator.
*/
                    MAV_ESTIMATOR_TYPE_UNKNOWN = 0,

                    /**
This is a naive estimator without any real covariance feedback.
*/
                    MAV_ESTIMATOR_TYPE_NAIVE = 1,

                    /**
Computer vision based estimate. Might be up to scale.
*/
                    MAV_ESTIMATOR_TYPE_VISION = 2,

                    /**
Visual-inertial estimate.
*/
                    MAV_ESTIMATOR_TYPE_VIO = 3,

                    /**
Plain GPS estimate.
*/
                    MAV_ESTIMATOR_TYPE_GPS = 4,

                    /**
Estimator integrating GPS and inertial sensing.
*/
                    MAV_ESTIMATOR_TYPE_GPS_INS = 5,

                    /**
Estimate from external motion capturing system.
*/
                    MAV_ESTIMATOR_TYPE_MOCAP = 6,

                    /**
Estimator based on lidar sensor input.
*/
                    MAV_ESTIMATOR_TYPE_LIDAR = 7,

                    /**
Estimator on autopilot.
*/
                    MAV_ESTIMATOR_TYPE_AUTOPILOT = 8,
                }

                /**
Enumeration of battery types
*/
                enum MAV_BATTERY_TYPE
                {
                    /**
Not specified.
*/
                    MAV_BATTERY_TYPE_UNKNOWN = 0,

                    /**
Lithium polymer battery
*/
                    MAV_BATTERY_TYPE_LIPO = 1,

                    /**
Lithium-iron-phosphate battery
*/
                    MAV_BATTERY_TYPE_LIFE = 2,

                    /**
Lithium-ION battery
*/
                    MAV_BATTERY_TYPE_LION = 3,

                    /**
Nickel metal hydride battery
*/
                    MAV_BATTERY_TYPE_NIMH = 4,
                }

                /**
Enumeration of battery functions
*/
                enum MAV_BATTERY_FUNCTION
                {
                    /**
Battery function is unknown
*/
                    MAV_BATTERY_FUNCTION_UNKNOWN = 0,

                    /**
Battery supports all flight systems
*/
                    MAV_BATTERY_FUNCTION_ALL = 1,

                    /**
Battery for the propulsion system
*/
                    MAV_BATTERY_FUNCTION_PROPULSION = 2,

                    /**
Avionics battery
*/
                    MAV_BATTERY_FUNCTION_AVIONICS = 3,

                    /**
Payload battery
*/
                    MAV_BATTERY_TYPE_PAYLOAD = 4,
                }

                /**
Enumeration for battery charge states.
*/
                enum MAV_BATTERY_CHARGE_STATE
                {
                    /**
Low battery state is not provided
*/
                    MAV_BATTERY_CHARGE_STATE_UNDEFINED = 0,

                    /**
Battery is not in low state. Normal operation.
*/
                    MAV_BATTERY_CHARGE_STATE_OK = 1,

                    /**
Battery state is low, warn and monitor close.
*/
                    MAV_BATTERY_CHARGE_STATE_LOW = 2,

                    /**
Battery state is critical, return or abort immediately.
*/
                    MAV_BATTERY_CHARGE_STATE_CRITICAL = 3,

                    /**
Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent
damage.
*/
                    MAV_BATTERY_CHARGE_STATE_EMERGENCY = 4,

                    /**
Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT.
*/
                    MAV_BATTERY_CHARGE_STATE_FAILED = 5,

                    /**
Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible
causes (faults) are listed in MAV_BATTERY_FAULT.
*/
                    MAV_BATTERY_CHARGE_STATE_UNHEALTHY = 6,

                    /**
Battery is charging.
*/
                    MAV_BATTERY_CHARGE_STATE_CHARGING = 7,
                }

                /**
Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN
to allow message trimming in normal flight.
*/
                enum MAV_BATTERY_MODE
                {
                    /**
Battery mode not supported/unknown battery mode/normal operation.
*/
                    MAV_BATTERY_MODE_UNKNOWN = 0,

                    /**
Battery is auto discharging (towards storage level).
*/
                    MAV_BATTERY_MODE_AUTO_DISCHARGING = 1,

                    /**
Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits).
*/
                    MAV_BATTERY_MODE_HOT_SWAP = 2,
                }

                /**
Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report
either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set.
*/
                enum MAV_BATTERY_FAULT
                {
                    /**
Battery has deep discharged.
*/
                    MAV_BATTERY_FAULT_DEEP_DISCHARGE = 1,

                    /**
Voltage spikes.
*/
                    MAV_BATTERY_FAULT_SPIKES = 2,

                    /**
One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not
be used).
*/
                    MAV_BATTERY_FAULT_CELL_FAIL = 4,

                    /**
Over-current fault.
*/
                    MAV_BATTERY_FAULT_OVER_CURRENT = 8,

                    /**
Over-temperature fault.
*/
                    MAV_BATTERY_FAULT_OVER_TEMPERATURE = 16,

                    /**
Under-temperature fault.
*/
                    MAV_BATTERY_FAULT_UNDER_TEMPERATURE = 32,

                    /**
Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar
voltage).
*/
                    MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE = 64,

                    /**
Battery firmware is not compatible with current autopilot firmware.
*/
                    MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE = 128,

                    /**
Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).
*/
                    BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION = 256,
                }

                /**
Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS
are conditions that cause the generator to fail. Warnings are conditions that require attention before
the next use (they indicate the system is not operating properly).
*/
                enum MAV_GENERATOR_STATUS_FLAG
                {
                    /**
Generator is off.
*/
                    MAV_GENERATOR_STATUS_FLAG_OFF = 1,

                    /**
Generator is ready to start generating power.
*/
                    MAV_GENERATOR_STATUS_FLAG_READY = 2,

                    /**
Generator is generating power.
*/
                    MAV_GENERATOR_STATUS_FLAG_GENERATING = 4,

                    /**
Generator is charging the batteries (generating enough power to charge and provide the load).
*/
                    MAV_GENERATOR_STATUS_FLAG_CHARGING = 8,

                    /**
Generator is operating at a reduced maximum power.
*/
                    MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER = 16,

                    /**
Generator is providing the maximum output.
*/
                    MAV_GENERATOR_STATUS_FLAG_MAXPOWER = 32,

                    /**
Generator is near the maximum operating temperature, cooling is insufficient.
*/
                    MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING = 64,

                    /**
Generator hit the maximum operating temperature and shutdown.
*/
                    MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT = 128,

                    /**
Power electronics are near the maximum operating temperature, cooling is insufficient.
*/
                    MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING = 256,

                    /**
Power electronics hit the maximum operating temperature and shutdown.
*/
                    MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT = 512,

                    /**
Power electronics experienced a fault and shutdown.
*/
                    MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT = 1024,

                    /**
The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer
providing power, solar cell is in shade, hydrogen reaction no longer happening.
*/
                    MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT = 2048,

                    /**
Generator controller having communication problems.
*/
                    MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING = 4096,

                    /**
Power electronic or generator cooling system error.
*/
                    MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING = 8192,

                    /**
Generator controller power rail experienced a fault.
*/
                    MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT = 16384,

                    /**
Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.
*/
                    MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT = 32768,

                    /**
Generator controller detected a high current going into the batteries and shutdown to prevent battery
damage.
*/
                    MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT = 65536,

                    /**
Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage
rating.
*/
                    MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT = 131072,

                    /**
Batteries are under voltage (generator will not start).
*/
                    MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT = 262144,

                    /**
Generator start is inhibited by e.g. a safety switch.
*/
                    MAV_GENERATOR_STATUS_FLAG_START_INHIBITED = 524288,

                    /**
Generator requires maintenance.
*/
                    MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED = 1048576,

                    /**
Generator is not ready to generate yet.
*/
                    MAV_GENERATOR_STATUS_FLAG_WARMING_UP = 2097152,

                    /**
Generator is idle.
*/
                    MAV_GENERATOR_STATUS_FLAG_IDLE = 4194304,
                }

                /**
Enumeration of VTOL states
*/
                enum MAV_VTOL_STATE
                {
                    /**
MAV is not configured as VTOL
*/
                    MAV_VTOL_STATE_UNDEFINED = 0,

                    /**
VTOL is in transition from multicopter to fixed-wing
*/
                    MAV_VTOL_STATE_TRANSITION_TO_FW = 1,

                    /**
VTOL is in transition from fixed-wing to multicopter
*/
                    MAV_VTOL_STATE_TRANSITION_TO_MC = 2,

                    /**
VTOL is in multicopter state
*/
                    MAV_VTOL_STATE_MC = 3,

                    /**
VTOL is in fixed-wing state
*/
                    MAV_VTOL_STATE_FW = 4,
                }

                /**
Enumeration of landed detector states
*/
                enum MAV_LANDED_STATE
                {
                    /**
MAV landed state is unknown
*/
                    MAV_LANDED_STATE_UNDEFINED = 0,

                    /**
MAV is landed (on ground)
*/
                    MAV_LANDED_STATE_ON_GROUND = 1,

                    /**
MAV is in air
*/
                    MAV_LANDED_STATE_IN_AIR = 2,

                    /**
MAV currently taking off
*/
                    MAV_LANDED_STATE_TAKEOFF = 3,

                    /**
MAV currently landing
*/
                    MAV_LANDED_STATE_LANDING = 4,
                }

                /**
Enumeration of the ADSB altimeter types
*/
                enum ADSB_ALTITUDE_TYPE
                {
                    /**
Altitude reported from a Baro source using QNH reference
*/
                    ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,

                    /**
Altitude reported from a GNSS source
*/
                    ADSB_ALTITUDE_TYPE_GEOMETRIC = 1,
                }

                /**
ADSB classification for the type of vehicle emitting the transponder signal
*/
                enum ADSB_EMITTER_TYPE
                {
                    ADSB_EMITTER_TYPE_NO_INFO           = 0,
                    ADSB_EMITTER_TYPE_LIGHT             = 1,
                    ADSB_EMITTER_TYPE_SMALL             = 2,
                    ADSB_EMITTER_TYPE_LARGE             = 3,
                    ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
                    ADSB_EMITTER_TYPE_HEAVY             = 5,
                    ADSB_EMITTER_TYPE_HIGHLY_MANUV      = 6,
                    ADSB_EMITTER_TYPE_ROTOCRAFT         = 7,
                    ADSB_EMITTER_TYPE_UNASSIGNED        = 8,
                    ADSB_EMITTER_TYPE_GLIDER            = 9,
                    ADSB_EMITTER_TYPE_LIGHTER_AIR       = 10,
                    ADSB_EMITTER_TYPE_PARACHUTE         = 11,
                    ADSB_EMITTER_TYPE_ULTRA_LIGHT       = 12,
                    ADSB_EMITTER_TYPE_UNASSIGNED2       = 13,
                    ADSB_EMITTER_TYPE_UAV               = 14,
                    ADSB_EMITTER_TYPE_SPACE             = 15,
                    ADSB_EMITTER_TYPE_UNASSGINED3       = 16,
                    ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
                    ADSB_EMITTER_TYPE_SERVICE_SURFACE   = 18,
                    ADSB_EMITTER_TYPE_POINT_OBSTACLE    = 19,
                }

                /**
These flags indicate status such as data validity of each data source. Set = data valid
*/
                enum ADSB_FLAGS
                {
                    ADSB_FLAGS_VALID_COORDS            = 1,
                    ADSB_FLAGS_VALID_ALTITUDE          = 2,
                    ADSB_FLAGS_VALID_HEADING           = 4,
                    ADSB_FLAGS_VALID_VELOCITY          = 8,
                    ADSB_FLAGS_VALID_CALLSIGN          = 16,
                    ADSB_FLAGS_VALID_SQUAWK            = 32,
                    ADSB_FLAGS_SIMULATED               = 64,
                    ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128,
                    ADSB_FLAGS_BARO_VALID              = 256,
                    ADSB_FLAGS_SOURCE_UAT              = 32768,
                }

                /**
Bitmap of options for the MAV_CMD_DO_REPOSITION
*/
                enum MAV_DO_REPOSITION_FLAGS
                {
                    /**
The aircraft should immediately transition into guided. This should not be set for follow me applications
*/
                    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1,
                }

                /**
Flags in ESTIMATOR_STATUS message
*/
                enum ESTIMATOR_STATUS_FLAGS
                {
                    /**
True if the attitude estimate is good
*/
                    ESTIMATOR_ATTITUDE = 1,

                    /**
True if the horizontal velocity estimate is good
*/
                    ESTIMATOR_VELOCITY_HORIZ = 2,

                    /**
True if the  vertical velocity estimate is good
*/
                    ESTIMATOR_VELOCITY_VERT = 4,

                    /**
True if the horizontal position (relative) estimate is good
*/
                    ESTIMATOR_POS_HORIZ_REL = 8,

                    /**
True if the horizontal position (absolute) estimate is good
*/
                    ESTIMATOR_POS_HORIZ_ABS = 16,

                    /**
True if the vertical position (absolute) estimate is good
*/
                    ESTIMATOR_POS_VERT_ABS = 32,

                    /**
True if the vertical position (above ground) estimate is good
*/
                    ESTIMATOR_POS_VERT_AGL = 64,

                    /**
True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
flow)
*/
                    ESTIMATOR_CONST_POS_MODE = 128,

                    /**
True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
*/
                    ESTIMATOR_PRED_POS_HORIZ_REL = 256,

                    /**
True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
*/
                    ESTIMATOR_PRED_POS_HORIZ_ABS = 512,

                    /**
True if the EKF has detected a GPS glitch
*/
                    ESTIMATOR_GPS_GLITCH = 1024,

                    /**
True if the EKF has detected bad accelerometer data
*/
                    ESTIMATOR_ACCEL_ERROR = 2048,
                }

                /**
Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST.
*/
                enum MOTOR_TEST_ORDER
                {
                    /**
Default autopilot motor test method.
*/
                    MOTOR_TEST_ORDER_DEFAULT = 0,

                    /**
Motor numbers are specified as their index in a predefined vehicle-specific sequence.
*/
                    MOTOR_TEST_ORDER_SEQUENCE = 1,

                    /**
Motor numbers are specified as the output as labeled on the board.
*/
                    MOTOR_TEST_ORDER_BOARD = 2,
                }

                /**
Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST.
*/
                enum MOTOR_TEST_THROTTLE_TYPE
                {
                    /**
Throttle as a percentage (0 ~ 100)
*/
                    MOTOR_TEST_THROTTLE_PERCENT = 0,

                    /**
Throttle as an absolute PWM value (normally in range of 1000~2000).
*/
                    MOTOR_TEST_THROTTLE_PWM = 1,

                    /**
Throttle pass-through from pilot's transmitter.
*/
                    MOTOR_TEST_THROTTLE_PILOT = 2,

                    /**
Per-motor compass calibration test.
*/
                    MOTOR_TEST_COMPASS_CAL = 3,
                }

                enum GPS_INPUT_IGNORE_FLAGS
                {
                    /**
ignore altitude field
*/
                    GPS_INPUT_IGNORE_FLAG_ALT = 1,

                    /**
ignore hdop field
*/
                    GPS_INPUT_IGNORE_FLAG_HDOP = 2,

                    /**
ignore vdop field
*/
                    GPS_INPUT_IGNORE_FLAG_VDOP = 4,

                    /**
ignore horizontal velocity field (vn and ve)
*/
                    GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,

                    /**
ignore vertical velocity field (vd)
*/
                    GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,

                    /**
ignore speed accuracy field
*/
                    GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,

                    /**
ignore horizontal accuracy field
*/
                    GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,

                    /**
ignore vertical accuracy field
*/
                    GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128,
                }

                /**
Possible actions an aircraft can take to avoid a collision.
*/
                enum MAV_COLLISION_ACTION
                {
                    /**
Ignore any potential collisions
*/
                    MAV_COLLISION_ACTION_NONE = 0,

                    /**
Report potential collision
*/
                    MAV_COLLISION_ACTION_REPORT = 1,

                    /**
Ascend or Descend to avoid threat
*/
                    MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,

                    /**
Move horizontally to avoid threat
*/
                    MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,

                    /**
Aircraft to move perpendicular to the collision's velocity vector
*/
                    MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,

                    /**
Aircraft to fly directly back to its launch point
*/
                    MAV_COLLISION_ACTION_RTL = 5,

                    /**
Aircraft to stop in place
*/
                    MAV_COLLISION_ACTION_HOVER = 6,
                }

                /**
Aircraft-rated danger from this threat.
*/
                enum MAV_COLLISION_THREAT_LEVEL
                {
                    /**
Not a threat
*/
                    MAV_COLLISION_THREAT_LEVEL_NONE = 0,

                    /**
Craft is mildly concerned about this threat
*/
                    MAV_COLLISION_THREAT_LEVEL_LOW = 1,

                    /**
Craft is panicking, and may take actions to avoid threat
*/
                    MAV_COLLISION_THREAT_LEVEL_HIGH = 2,
                }

                /**
Source of information about this collision.
*/
                enum MAV_COLLISION_SRC
                {
                    /**
ID field references ADSB_VEHICLE packets
*/
                    MAV_COLLISION_SRC_ADSB = 0,

                    /**
ID field references MAVLink SRC ID
*/
                    MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1,
                }

                /**
Type of GPS fix
*/
                enum GPS_FIX_TYPE
                {
                    /**
No GPS connected
*/
                    GPS_FIX_TYPE_NO_GPS = 0,

                    /**
No position information, GPS is connected
*/
                    GPS_FIX_TYPE_NO_FIX = 1,

                    /**
2D position
*/
                    GPS_FIX_TYPE_2D_FIX = 2,

                    /**
3D position
*/
                    GPS_FIX_TYPE_3D_FIX = 3,

                    /**
DGPS/SBAS aided 3D position
*/
                    GPS_FIX_TYPE_DGPS = 4,

                    /**
RTK float, 3D position
*/
                    GPS_FIX_TYPE_RTK_FLOAT = 5,

                    /**
RTK Fixed, 3D position
*/
                    GPS_FIX_TYPE_RTK_FIXED = 6,

                    /**
Static fixed, typically used for base stations
*/
                    GPS_FIX_TYPE_STATIC = 7,

                    /**
PPP, 3D position.
*/
                    GPS_FIX_TYPE_PPP = 8,
                }

                /**
RTK GPS baseline coordinate system, used for RTK corrections
*/
                enum RTK_BASELINE_COORDINATE_SYSTEM
                {
                    /**
Earth-centered, Earth-fixed
*/
                    RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0,

                    /**
RTK basestation centered, north, east, down
*/
                    RTK_BASELINE_COORDINATE_SYSTEM_NED = 1,
                }

                /**
Type of landing target
*/
                enum LANDING_TARGET_TYPE
                {
                    /**
Landing target signaled by light beacon (ex: IR-LOCK)
*/
                    LANDING_TARGET_TYPE_LIGHT_BEACON = 0,

                    /**
Landing target signaled by radio beacon (ex: ILS, NDB)
*/
                    LANDING_TARGET_TYPE_RADIO_BEACON = 1,

                    /**
Landing target represented by a fiducial marker (ex: ARTag)
*/
                    LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,

                    /**
Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
*/
                    LANDING_TARGET_TYPE_VISION_OTHER = 3,
                }

                /**
Direction of VTOL transition
*/
                enum VTOL_TRANSITION_HEADING
                {
                    /**
Respect the heading configuration of the vehicle.
*/
                    VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT = 0,

                    /**
Use the heading pointing towards the next waypoint.
*/
                    VTOL_TRANSITION_HEADING_NEXT_WAYPOINT = 1,

                    /**
Use the heading on takeoff (while sitting on the ground).
*/
                    VTOL_TRANSITION_HEADING_TAKEOFF = 2,

                    /**
Use the specified heading in parameter 4.
*/
                    VTOL_TRANSITION_HEADING_SPECIFIED = 3,

                    /**
Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning
is active).
*/
                    VTOL_TRANSITION_HEADING_ANY = 4,
                }

                /**
Camera capability flags (Bitmap)
*/
                enum CAMERA_CAP_FLAGS
                {
                    /**
Camera is able to record video
*/
                    CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1,

                    /**
Camera is able to capture images
*/
                    CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2,

                    /**
Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
*/
                    CAMERA_CAP_FLAGS_HAS_MODES = 4,

                    /**
Camera can capture images while in video mode
*/
                    CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,

                    /**
Camera can capture videos while in Photo/Image mode
*/
                    CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,

                    /**
Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
*/
                    CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32,

                    /**
Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
*/
                    CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM = 64,

                    /**
Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
*/
                    CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS = 128,

                    /**
Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE
for video streaming info)
*/
                    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM = 256,

                    /**
Camera supports tracking of a point on the camera view.
*/
                    CAMERA_CAP_FLAGS_HAS_TRACKING_POINT = 512,

                    /**
Camera supports tracking of a selection rectangle on the camera view.
*/
                    CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE = 1024,

                    /**
Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS).
*/
                    CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS = 2048,
                }

                /**
Stream status flags (Bitmap)
*/
                enum VIDEO_STREAM_STATUS_FLAGS
                {
                    /**
Stream is active (running)
*/
                    VIDEO_STREAM_STATUS_FLAGS_RUNNING = 1,

                    /**
Stream is thermal imaging
*/
                    VIDEO_STREAM_STATUS_FLAGS_THERMAL = 2,
                }

                /**
Video stream types
*/
                enum VIDEO_STREAM_TYPE
                {
                    /**
Stream is RTSP
*/
                    VIDEO_STREAM_TYPE_RTSP = 0,

                    /**
Stream is RTP UDP (URI gives the port number)
*/
                    VIDEO_STREAM_TYPE_RTPUDP = 1,

                    /**
Stream is MPEG on TCP
*/
                    VIDEO_STREAM_TYPE_TCP_MPEG = 2,

                    /**
Stream is h.264 on MPEG TS (URI gives the port number)
*/
                    VIDEO_STREAM_TYPE_MPEG_TS_H264 = 3,
                }

                /**
Camera tracking status flags
*/
                enum CAMERA_TRACKING_STATUS_FLAGS
                {
                    /**
Camera is not tracking
*/
                    CAMERA_TRACKING_STATUS_FLAGS_IDLE = 0,

                    /**
Camera is tracking
*/
                    CAMERA_TRACKING_STATUS_FLAGS_ACTIVE = 1,

                    /**
Camera tracking in error state
*/
                    CAMERA_TRACKING_STATUS_FLAGS_ERROR = 2,
                }

                /**
Camera tracking modes
*/
                enum CAMERA_TRACKING_MODE
                {
                    /**
Not tracking
*/
                    CAMERA_TRACKING_MODE_NONE = 0,

                    /**
Target is a point
*/
                    CAMERA_TRACKING_MODE_POINT = 1,

                    /**
Target is a rectangle
*/
                    CAMERA_TRACKING_MODE_RECTANGLE = 2,
                }

                /**
Camera tracking target data (shows where tracked target is within image)
*/
                enum CAMERA_TRACKING_TARGET_DATA
                {
                    /**
No target data
*/
                    CAMERA_TRACKING_TARGET_DATA_NONE = 0,

                    /**
Target data embedded in image data (proprietary)
*/
                    CAMERA_TRACKING_TARGET_DATA_EMBEDDED = 1,

                    /**
Target data rendered in image
*/
                    CAMERA_TRACKING_TARGET_DATA_RENDERED = 2,

                    /**
Target data within status message (Point or Rectangle)
*/
                    CAMERA_TRACKING_TARGET_DATA_IN_STATUS = 4,
                }

                /**
Zoom types for MAV_CMD_SET_CAMERA_ZOOM
*/
                enum CAMERA_ZOOM_TYPE
                {
                    /**
Zoom one step increment (-1 for wide, 1 for tele)
*/
                    ZOOM_TYPE_STEP = 0,

                    /**
Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)
*/
                    ZOOM_TYPE_CONTINUOUS = 1,

                    /**
Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
*/
                    ZOOM_TYPE_RANGE = 2,

                    /**
Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom range
of the camera, so this can type can only be used for cameras where the zoom range is known (implying
that this cannot reliably be used in a GCS for an arbitrary camera)
*/
                    ZOOM_TYPE_FOCAL_LENGTH = 3,
                }

                /**
Focus types for MAV_CMD_SET_CAMERA_FOCUS
*/
                enum SET_FOCUS_TYPE
                {
                    /**
Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
*/
                    FOCUS_TYPE_STEP = 0,

                    /**
Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to
stop focusing)
*/
                    FOCUS_TYPE_CONTINUOUS = 1,

                    /**
Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
*/
                    FOCUS_TYPE_RANGE = 2,

                    /**
Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this
can type can only be used for cameras where the range is known (implying that this cannot reliably be
used in a GCS for an arbitrary camera).
*/
                    FOCUS_TYPE_METERS = 3,

                    /**
Focus automatically.
*/
                    FOCUS_TYPE_AUTO = 4,

                    /**
Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S.
*/
                    FOCUS_TYPE_AUTO_SINGLE = 5,

                    /**
Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C.
*/
                    FOCUS_TYPE_AUTO_CONTINUOUS = 6,
                }

                /**
Result from PARAM_EXT_SET message (or a PARAM_SET within a transaction).
*/
                enum PARAM_ACK
                {
                    /**
Parameter value ACCEPTED and SET
*/
                    PARAM_ACK_ACCEPTED = 0,

                    /**
Parameter value UNKNOWN/UNSUPPORTED
*/
                    PARAM_ACK_VALUE_UNSUPPORTED = 1,

                    /**
Parameter failed to set
*/
                    PARAM_ACK_FAILED = 2,

                    /**
Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or PARAM_EXT_ACK
with the final result will follow once operation is completed. This is returned immediately for parameters
that take longer to set, indicating taht the the parameter was recieved and does not need to be resent.
*/
                    PARAM_ACK_IN_PROGRESS = 3,
                }

                /**
Camera Modes.
*/
                enum CAMERA_MODE
                {
                    /**
Camera is in image/photo capture mode.
*/
                    CAMERA_MODE_IMAGE = 0,

                    /**
Camera is in video capture mode.
*/
                    CAMERA_MODE_VIDEO = 1,

                    /**
Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.
*/
                    CAMERA_MODE_IMAGE_SURVEY = 2,
                }

                enum MAV_ARM_AUTH_DENIED_REASON
                {
                    /**
Not a specific reason
*/
                    MAV_ARM_AUTH_DENIED_REASON_GENERIC = 0,

                    /**
Authorizer will send the error as string to GCS
*/
                    MAV_ARM_AUTH_DENIED_REASON_NONE = 1,

                    /**
At least one waypoint have a invalid value
*/
                    MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2,

                    /**
Timeout in the authorizer process(in case it depends on network)
*/
                    MAV_ARM_AUTH_DENIED_REASON_TIMEOUT = 3,

                    /**
Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that
caused it to be denied.
*/
                    MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4,

                    /**
Weather is not good to fly
*/
                    MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5,
                }

                /**
RC type
*/
                enum RC_TYPE
                {
                    /**
Spektrum DSM2
*/
                    RC_TYPE_SPEKTRUM_DSM2 = 0,

                    /**
Spektrum DSMX
*/
                    RC_TYPE_SPEKTRUM_DSMX = 1,
                }

                /**
Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set
the floats afx afy afz should be interpreted as force instead of acceleration.
*/
                enum POSITION_TARGET_TYPEMASK
                {
                    /**
Ignore position x
*/
                    POSITION_TARGET_TYPEMASK_X_IGNORE = 1,

                    /**
Ignore position y
*/
                    POSITION_TARGET_TYPEMASK_Y_IGNORE = 2,

                    /**
Ignore position z
*/
                    POSITION_TARGET_TYPEMASK_Z_IGNORE = 4,

                    /**
Ignore velocity x
*/
                    POSITION_TARGET_TYPEMASK_VX_IGNORE = 8,

                    /**
Ignore velocity y
*/
                    POSITION_TARGET_TYPEMASK_VY_IGNORE = 16,

                    /**
Ignore velocity z
*/
                    POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32,

                    /**
Ignore acceleration x
*/
                    POSITION_TARGET_TYPEMASK_AX_IGNORE = 64,

                    /**
Ignore acceleration y
*/
                    POSITION_TARGET_TYPEMASK_AY_IGNORE = 128,

                    /**
Ignore acceleration z
*/
                    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256,

                    /**
Use force instead of acceleration
*/
                    POSITION_TARGET_TYPEMASK_FORCE_SET = 512,

                    /**
Ignore yaw
*/
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024,

                    /**
Ignore yaw rate
*/
                    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048,
                }

                /**
Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates
that none of the setpoint dimensions should be ignored.
*/
                enum ATTITUDE_TARGET_TYPEMASK
                {
                    /**
Ignore body roll rate
*/
                    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE = 1,

                    /**
Ignore body pitch rate
*/
                    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2,

                    /**
Ignore body yaw rate
*/
                    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE = 4,

                    /**
Use 3D body thrust setpoint instead of throttle
*/
                    ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET = 32,

                    /**
Ignore throttle
*/
                    ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE = 64,

                    /**
Ignore attitude
*/
                    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE = 128,
                }

                /**
Airborne status of UAS.
*/
                enum UTM_FLIGHT_STATE
                {
                    /**
The flight state can't be determined.
*/
                    UTM_FLIGHT_STATE_UNKNOWN = 1,

                    /**
UAS on ground.
*/
                    UTM_FLIGHT_STATE_GROUND = 2,

                    /**
UAS airborne.
*/
                    UTM_FLIGHT_STATE_AIRBORNE = 3,

                    /**
UAS is in an emergency flight state.
*/
                    UTM_FLIGHT_STATE_EMERGENCY = 16,

                    /**
UAS has no active controls.
*/
                    UTM_FLIGHT_STATE_NOCTRL = 32,
                }

                /**
Flags for the global position report.
*/
                enum UTM_DATA_AVAIL_FLAGS
                {
                    /**
The field time contains valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_TIME_VALID = 1,

                    /**
The field uas_id contains valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE = 2,

                    /**
The fields lat, lon and h_acc contain valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE = 4,

                    /**
The fields alt and v_acc contain valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE = 8,

                    /**
The field relative_alt contains valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE = 16,

                    /**
The fields vx and vy contain valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE = 32,

                    /**
The field vz contains valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE = 64,

                    /**
The fields next_lat, next_lon and next_alt contain valid data.
*/
                    UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE = 128,
                }

                /**
These flags encode the cellular network status
*/
                enum CELLULAR_STATUS_FLAG
                {
                    /**
State unknown or not reportable.
*/
                    CELLULAR_STATUS_FLAG_UNKNOWN = 0,

                    /**
Modem is unusable
*/
                    CELLULAR_STATUS_FLAG_FAILED = 1,

                    /**
Modem is being initialized
*/
                    CELLULAR_STATUS_FLAG_INITIALIZING = 2,

                    /**
Modem is locked
*/
                    CELLULAR_STATUS_FLAG_LOCKED = 3,

                    /**
Modem is not enabled and is powered down
*/
                    CELLULAR_STATUS_FLAG_DISABLED = 4,

                    /**
Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state
*/
                    CELLULAR_STATUS_FLAG_DISABLING = 5,

                    /**
Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state
*/
                    CELLULAR_STATUS_FLAG_ENABLING = 6,

                    /**
Modem is enabled and powered on but not registered with a network provider and not available for data
connections
*/
                    CELLULAR_STATUS_FLAG_ENABLED = 7,

                    /**
Modem is searching for a network provider to register
*/
                    CELLULAR_STATUS_FLAG_SEARCHING = 8,

                    /**
Modem is registered with a network provider, and data connections and messaging may be available for use
*/
                    CELLULAR_STATUS_FLAG_REGISTERED = 9,

                    /**
Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered
if more than one packet data bearer is active and one of the active bearers is deactivated
*/
                    CELLULAR_STATUS_FLAG_DISCONNECTING = 10,

                    /**
Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another
bearer is already active do not cause this state to be entered
*/
                    CELLULAR_STATUS_FLAG_CONNECTING = 11,

                    /**
One or more packet data bearers is active and connected
*/
                    CELLULAR_STATUS_FLAG_CONNECTED = 12,
                }

                /**
These flags are used to diagnose the failure state of CELLULAR_STATUS
*/
                enum CELLULAR_NETWORK_FAILED_REASON
                {
                    /**
No error
*/
                    CELLULAR_NETWORK_FAILED_REASON_NONE = 0,

                    /**
Error state is unknown
*/
                    CELLULAR_NETWORK_FAILED_REASON_UNKNOWN = 1,

                    /**
SIM is required for the modem but missing
*/
                    CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING = 2,

                    /**
SIM is available, but not usuable for connection
*/
                    CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR = 3,
                }

                /**
Cellular network radio type
*/
                enum CELLULAR_NETWORK_RADIO_TYPE
                {
                    CELLULAR_NETWORK_RADIO_TYPE_NONE  = 0,
                    CELLULAR_NETWORK_RADIO_TYPE_GSM   = 1,
                    CELLULAR_NETWORK_RADIO_TYPE_CDMA  = 2,
                    CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3,
                    CELLULAR_NETWORK_RADIO_TYPE_LTE   = 4,
                }

                /**
Precision land modes (used in MAV_CMD_NAV_LAND).
*/
                enum PRECISION_LAND_MODE
                {
                    /**
Normal (non-precision) landing.
*/
                    PRECISION_LAND_MODE_DISABLED = 0,

                    /**
Use precision landing if beacon detected when land command accepted, otherwise land normally.
*/
                    PRECISION_LAND_MODE_OPPORTUNISTIC = 1,

                    /**
Use precision landing, searching for beacon if not found when land command accepted (land normally if
beacon cannot be found).
*/
                    PRECISION_LAND_MODE_REQUIRED = 2,
                }

                /**
Parachute actions. Trigger release and enable/disable auto-release.
*/
                enum PARACHUTE_ACTION
                {
                    /**
Disable auto-release of parachute (i.e. release triggered by crash detectors).
*/
                    PARACHUTE_DISABLE = 0,

                    /**
Enable auto-release of parachute.
*/
                    PARACHUTE_ENABLE = 1,

                    /**
Release parachute and kill motors.
*/
                    PARACHUTE_RELEASE = 2,
                }

                enum MAV_TUNNEL_PAYLOAD_TYPE
                {
                    /**
Encoding of payload unknown.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN = 0,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208,

                    /**
Registered for STorM32 gimbal controller.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209,
                }

                enum MAV_ODID_ID_TYPE
                {
                    /**
No type defined.
*/
                    MAV_ODID_ID_TYPE_NONE = 0,

                    /**
Manufacturer Serial Number (ANSI/CTA-2063 format).
*/
                    MAV_ODID_ID_TYPE_SERIAL_NUMBER = 1,

                    /**
.
*/
                    MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID = 2,

                    /**
UTM (Unmanned Traffic Management) assigned UUID (RFC4122).
*/
                    MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID = 3,

                    /**
A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id
and these type values are managed by ICAO.
*/
                    MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID = 4,
                }

                enum MAV_ODID_UA_TYPE
                {
                    /**
No UA (Unmanned Aircraft) type defined.
*/
                    MAV_ODID_UA_TYPE_NONE = 0,

                    /**
Aeroplane/Airplane. Fixed wing.
*/
                    MAV_ODID_UA_TYPE_AEROPLANE = 1,

                    /**
Helicopter or multirotor.
*/
                    MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR = 2,

                    /**
Gyroplane.
*/
                    MAV_ODID_UA_TYPE_GYROPLANE = 3,

                    /**
VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
*/
                    MAV_ODID_UA_TYPE_HYBRID_LIFT = 4,

                    /**
Ornithopter.
*/
                    MAV_ODID_UA_TYPE_ORNITHOPTER = 5,

                    /**
Glider.
*/
                    MAV_ODID_UA_TYPE_GLIDER = 6,

                    /**
Kite.
*/
                    MAV_ODID_UA_TYPE_KITE = 7,

                    /**
Free Balloon.
*/
                    MAV_ODID_UA_TYPE_FREE_BALLOON = 8,

                    /**
Captive Balloon.
*/
                    MAV_ODID_UA_TYPE_CAPTIVE_BALLOON = 9,

                    /**
Airship. E.g. a blimp.
*/
                    MAV_ODID_UA_TYPE_AIRSHIP = 10,

                    /**
Free Fall/Parachute (unpowered).
*/
                    MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE = 11,

                    /**
Rocket.
*/
                    MAV_ODID_UA_TYPE_ROCKET = 12,

                    /**
Tethered powered aircraft.
*/
                    MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT = 13,

                    /**
Ground Obstacle.
*/
                    MAV_ODID_UA_TYPE_GROUND_OBSTACLE = 14,

                    /**
Other type of aircraft not listed earlier.
*/
                    MAV_ODID_UA_TYPE_OTHER = 15,
                }

                enum MAV_ODID_STATUS
                {
                    /**
The status of the (UA) Unmanned Aircraft is undefined.
*/
                    MAV_ODID_STATUS_UNDECLARED = 0,

                    /**
The UA is on the ground.
*/
                    MAV_ODID_STATUS_GROUND = 1,

                    /**
The UA is in the air.
*/
                    MAV_ODID_STATUS_AIRBORNE = 2,

                    /**
The UA is having an emergency.
*/
                    MAV_ODID_STATUS_EMERGENCY = 3,
                }

                enum MAV_ODID_HEIGHT_REF
                {
                    /**
The height field is relative to the take-off location.
*/
                    MAV_ODID_HEIGHT_REF_OVER_TAKEOFF = 0,

                    /**
The height field is relative to ground.
*/
                    MAV_ODID_HEIGHT_REF_OVER_GROUND = 1,
                }

                enum MAV_ODID_HOR_ACC
                {
                    /**
The horizontal accuracy is unknown.
*/
                    MAV_ODID_HOR_ACC_UNKNOWN = 0,

                    /**
The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km.
*/
                    MAV_ODID_HOR_ACC_10NM = 1,

                    /**
The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km.
*/
                    MAV_ODID_HOR_ACC_4NM = 2,

                    /**
The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km.
*/
                    MAV_ODID_HOR_ACC_2NM = 3,

                    /**
The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km.
*/
                    MAV_ODID_HOR_ACC_1NM = 4,

                    /**
The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m.
*/
                    MAV_ODID_HOR_ACC_0_5NM = 5,

                    /**
The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m.
*/
                    MAV_ODID_HOR_ACC_0_3NM = 6,

                    /**
The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m.
*/
                    MAV_ODID_HOR_ACC_0_1NM = 7,

                    /**
The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m.
*/
                    MAV_ODID_HOR_ACC_0_05NM = 8,

                    /**
The horizontal accuracy is smaller than 30 meter.
*/
                    MAV_ODID_HOR_ACC_30_METER = 9,

                    /**
The horizontal accuracy is smaller than 10 meter.
*/
                    MAV_ODID_HOR_ACC_10_METER = 10,

                    /**
The horizontal accuracy is smaller than 3 meter.
*/
                    MAV_ODID_HOR_ACC_3_METER = 11,

                    /**
The horizontal accuracy is smaller than 1 meter.
*/
                    MAV_ODID_HOR_ACC_1_METER = 12,
                }

                enum MAV_ODID_VER_ACC
                {
                    /**
The vertical accuracy is unknown.
*/
                    MAV_ODID_VER_ACC_UNKNOWN = 0,

                    /**
The vertical accuracy is smaller than 150 meter.
*/
                    MAV_ODID_VER_ACC_150_METER = 1,

                    /**
The vertical accuracy is smaller than 45 meter.
*/
                    MAV_ODID_VER_ACC_45_METER = 2,

                    /**
The vertical accuracy is smaller than 25 meter.
*/
                    MAV_ODID_VER_ACC_25_METER = 3,

                    /**
The vertical accuracy is smaller than 10 meter.
*/
                    MAV_ODID_VER_ACC_10_METER = 4,

                    /**
The vertical accuracy is smaller than 3 meter.
*/
                    MAV_ODID_VER_ACC_3_METER = 5,

                    /**
The vertical accuracy is smaller than 1 meter.
*/
                    MAV_ODID_VER_ACC_1_METER = 6,
                }

                enum MAV_ODID_SPEED_ACC
                {
                    /**
The speed accuracy is unknown.
*/
                    MAV_ODID_SPEED_ACC_UNKNOWN = 0,

                    /**
The speed accuracy is smaller than 10 meters per second.
*/
                    MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND = 1,

                    /**
The speed accuracy is smaller than 3 meters per second.
*/
                    MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND = 2,

                    /**
The speed accuracy is smaller than 1 meters per second.
*/
                    MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND = 3,

                    /**
The speed accuracy is smaller than 0.3 meters per second.
*/
                    MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND = 4,
                }

                enum MAV_ODID_TIME_ACC
                {
                    /**
The timestamp accuracy is unknown.
*/
                    MAV_ODID_TIME_ACC_UNKNOWN = 0,

                    /**
The timestamp accuracy is smaller than or equal to 0.1 second.
*/
                    MAV_ODID_TIME_ACC_0_1_SECOND = 1,

                    /**
The timestamp accuracy is smaller than or equal to 0.2 second.
*/
                    MAV_ODID_TIME_ACC_0_2_SECOND = 2,

                    /**
The timestamp accuracy is smaller than or equal to 0.3 second.
*/
                    MAV_ODID_TIME_ACC_0_3_SECOND = 3,

                    /**
The timestamp accuracy is smaller than or equal to 0.4 second.
*/
                    MAV_ODID_TIME_ACC_0_4_SECOND = 4,

                    /**
The timestamp accuracy is smaller than or equal to 0.5 second.
*/
                    MAV_ODID_TIME_ACC_0_5_SECOND = 5,

                    /**
The timestamp accuracy is smaller than or equal to 0.6 second.
*/
                    MAV_ODID_TIME_ACC_0_6_SECOND = 6,

                    /**
The timestamp accuracy is smaller than or equal to 0.7 second.
*/
                    MAV_ODID_TIME_ACC_0_7_SECOND = 7,

                    /**
The timestamp accuracy is smaller than or equal to 0.8 second.
*/
                    MAV_ODID_TIME_ACC_0_8_SECOND = 8,

                    /**
The timestamp accuracy is smaller than or equal to 0.9 second.
*/
                    MAV_ODID_TIME_ACC_0_9_SECOND = 9,

                    /**
The timestamp accuracy is smaller than or equal to 1.0 second.
*/
                    MAV_ODID_TIME_ACC_1_0_SECOND = 10,

                    /**
The timestamp accuracy is smaller than or equal to 1.1 second.
*/
                    MAV_ODID_TIME_ACC_1_1_SECOND = 11,

                    /**
The timestamp accuracy is smaller than or equal to 1.2 second.
*/
                    MAV_ODID_TIME_ACC_1_2_SECOND = 12,

                    /**
The timestamp accuracy is smaller than or equal to 1.3 second.
*/
                    MAV_ODID_TIME_ACC_1_3_SECOND = 13,

                    /**
The timestamp accuracy is smaller than or equal to 1.4 second.
*/
                    MAV_ODID_TIME_ACC_1_4_SECOND = 14,

                    /**
The timestamp accuracy is smaller than or equal to 1.5 second.
*/
                    MAV_ODID_TIME_ACC_1_5_SECOND = 15,
                }

                enum MAV_ODID_AUTH_TYPE
                {
                    /**
No authentication type is specified.
*/
                    MAV_ODID_AUTH_TYPE_NONE = 0,

                    /**
Signature for the UAS (Unmanned Aircraft System) ID.
*/
                    MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE = 1,

                    /**
Signature for the Operator ID.
*/
                    MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE = 2,

                    /**
Signature for the entire message set.
*/
                    MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE = 3,

                    /**
Authentication is provided by Network Remote ID.
*/
                    MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID = 4,

                    /**
The exact authentication type is indicated by the first byte of authentication_data and these type values
are managed by ICAO.
*/
                    MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION = 5,
                }

                enum MAV_ODID_DESC_TYPE
                {
                    /**
Free-form text description of the purpose of the flight.
*/
                    MAV_ODID_DESC_TYPE_TEXT = 0,
                }

                enum MAV_ODID_OPERATOR_LOCATION_TYPE
                {
                    /**
The location of the operator is the same as the take-off location.
*/
                    MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF = 0,

                    /**
The location of the operator is based on live GNSS data.
*/
                    MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS = 1,

                    /**
The location of the operator is a fixed location.
*/
                    MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED = 2,
                }

                enum MAV_ODID_CLASSIFICATION_TYPE
                {
                    /**
The classification type for the UA is undeclared.
*/
                    MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED = 0,

                    /**
The classification type for the UA follows EU (European Union) specifications.
*/
                    MAV_ODID_CLASSIFICATION_TYPE_EU = 1,
                }

                enum MAV_ODID_CATEGORY_EU
                {
                    /**
The category for the UA, according to the EU specification, is undeclared.
*/
                    MAV_ODID_CATEGORY_EU_UNDECLARED = 0,

                    /**
The category for the UA, according to the EU specification, is the Open category.
*/
                    MAV_ODID_CATEGORY_EU_OPEN = 1,

                    /**
The category for the UA, according to the EU specification, is the Specific category.
*/
                    MAV_ODID_CATEGORY_EU_SPECIFIC = 2,

                    /**
The category for the UA, according to the EU specification, is the Certified category.
*/
                    MAV_ODID_CATEGORY_EU_CERTIFIED = 3,
                }

                enum MAV_ODID_CLASS_EU
                {
                    /**
The class for the UA, according to the EU specification, is undeclared.
*/
                    MAV_ODID_CLASS_EU_UNDECLARED = 0,

                    /**
The class for the UA, according to the EU specification, is Class 0.
*/
                    MAV_ODID_CLASS_EU_CLASS_0 = 1,

                    /**
The class for the UA, according to the EU specification, is Class 1.
*/
                    MAV_ODID_CLASS_EU_CLASS_1 = 2,

                    /**
The class for the UA, according to the EU specification, is Class 2.
*/
                    MAV_ODID_CLASS_EU_CLASS_2 = 3,

                    /**
The class for the UA, according to the EU specification, is Class 3.
*/
                    MAV_ODID_CLASS_EU_CLASS_3 = 4,

                    /**
The class for the UA, according to the EU specification, is Class 4.
*/
                    MAV_ODID_CLASS_EU_CLASS_4 = 5,

                    /**
The class for the UA, according to the EU specification, is Class 5.
*/
                    MAV_ODID_CLASS_EU_CLASS_5 = 6,

                    /**
The class for the UA, according to the EU specification, is Class 6.
*/
                    MAV_ODID_CLASS_EU_CLASS_6 = 7,
                }

                enum MAV_ODID_OPERATOR_ID_TYPE
                {
                    /**
CAA (Civil Aviation Authority) registered operator ID.
*/
                    MAV_ODID_OPERATOR_ID_TYPE_CAA = 0,
                }

                /**
Tune formats (used for vehicle buzzer/tone generation).
*/
                enum TUNE_FORMAT
                {
                    /**
Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm.
*/
                    TUNE_FORMAT_QBASIC1_1 = 1,

                    /**
Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML.
*/
                    TUNE_FORMAT_MML_MODERN = 2,
                }

                /**
Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
*/
                enum AIS_TYPE
                {
                    /**
Not available (default).
*/
                    AIS_TYPE_UNKNOWN = 0,
                    AIS_TYPE_RESERVED_1  = 1,
                    AIS_TYPE_RESERVED_2  = 2,
                    AIS_TYPE_RESERVED_3  = 3,
                    AIS_TYPE_RESERVED_4  = 4,
                    AIS_TYPE_RESERVED_5  = 5,
                    AIS_TYPE_RESERVED_6  = 6,
                    AIS_TYPE_RESERVED_7  = 7,
                    AIS_TYPE_RESERVED_8  = 8,
                    AIS_TYPE_RESERVED_9  = 9,
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

                    /**
Wing In Ground effect.
*/
                    AIS_TYPE_WIG = 20,
                    AIS_TYPE_WIG_HAZARDOUS_A = 21,
                    AIS_TYPE_WIG_HAZARDOUS_B = 22,
                    AIS_TYPE_WIG_HAZARDOUS_C = 23,
                    AIS_TYPE_WIG_HAZARDOUS_D = 24,
                    AIS_TYPE_WIG_RESERVED_1  = 25,
                    AIS_TYPE_WIG_RESERVED_2  = 26,
                    AIS_TYPE_WIG_RESERVED_3  = 27,
                    AIS_TYPE_WIG_RESERVED_4  = 28,
                    AIS_TYPE_WIG_RESERVED_5  = 29,
                    AIS_TYPE_FISHING         = 30,
                    AIS_TYPE_TOWING          = 31,

                    /**
Towing: length exceeds 200m or breadth exceeds 25m.
*/
                    AIS_TYPE_TOWING_LARGE = 32,

                    /**
Dredging or other underwater ops.
*/
                    AIS_TYPE_DREDGING = 33,
                    AIS_TYPE_DIVING      = 34,
                    AIS_TYPE_MILITARY    = 35,
                    AIS_TYPE_SAILING     = 36,
                    AIS_TYPE_PLEASURE    = 37,
                    AIS_TYPE_RESERVED_20 = 38,
                    AIS_TYPE_RESERVED_21 = 39,

                    /**
High Speed Craft.
*/
                    AIS_TYPE_HSC = 40,
                    AIS_TYPE_HSC_HAZARDOUS_A = 41,
                    AIS_TYPE_HSC_HAZARDOUS_B = 42,
                    AIS_TYPE_HSC_HAZARDOUS_C = 43,
                    AIS_TYPE_HSC_HAZARDOUS_D = 44,
                    AIS_TYPE_HSC_RESERVED_1  = 45,
                    AIS_TYPE_HSC_RESERVED_2  = 46,
                    AIS_TYPE_HSC_RESERVED_3  = 47,
                    AIS_TYPE_HSC_RESERVED_4  = 48,
                    AIS_TYPE_HSC_UNKNOWN     = 49,
                    AIS_TYPE_PILOT           = 50,

                    /**
Search And Rescue vessel.
*/
                    AIS_TYPE_SAR = 51,
                    AIS_TYPE_TUG         = 52,
                    AIS_TYPE_PORT_TENDER = 53,

                    /**
Anti-pollution equipment.
*/
                    AIS_TYPE_ANTI_POLLUTION = 54,
                    AIS_TYPE_LAW_ENFORCEMENT   = 55,
                    AIS_TYPE_SPARE_LOCAL_1     = 56,
                    AIS_TYPE_SPARE_LOCAL_2     = 57,
                    AIS_TYPE_MEDICAL_TRANSPORT = 58,

                    /**
Noncombatant ship according to RR Resolution No. 18.
*/
                    AIS_TYPE_NONECOMBATANT = 59,
                    AIS_TYPE_PASSENGER                      = 60,
                    AIS_TYPE_PASSENGER_HAZARDOUS_A          = 61,
                    AIS_TYPE_PASSENGER_HAZARDOUS_B          = 62,
                    AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C = 63,
                    AIS_TYPE_PASSENGER_HAZARDOUS_D          = 64,
                    AIS_TYPE_PASSENGER_RESERVED_1           = 65,
                    AIS_TYPE_PASSENGER_RESERVED_2           = 66,
                    AIS_TYPE_PASSENGER_RESERVED_3           = 67,
                    AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4  = 68,
                    AIS_TYPE_PASSENGER_UNKNOWN              = 69,
                    AIS_TYPE_CARGO                          = 70,
                    AIS_TYPE_CARGO_HAZARDOUS_A              = 71,
                    AIS_TYPE_CARGO_HAZARDOUS_B              = 72,
                    AIS_TYPE_CARGO_HAZARDOUS_C              = 73,
                    AIS_TYPE_CARGO_HAZARDOUS_D              = 74,
                    AIS_TYPE_CARGO_RESERVED_1               = 75,
                    AIS_TYPE_CARGO_RESERVED_2               = 76,
                    AIS_TYPE_CARGO_RESERVED_3               = 77,
                    AIS_TYPE_CARGO_RESERVED_4               = 78,
                    AIS_TYPE_CARGO_UNKNOWN                  = 79,
                    AIS_TYPE_TANKER                         = 80,
                    AIS_TYPE_TANKER_HAZARDOUS_A             = 81,
                    AIS_TYPE_TANKER_HAZARDOUS_B             = 82,
                    AIS_TYPE_TANKER_HAZARDOUS_C             = 83,
                    AIS_TYPE_TANKER_HAZARDOUS_D             = 84,
                    AIS_TYPE_TANKER_RESERVED_1              = 85,
                    AIS_TYPE_TANKER_RESERVED_2              = 86,
                    AIS_TYPE_TANKER_RESERVED_3              = 87,
                    AIS_TYPE_TANKER_RESERVED_4              = 88,
                    AIS_TYPE_TANKER_UNKNOWN                 = 89,
                    AIS_TYPE_OTHER                          = 90,
                    AIS_TYPE_OTHER_HAZARDOUS_A              = 91,
                    AIS_TYPE_OTHER_HAZARDOUS_B              = 92,
                    AIS_TYPE_OTHER_HAZARDOUS_C              = 93,
                    AIS_TYPE_OTHER_HAZARDOUS_D              = 94,
                    AIS_TYPE_OTHER_RESERVED_1               = 95,
                    AIS_TYPE_OTHER_RESERVED_2               = 96,
                    AIS_TYPE_OTHER_RESERVED_3               = 97,
                    AIS_TYPE_OTHER_RESERVED_4               = 98,
                    AIS_TYPE_OTHER_UNKNOWN                  = 99,
                }

                /**
Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
*/
                enum AIS_NAV_STATUS
                {
                    /**
Under way using engine.
*/
                    UNDER_WAY = 0,
                    AIS_NAV_ANCHORED                    = 1,
                    AIS_NAV_UN_COMMANDED                = 2,
                    AIS_NAV_RESTRICTED_MANOEUVERABILITY = 3,
                    AIS_NAV_DRAUGHT_CONSTRAINED         = 4,
                    AIS_NAV_MOORED                      = 5,
                    AIS_NAV_AGROUND                     = 6,
                    AIS_NAV_FISHING                     = 7,
                    AIS_NAV_SAILING                     = 8,
                    AIS_NAV_RESERVED_HSC                = 9,
                    AIS_NAV_RESERVED_WIG                = 10,
                    AIS_NAV_RESERVED_1                  = 11,
                    AIS_NAV_RESERVED_2                  = 12,
                    AIS_NAV_RESERVED_3                  = 13,

                    /**
Search And Rescue Transponder.
*/
                    AIS_NAV_AIS_SART = 14,

                    /**
Not available (default).
*/
                    AIS_NAV_UNKNOWN = 15,
                }

                /**
These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message
fields. When set, the data is valid.
*/
                enum AIS_FLAGS
                {
                    /**
1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m.
*/
                    AIS_FLAGS_POSITION_ACCURACY = 1,
                    AIS_FLAGS_VALID_COG      = 2,
                    AIS_FLAGS_VALID_VELOCITY = 4,

                    /**
1 = Velocity over 52.5765m/s (102.2 knots)
*/
                    AIS_FLAGS_HIGH_VELOCITY = 8,
                    AIS_FLAGS_VALID_TURN_RATE = 16,

                    /**
Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s
*/
                    AIS_FLAGS_TURN_RATE_SIGN_ONLY = 32,
                    AIS_FLAGS_VALID_DIMENSIONS = 64,

                    /**
Distance to bow is larger than 511m
*/
                    AIS_FLAGS_LARGE_BOW_DIMENSION = 128,

                    /**
Distance to stern is larger than 511m
*/
                    AIS_FLAGS_LARGE_STERN_DIMENSION = 256,

                    /**
Distance to port side is larger than 63m
*/
                    AIS_FLAGS_LARGE_PORT_DIMENSION = 512,

                    /**
Distance to starboard side is larger than 63m
*/
                    AIS_FLAGS_LARGE_STARBOARD_DIMENSION = 1024,
                    AIS_FLAGS_VALID_CALLSIGN = 2048,
                    AIS_FLAGS_VALID_NAME     = 4096,
                }

                /**
List of possible units where failures can be injected.
*/
                enum FAILURE_UNIT
                {
                    FAILURE_UNIT_SENSOR_GYRO            = 0,
                    FAILURE_UNIT_SENSOR_ACCEL           = 1,
                    FAILURE_UNIT_SENSOR_MAG             = 2,
                    FAILURE_UNIT_SENSOR_BARO            = 3,
                    FAILURE_UNIT_SENSOR_GPS             = 4,
                    FAILURE_UNIT_SENSOR_OPTICAL_FLOW    = 5,
                    FAILURE_UNIT_SENSOR_VIO             = 6,
                    FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7,
                    FAILURE_UNIT_SENSOR_AIRSPEED        = 8,
                    FAILURE_UNIT_SYSTEM_BATTERY         = 100,
                    FAILURE_UNIT_SYSTEM_MOTOR           = 101,
                    FAILURE_UNIT_SYSTEM_SERVO           = 102,
                    FAILURE_UNIT_SYSTEM_AVOIDANCE       = 103,
                    FAILURE_UNIT_SYSTEM_RC_SIGNAL       = 104,
                    FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL  = 105,
                }

                /**
List of possible failure type to inject.
*/
                enum FAILURE_TYPE
                {
                    /**
No failure injected, used to reset a previous failure.
*/
                    FAILURE_TYPE_OK = 0,

                    /**
Sets unit off, so completely non-responsive.
*/
                    FAILURE_TYPE_OFF = 1,

                    /**
Unit is stuck e.g. keeps reporting the same value.
*/
                    FAILURE_TYPE_STUCK = 2,

                    /**
Unit is reporting complete garbage.
*/
                    FAILURE_TYPE_GARBAGE = 3,

                    /**
Unit is consistently wrong.
*/
                    FAILURE_TYPE_WRONG = 4,

                    /**
Unit is slow, so e.g. reporting at slower than expected rate.
*/
                    FAILURE_TYPE_SLOW = 5,

                    /**
Data of unit is delayed in time.
*/
                    FAILURE_TYPE_DELAYED = 6,

                    /**
Unit is sometimes working, sometimes not.
*/
                    FAILURE_TYPE_INTERMITTENT = 7,
                }

                enum NAV_VTOL_LAND_OPTIONS
                {
                    /**
Default autopilot landing behaviour.
*/
                    NAV_VTOL_LAND_OPTIONS_DEFAULT = 0,

                    /**
Descend in fixed wing mode, transitioning to multicopter mode for vertical landing when close to the ground.

         The fixed wing descent pattern is at the discretion of the vehicle (e.g. transition altitude,
loiter direction, radius, and speed, etc.).
*/
                    NAV_VTOL_LAND_OPTIONS_FW_DESCENT = 1,

                    /**
Land in multicopter mode on reaching the landing co-ordinates (the whole landing is by "hover descent").
*/
                    NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT = 2,
                }

                /**
Winch status flags used in WINCH_STATUS
*/
                enum MAV_WINCH_STATUS_FLAG
                {
                    /**
Winch is healthy
*/
                    MAV_WINCH_STATUS_HEALTHY = 1,

                    /**
Winch line is fully retracted
*/
                    MAV_WINCH_STATUS_FULLY_RETRACTED = 2,

                    /**
Winch motor is moving
*/
                    MAV_WINCH_STATUS_MOVING = 4,

                    /**
Winch clutch is engaged allowing motor to move freely.
*/
                    MAV_WINCH_STATUS_CLUTCH_ENGAGED = 8,

                    /**
Winch is locked by locking mechanism.
*/
                    MAV_WINCH_STATUS_LOCKED = 16,

                    /**
Winch is gravity dropping payload.
*/
                    MAV_WINCH_STATUS_DROPPING = 32,

                    /**
Winch is arresting payload descent.
*/
                    MAV_WINCH_STATUS_ARRESTING = 64,

                    /**
Winch is using torque measurements to sense the ground.
*/
                    MAV_WINCH_STATUS_GROUND_SENSE = 128,

                    /**
Winch is returning to the fully retracted position.
*/
                    MAV_WINCH_STATUS_RETRACTING = 256,

                    /**
Winch is redelivering the payload. This is a failover state if the line tension goes above a threshold
during RETRACTING.
*/
                    MAV_WINCH_STATUS_REDELIVER = 512,

                    /**
Winch is abandoning the line and possibly payload. Winch unspools the entire calculated line length. This
is a failover state from REDELIVER if the number of attemps exceeds a threshold.
*/
                    MAV_WINCH_STATUS_ABANDON_LINE = 1024,
                }

                enum MAG_CAL_STATUS
                {
                    MAG_CAL_NOT_STARTED      = 0,
                    MAG_CAL_WAITING_TO_START = 1,
                    MAG_CAL_RUNNING_STEP_ONE = 2,
                    MAG_CAL_RUNNING_STEP_TWO = 3,
                    MAG_CAL_SUCCESS          = 4,
                    MAG_CAL_FAILED           = 5,
                    MAG_CAL_BAD_ORIENTATION  = 6,
                    MAG_CAL_BAD_RADIUS       = 7,
                }

                /**
Reason for an event error response.
*/
                enum MAV_EVENT_ERROR_REASON
                {
                    /**
The requested event is not available (anymore).
*/
                    MAV_EVENT_ERROR_REASON_UNAVAILABLE = 0,
                }

                /**
Flags for CURRENT_EVENT_SEQUENCE.
*/
                enum MAV_EVENT_CURRENT_SEQUENCE_FLAGS
                {
                    /**
A sequence reset has happened (e.g. vehicle reboot).
*/
                    MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET = 1,
                }

                /**
Flags in the HIL_SENSOR message indicate which fields have updated since the last message
*/
                enum HIL_SENSOR_UPDATED_FLAGS : long
                {
                    /**
None of the fields in HIL_SENSOR have been updated
*/
                    HIL_SENSOR_UPDATED_NONE = 0,

                    /**
The value in the xacc field has been updated
*/
                    HIL_SENSOR_UPDATED_XACC = 1,

                    /**
The value in the yacc field has been updated
*/
                    HIL_SENSOR_UPDATED_YACC = 2,

                    /**
The value in the zacc field has been updated
*/
                    HIL_SENSOR_UPDATED_ZACC = 4,

                    /**
The value in the xgyro field has been updated
*/
                    HIL_SENSOR_UPDATED_XGYRO = 8,

                    /**
The value in the ygyro field has been updated
*/
                    HIL_SENSOR_UPDATED_YGYRO = 16,

                    /**
The value in the zgyro field has been updated
*/
                    HIL_SENSOR_UPDATED_ZGYRO = 32,

                    /**
The value in the xmag field has been updated
*/
                    HIL_SENSOR_UPDATED_XMAG = 64,

                    /**
The value in the ymag field has been updated
*/
                    HIL_SENSOR_UPDATED_YMAG = 128,

                    /**
The value in the zmag field has been updated
*/
                    HIL_SENSOR_UPDATED_ZMAG = 256,

                    /**
The value in the abs_pressure field has been updated
*/
                    HIL_SENSOR_UPDATED_ABS_PRESSURE = 512,

                    /**
The value in the diff_pressure field has been updated
*/
                    HIL_SENSOR_UPDATED_DIFF_PRESSURE = 1024,

                    /**
The value in the pressure_alt field has been updated
*/
                    HIL_SENSOR_UPDATED_PRESSURE_ALT = 2048,

                    /**
The value in the temperature field has been updated
*/
                    HIL_SENSOR_UPDATED_TEMPERATURE = 4096,

                    /**
Full reset of attitude/position/velocities/etc was performed in sim (Bit 31).
*/
                    HIL_SENSOR_UPDATED_RESET = 2147483648,
                }

                /**
Flags in the HIGHRES_IMU message indicate which fields have updated since the last message
*/
                enum HIGHRES_IMU_UPDATED_FLAGS
                {
                    /**
None of the fields in HIGHRES_IMU have been updated
*/
                    HIGHRES_IMU_UPDATED_NONE = 0,

                    /**
The value in the xacc field has been updated
*/
                    HIGHRES_IMU_UPDATED_XACC = 1,

                    /**
The value in the yacc field has been updated
*/
                    HIGHRES_IMU_UPDATED_YACC = 2,

                    /**
The value in the zacc field has been updated since
*/
                    HIGHRES_IMU_UPDATED_ZACC = 4,

                    /**
The value in the xgyro field has been updated
*/
                    HIGHRES_IMU_UPDATED_XGYRO = 8,

                    /**
The value in the ygyro field has been updated
*/
                    HIGHRES_IMU_UPDATED_YGYRO = 16,

                    /**
The value in the zgyro field has been updated
*/
                    HIGHRES_IMU_UPDATED_ZGYRO = 32,

                    /**
The value in the xmag field has been updated
*/
                    HIGHRES_IMU_UPDATED_XMAG = 64,

                    /**
The value in the ymag field has been updated
*/
                    HIGHRES_IMU_UPDATED_YMAG = 128,

                    /**
The value in the zmag field has been updated
*/
                    HIGHRES_IMU_UPDATED_ZMAG = 256,

                    /**
The value in the abs_pressure field has been updated
*/
                    HIGHRES_IMU_UPDATED_ABS_PRESSURE = 512,

                    /**
The value in the diff_pressure field has been updated
*/
                    HIGHRES_IMU_UPDATED_DIFF_PRESSURE = 1024,

                    /**
The value in the pressure_alt field has been updated
*/
                    HIGHRES_IMU_UPDATED_PRESSURE_ALT = 2048,

                    /**
The value in the temperature field has been updated
*/
                    HIGHRES_IMU_UPDATED_TEMPERATURE = 4096,

                    /**
All fields in HIGHRES_IMU have been updated.
*/
                    HIGHRES_IMU_UPDATED_ALL = 65535,
                }

                enum CAN_FILTER_OP
                {
                    CAN_FILTER_REPLACE = 0, CAN_FILTER_ADD = 1, CAN_FILTER_REMOVE = 2,
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
timeout.
*/
                class SYS_STATUS
                {
                    /**
Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
present.
*/
                    MAV_SYS_STATUS_SENSOR onboard_control_sensors_present;

                    /**
Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1:
enabled.
*/
                    MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled;

                    /**
Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
Value of 1: healthy.
*/
                    MAV_SYS_STATUS_SENSOR onboard_control_sensors_health;

                    /**
Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
*/
                    ushort load;

                    /**
Battery voltage, UINT16_MAX: Voltage not sent by autopilot
*/
                    ushort voltage_battery;

                    /**
Battery current, -1: Current not sent by autopilot
*/
                    short current_battery;

                    /**
Battery energy remaining, -1: Battery remaining energy not sent by autopilot
*/
                    sbyte battery_remaining;

                    /**
Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
on reception on the MAV)
*/
                    ushort drop_rate_comm;

                    /**
Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
on reception on the MAV)
*/
                    ushort errors_comm;

                    /**
Autopilot-specific errors
*/
                    ushort errors_count1;

                    /**
Autopilot-specific errors
*/
                    ushort errors_count2;

                    /**
Autopilot-specific errors
*/
                    ushort errors_count3;

                    /**
Autopilot-specific errors
*/
                    ushort errors_count4;

                    /**
Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
present.
*/
                    MAV_SYS_STATUS_SENSOR_EXTENDED onboard_control_sensors_present_extended;

                    /**
Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1:
enabled.
*/
                    MAV_SYS_STATUS_SENSOR_EXTENDED onboard_control_sensors_enabled_extended;

                    /**
Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
Value of 1: healthy.
*/
                    MAV_SYS_STATUS_SENSOR_EXTENDED onboard_control_sensors_health_extended;
                }

                /**
The system time is the time of the master clock, typically the computer clock of the main onboard computer.
*/
                class SYSTEM_TIME
                {
                    /**
Timestamp (UNIX epoch time).
*/
                    ulong time_unix_usec;

                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;
                }

                /**
A ping message either requesting or responding to a ping. This allows to measure the system latencies,
including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.html
*/
                class PING
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
PING sequence
*/
                    uint seq;

                    /**
0: request ping from all receiving systems. If greater than 0: message is a ping response and number is
the system id of the requesting system
*/
                    byte target_system;

                    /**
0: request ping from all receiving components. If greater than 0: message is a ping response and number
is the component id of the requesting component.
*/
                    byte target_component;
                }

                /**
Request to control this MAV
*/
                class CHANGE_OPERATOR_CONTROL
                {
                    /**
System the GCS requests control for
*/
                    byte target_system;

                    /**
0: request control of this MAV, 1: Release control of this MAV
*/
                    byte control_request;

                    /**
0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
message indicating an encryption mismatch.
*/
                    byte version;

                    /**
Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
characters may involve A-Z, a-z, 0-9, and "!?,.-"
*/
                    string passkey;
                }

                /**
Accept / deny control of this MAV
*/
                class CHANGE_OPERATOR_CONTROL_ACK
                {
                    /**
ID of the GCS this message 
*/
                    byte gcs_system_id;

                    /**
0: request control of this MAV, 1: Release control of this MAV
*/
                    byte control_request;

                    /**
0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
control
*/
                    byte ack;
                }

                /**
Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
so transmitting the key requires an encrypted channel for true safety.
*/
                class AUTH_KEY
                {
                    /**
key
*/
                    string key;
                }

                /**
Status generated in each node in the communication chain and injected into MAVLink stream.
*/
                class LINK_NODE_STATUS
                {
                    /**
Timestamp (time since system boot).
*/
                    ulong timestamp;

                    /**
Remaining free transmit buffer space
*/
                    byte tx_buf;

                    /**
Remaining free receive buffer space
*/
                    byte rx_buf;

                    /**
Transmit rate
*/
                    uint tx_rate;

                    /**
Receive rate
*/
                    uint rx_rate;

                    /**
Number of bytes that could not be parsed correctly.
*/
                    ushort rx_parse_err;

                    /**
Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
*/
                    ushort tx_overflows;

                    /**
Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
*/
                    ushort rx_overflows;

                    /**
Messages sent
*/
                    uint messages_sent;

                    /**
Messages received (estimated from counting seq)
*/
                    uint messages_received;

                    /**
Messages lost (estimated from counting seq)
*/
                    uint messages_lost;
                }

                /**
Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition
for the overall aircraft, not only for one component.
*/
                class SET_MODE
                {
                    /**
The system setting the mode
*/
                    byte target_system;

                    /**
The new base mode.
*/
                    MAV_MODE base_mode;

                    /**
The new autopilot-specific mode. This field can be ignored by an autopilot.
*/
                    uint custom_mode;
                }

                /**
value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation
of QGroundControl and IMU code.
*/
                class PARAM_REQUEST_READ
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as string
*/
                    string param_id;

                    /**
Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
*/
                    short param_index;
                }

                /**
Request all parameters of this component. After this request, all parameters are emitted. The parameter
microservice is documented at https://mavlink.io/en/services/parameter.html
*/
                class PARAM_REQUEST_LIST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;
                }

                /**
Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
the recipient to keep track of received parameters and allows him to re-request missing parameters after
a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
*/
                class PARAM_VALUE
                {
                    /**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as string
*/
                    string param_id;

                    /**
Onboard parameter value
*/
                    float param_value;

                    /**
Onboard parameter type.
*/
                    MAV_PARAM_TYPE param_type;

                    /**
Total number of onboard parameters
*/
                    ushort param_count;

                    /**
Index of this onboard parameter
*/
                    ushort param_index;
                }

                /**
PARAM_SET may also be called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION).
Within a transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter
component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not received.
*/
                class PARAM_SET
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as string
*/
                    string param_id;

                    /**
Onboard parameter value
*/
                    float param_value;

                    /**
Onboard parameter type.
*/
                    MAV_PARAM_TYPE param_type;
                }

                /**
The global position, as returned by the Global Positioning System (GPS). This is
                NOT the
global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION_INT
for the global position estimate.
*/
                class GPS_RAW_INT
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
GPS fix type.
*/
                    GPS_FIX_TYPE fix_type;

                    /**
Latitude (WGS84, EGM96 ellipsoid)
*/
                    int lat;

                    /**
Longitude (WGS84, EGM96 ellipsoid)
*/
                    int lon;

                    /**
Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition
to the WGS84 altitude.
*/
                    int alt;

                    /**
GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
*/
                    ushort eph;

                    /**
GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
*/
                    ushort epv;

                    /**
GPS ground speed. If unknown, set to: UINT16_MAX
*/
                    ushort vel;

                    /**
Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
unknown, set to: UINT16_MAX
*/
                    ushort cog;

                    /**
Number of satellites visible. If unknown, set to UINT8_MAX
*/
                    byte satellites_visible;

                    /**
Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
*/
                    int alt_ellipsoid;

                    /**
Position uncertainty.
*/
                    uint h_acc;

                    /**
Altitude uncertainty.
*/
                    uint v_acc;

                    /**
Speed uncertainty.
*/
                    uint vel_acc;

                    /**
Heading / track uncertainty
*/
                    uint hdg_acc;

                    /**
Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured
to provide yaw and is currently unable to provide it. Use 36000 for north.
*/
                    ushort yaw;
                }

                /**
The positioning status, as reported by GPS. This message is intended to display status information about
each satellite visible to the receiver. See message GLOBAL_POSITION_INT for the global position estimate.
This message can contain information for up to 20 satellites.
*/
                class GPS_STATUS
                {
                    /**
Number of satellites visible
*/
                    byte satellites_visible;

                    /**
Global satellite ID
*/
                    [Dims(+20)] byte satellite_prn;

                    /**
0: Satellite not used, 1: used for localization
*/
                    [Dims(+20)] byte satellite_used;

                    /**
Elevation (0: right on top of receiver, 90: on the horizon) of satellite
*/
                    [Dims(+20)] byte satellite_elevation;

                    /**
Direction of satellite, 0: 0 deg, 255: 360 deg.
*/
                    [Dims(+20)] byte satellite_azimuth;

                    /**
Signal to noise ratio of satellite
*/
                    [Dims(+20)] byte satellite_snr;
                }

                /**
The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
the described units
*/
                class SCALED_IMU
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
X acceleration
*/
                    short xacc;

                    /**
Y acceleration
*/
                    short yacc;

                    /**
Z acceleration
*/
                    short zacc;

                    /**
Angular speed around X axis
*/
                    short xgyro;

                    /**
Angular speed around Y axis
*/
                    short ygyro;

                    /**
Angular speed around Z axis
*/
                    short zgyro;

                    /**
X Magnetic field
*/
                    short xmag;

                    /**
Y Magnetic field
*/
                    short ymag;

                    /**
Z Magnetic field
*/
                    short zmag;

                    /**
Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
*/
                    short temperature;
                }

                /**
The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should
always contain the true raw values without any scaling to allow data capture and system debugging.
*/
                class RAW_IMU
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
X acceleration (raw)
*/
                    short xacc;

                    /**
Y acceleration (raw)
*/
                    short yacc;

                    /**
Z acceleration (raw)
*/
                    short zacc;

                    /**
Angular speed around X axis (raw)
*/
                    short xgyro;

                    /**
Angular speed around Y axis (raw)
*/
                    short ygyro;

                    /**
Angular speed around Z axis (raw)
*/
                    short zgyro;

                    /**
X Magnetic field (raw)
*/
                    short xmag;

                    /**
Y Magnetic field (raw)
*/
                    short ymag;

                    /**
Z Magnetic field (raw)
*/
                    short zmag;

                    /**
Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
*/
                    byte id;

                    /**
Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
*/
                    short temperature;
                }

                /**
The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
sensor. The sensor values should be the raw, UNSCALED ADC values.
*/
                class RAW_PRESSURE
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Absolute pressure (raw)
*/
                    short press_abs;

                    /**
Differential pressure 1 (raw, 0 if nonexistent)
*/
                    short press_diff1;

                    /**
Differential pressure 2 (raw, 0 if nonexistent)
*/
                    short press_diff2;

                    /**
Raw Temperature measurement (raw)
*/
                    short temperature;
                }

                /**
The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
are as specified in each field.
*/
                class SCALED_PRESSURE
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Absolute pressure
*/
                    float press_abs;

                    /**
Differential pressure 1
*/
                    float press_diff;

                    /**
Absolute pressure temperature
*/
                    short temperature;

                    /**
Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
*/
                    short temperature_press_diff;
                }

                /**
The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
*/
                class ATTITUDE
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Roll angle (-pi..+pi)
*/
                    float roll;

                    /**
Pitch angle (-pi..+pi)
*/
                    float pitch;

                    /**
Yaw angle (-pi..+pi)
*/
                    float yaw;

                    /**
Roll angular speed
*/
                    float rollspeed;

                    /**
Pitch angular speed
*/
                    float pitchspeed;

                    /**
Yaw angular speed
*/
                    float yawspeed;
                }

                /**
The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
*/
                class ATTITUDE_QUATERNION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Quaternion component 1, w (1 in null-rotation)
*/
                    float q1;

                    /**
Quaternion component 2, x (0 in null-rotation)
*/
                    float q2;

                    /**
Quaternion component 3, y (0 in null-rotation)
*/
                    float q3;

                    /**
Quaternion component 4, z (0 in null-rotation)
*/
                    float q4;

                    /**
Roll angular speed
*/
                    float rollspeed;

                    /**
Pitch angular speed
*/
                    float pitchspeed;

                    /**
Yaw angular speed
*/
                    float yawspeed;

                    /**
in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.
*/
                    [Dims(+4)] float repr_offset_q;
                }

                /**
The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
Z-axis down (aeronautical frame, NED / north-east-down convention)
*/
                class LOCAL_POSITION_NED
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
X Position
*/
                    float x;

                    /**
Y Position
*/
                    float y;

                    /**
Z Position
*/
                    float z;

                    /**
X Speed
*/
                    float vx;

                    /**
Y Speed
*/
                    float vy;

                    /**
Z Speed
*/
                    float vz;
                }

                /**
The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
Z-up). It
               is designed as scaled integer message since the resolution of float is not sufficient.
*/
                class GLOBAL_POSITION_INT
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Latitude, expressed
*/
                    int lat;

                    /**
Longitude, expressed
*/
                    int lon;

                    /**
Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
*/
                    int alt;

                    /**
Altitude above ground
*/
                    int relative_alt;

                    /**
Ground X Speed (Latitude, positive north)
*/
                    short vx;

                    /**
Ground Y Speed (Longitude, positive east)
*/
                    short vy;

                    /**
Ground Z Speed (Altitude, positive down)
*/
                    short vz;

                    /**
Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
*/
                    ushort hdg;
                }

                /**
The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
inactive should be set to UINT16_MAX.
*/
                class RC_CHANNELS_SCALED
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN,
1 = AUX.
*/
                    byte port;

                    /**
RC channel 1 value scaled.
*/
                    short chan1_scaled;

                    /**
RC channel 2 value scaled.
*/
                    short chan2_scaled;

                    /**
RC channel 3 value scaled.
*/
                    short chan3_scaled;

                    /**
RC channel 4 value scaled.
*/
                    short chan4_scaled;

                    /**
RC channel 5 value scaled.
*/
                    short chan5_scaled;

                    /**
RC channel 6 value scaled.
*/
                    short chan6_scaled;

                    /**
RC channel 7 value scaled.
*/
                    short chan7_scaled;

                    /**
RC channel 8 value scaled.
*/
                    short chan8_scaled;

                    /**
Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
*/
                    byte rssi;
                }

                /**
The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters
might violate this specification.
*/
                class RC_CHANNELS_RAW
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN,
1 = AUX.
*/
                    byte port;

                    /**
RC channel 1 value.
*/
                    ushort chan1_raw;

                    /**
RC channel 2 value.
*/
                    ushort chan2_raw;

                    /**
RC channel 3 value.
*/
                    ushort chan3_raw;

                    /**
RC channel 4 value.
*/
                    ushort chan4_raw;

                    /**
RC channel 5 value.
*/
                    ushort chan5_raw;

                    /**
RC channel 6 value.
*/
                    ushort chan6_raw;

                    /**
RC channel 7 value.
*/
                    ushort chan7_raw;

                    /**
RC channel 8 value.
*/
                    ushort chan8_raw;

                    /**
Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
*/
                    byte rssi;
                }

                /**
Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the remote,
use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000
microseconds: 100%.
*/
                class SERVO_OUTPUT_RAW
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    uint time_usec;

                    /**
Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN,
1 = AUX.
*/
                    byte port;

                    /**
Servo output 1 value
*/
                    ushort servo1_raw;

                    /**
Servo output 2 value
*/
                    ushort servo2_raw;

                    /**
Servo output 3 value
*/
                    ushort servo3_raw;

                    /**
Servo output 4 value
*/
                    ushort servo4_raw;

                    /**
Servo output 5 value
*/
                    ushort servo5_raw;

                    /**
Servo output 6 value
*/
                    ushort servo6_raw;

                    /**
Servo output 7 value
*/
                    ushort servo7_raw;

                    /**
Servo output 8 value
*/
                    ushort servo8_raw;

                    /**
Servo output 9 value
*/
                    ushort servo9_raw;

                    /**
Servo output 10 value
*/
                    ushort servo10_raw;

                    /**
Servo output 11 value
*/
                    ushort servo11_raw;

                    /**
Servo output 12 value
*/
                    ushort servo12_raw;

                    /**
Servo output 13 value
*/
                    ushort servo13_raw;

                    /**
Servo output 14 value
*/
                    ushort servo14_raw;

                    /**
Servo output 15 value
*/
                    ushort servo15_raw;

                    /**
Servo output 16 value
*/
                    ushort servo16_raw;
                }

                /**
Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html.
If start and end index are the same, just send one waypoint.
*/
                class MISSION_REQUEST_PARTIAL_LIST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Start index
*/
                    short start_index;

                    /**
End index, -1 by default (-1: send list to end). Else a valid index of the list
*/
                    short end_index;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
This message is sent to the MAV to write a partial list. If start index == end index, only one item will
be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
be REJECTED!
*/
                class MISSION_WRITE_PARTIAL_LIST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Start index. Must be smaller / equal to the largest index of the current onboard list.
*/
                    short start_index;

                    /**
End index, equal or greater than start index.
*/
                    short end_index;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
See also https://mavlink.io/en/services/mission.html.
*/
                class MISSION_ITEM
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Sequence
*/
                    ushort seq;

                    /**
The coordinate system of the waypoint.
*/
                    MAV_FRAME frame;

                    /**
The scheduled action for the waypoint.
*/
                    MAV_CMD command;

                    /**
false:0, true:1
*/
                    byte current;

                    /**
Autocontinue to next waypoint
*/
                    byte autocontinue;

                    /**
PARAM1, see MAV_CMD enum
*/
                    float param1;

                    /**
PARAM2, see MAV_CMD enum
*/
                    float param2;

                    /**
PARAM3, see MAV_CMD enum
*/
                    float param3;

                    /**
PARAM4, see MAV_CMD enum
*/
                    float param4;

                    /**
PARAM5 / local: X coordinate, global: latitude
*/
                    float x;

                    /**
PARAM6 / local: Y coordinate, global: longitude
*/
                    float y;

                    /**
PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
*/
                    float z;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
Request the information of the mission item with the sequence number seq. The response of the system to
this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
*/
                class MISSION_REQUEST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Sequence
*/
                    ushort seq;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
Set the mission item with sequence number seq as current item. This means that the MAV will continue to
this mission item on the shortest path (not following the mission items in-between).
*/
                class MISSION_SET_CURRENT
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Sequence
*/
                    ushort seq;
                }

                /**
Message that announces the sequence number of the current active mission item. The MAV will fly towards
this mission item.
*/
                class MISSION_CURRENT
                {
                    /**
Sequence
*/
                    ushort seq;
                }

                /**
Request the overall list of mission items from the system/component.
*/
                class MISSION_REQUEST_LIST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
*/
                class MISSION_COUNT
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Number of mission items in the sequence
*/
                    ushort count;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
Delete all mission items at once.
*/
                class MISSION_CLEAR_ALL
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
or (if the autocontinue on the WP was set) continue to the next waypoint.
*/
                class MISSION_ITEM_REACHED
                {
                    /**
Sequence
*/
                    ushort seq;
                }

                /**
Acknowledgment message during waypoint handling. The type field states if this message is a positive ack
(type=0) or if an error happened (type=non-zero).
*/
                class MISSION_ACK
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Mission result.
*/
                    MAV_MISSION_RESULT Typ;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
Sets the GPS co-ordinates of the vehicle local origin (0,0,0) position. Vehicle should emit GPS_GLOBAL_ORIGIN
irrespective of whether the origin is changed. This enables transform between the local coordinate frame
and the global (GPS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings
are connected and the MAV should move from in- to outdoor.
*/
                class SET_GPS_GLOBAL_ORIGIN
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Latitude (WGS84)
*/
                    int latitude;

                    /**
Longitude (WGS84)
*/
                    int longitude;

                    /**
Altitude (MSL). Positive for up.
*/
                    int altitude;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;
                }

                /**
Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local
position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.
*/
                class GPS_GLOBAL_ORIGIN
                {
                    /**
Latitude (WGS84)
*/
                    int latitude;

                    /**
Longitude (WGS84)
*/
                    int longitude;

                    /**
Altitude (MSL). Positive for up.
*/
                    int altitude;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;
                }

                /**
Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
*/
                class PARAM_MAP_RC
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
storage if the ID is stored as string
*/
                    string param_id;

                    /**
Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
send -2 to disable any existing map for this rc_channel_index.
*/
                    short param_index;

                    /**
Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a potentiometer-knob
on the RC.
*/
                    byte parameter_rc_channel_index;

                    /**
Initial parameter value
*/
                    float param_value0;

                    /**
Scale, maps the RC range [-1, 1] to a parameter value
*/
                    float scale;

                    /**
Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
on implementation)
*/
                    float param_value_min;

                    /**
Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
on implementation)
*/
                    float param_value_max;
                }

                /**
Request the information of the mission item with the sequence number seq. The response of the system to
this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
*/
                class MISSION_REQUEST_INT
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Sequence
*/
                    ushort seq;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
or competition regulations.
*/
                class SAFETY_SET_ALLOWED_AREA
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis
down.
*/
                    MAV_FRAME frame;

                    /**
x position 1 / Latitude 1
*/
                    float p1x;

                    /**
y position 1 / Longitude 1
*/
                    float p1y;

                    /**
z position 1 / Altitude 1
*/
                    float p1z;

                    /**
x position 2 / Latitude 2
*/
                    float p2x;

                    /**
y position 2 / Longitude 2
*/
                    float p2y;

                    /**
z position 2 / Altitude 2
*/
                    float p2z;
                }

                /**
Read out the safety zone the MAV currently assumes.
*/
                class SAFETY_ALLOWED_AREA
                {
                    /**
Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis
down.
*/
                    MAV_FRAME frame;

                    /**
x position 1 / Latitude 1
*/
                    float p1x;

                    /**
y position 1 / Longitude 1
*/
                    float p1y;

                    /**
z position 1 / Altitude 1
*/
                    float p1z;

                    /**
x position 2 / Latitude 2
*/
                    float p2x;

                    /**
y position 2 / Longitude 2
*/
                    float p2y;

                    /**
z position 2 / Altitude 2
*/
                    float p2z;
                }

                /**
The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
*/
                class ATTITUDE_QUATERNION_COV
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
*/
                    [Dims(+4)] float q;

                    /**
Roll angular speed
*/
                    float rollspeed;

                    /**
Pitch angular speed
*/
                    float pitchspeed;

                    /**
Yaw angular speed
*/
                    float yawspeed;

                    /**
Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries
are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first
element in the array.
*/
                    [Dims(+9)] float covariance;
                }

                /**
The state of the navigation and position controller.
*/
                class NAV_CONTROLLER_OUTPUT
                {
                    /**
Current desired roll
*/
                    float nav_roll;

                    /**
Current desired pitch
*/
                    float nav_pitch;

                    /**
Current desired heading
*/
                    short nav_bearing;

                    /**
Bearing to current waypoint/target
*/
                    short target_bearing;

                    /**
Distance to active waypoint
*/
                    ushort wp_dist;

                    /**
Current altitude error
*/
                    float alt_error;

                    /**
Current airspeed error
*/
                    float aspd_error;

                    /**
Current crosstrack error on x-y plane
*/
                    float xtrack_error;
                }

                /**
The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
*/
                class GLOBAL_POSITION_INT_COV
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Class id of the estimator this estimate originated from.
*/
                    MAV_ESTIMATOR_TYPE estimator_type;

                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;

                    /**
Altitude in meters above MSL
*/
                    int alt;

                    /**
Altitude above ground
*/
                    int relative_alt;

                    /**
Ground X Speed (Latitude)
*/
                    float vx;

                    /**
Ground Y Speed (Longitude)
*/
                    float vy;

                    /**
Ground Z Speed (Altitude)
*/
                    float vz;

                    /**
Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon,
alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If
unknown, assign NaN value to first element in the array.
*/
                    [Dims(+36)] float covariance;
                }

                /**
The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
Z-axis down (aeronautical frame, NED / north-east-down convention)
*/
                class LOCAL_POSITION_NED_COV
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Class id of the estimator this estimate originated from.
*/
                    MAV_ESTIMATOR_TYPE estimator_type;

                    /**
X Position
*/
                    float x;

                    /**
Y Position
*/
                    float y;

                    /**
Z Position
*/
                    float z;

                    /**
X Speed
*/
                    float vx;

                    /**
Y Speed
*/
                    float vy;

                    /**
Z Speed
*/
                    float vz;

                    /**
X Acceleration
*/
                    float ax;

                    /**
Y Acceleration
*/
                    float ay;

                    /**
Z Acceleration
*/
                    float az;

                    /**
Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right
triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries
are the second row, etc.). If unknown, assign NaN value to first element in the array.
*/
                    [Dims(+45)] float covariance;
                }

                /**
The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters
might violate this specification.
*/
                class RC_CHANNELS
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Total number of RC channels being received. This can be larger than 18, indicating that more channels
are available but not given in this message. This value should be 0 when no RC channels are available.
*/
                    byte chancount;

                    /**
RC channel 1 value.
*/
                    ushort chan1_raw;

                    /**
RC channel 2 value.
*/
                    ushort chan2_raw;

                    /**
RC channel 3 value.
*/
                    ushort chan3_raw;

                    /**
RC channel 4 value.
*/
                    ushort chan4_raw;

                    /**
RC channel 5 value.
*/
                    ushort chan5_raw;

                    /**
RC channel 6 value.
*/
                    ushort chan6_raw;

                    /**
RC channel 7 value.
*/
                    ushort chan7_raw;

                    /**
RC channel 8 value.
*/
                    ushort chan8_raw;

                    /**
RC channel 9 value.
*/
                    ushort chan9_raw;

                    /**
RC channel 10 value.
*/
                    ushort chan10_raw;

                    /**
RC channel 11 value.
*/
                    ushort chan11_raw;

                    /**
RC channel 12 value.
*/
                    ushort chan12_raw;

                    /**
RC channel 13 value.
*/
                    ushort chan13_raw;

                    /**
RC channel 14 value.
*/
                    ushort chan14_raw;

                    /**
RC channel 15 value.
*/
                    ushort chan15_raw;

                    /**
RC channel 16 value.
*/
                    ushort chan16_raw;

                    /**
RC channel 17 value.
*/
                    ushort chan17_raw;

                    /**
RC channel 18 value.
*/
                    ushort chan18_raw;

                    /**
Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
*/
                    byte rssi;
                }

                /**
Request a data stream.
*/
                class REQUEST_DATA_STREAM
                {
                    /**
The target requested to send the message stream.
*/
                    byte target_system;

                    /**
The target requested to send the message stream.
*/
                    byte target_component;

                    /**
The ID of the requested data stream
*/
                    byte req_stream_id;

                    /**
The requested message rate
*/
                    ushort req_message_rate;

                    /**
1 to start sending, 0 to stop sending.
*/
                    byte start_stop;
                }

                /**
Data stream status information.
*/
                class DATA_STREAM
                {
                    /**
The ID of the requested data stream
*/
                    byte stream_id;

                    /**
The message rate
*/
                    ushort message_rate;

                    /**
1 stream is enabled, 0 stream is stopped.
*/
                    byte on_off;
                }

                /**
This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
along with a joystick-like input device. Unused axes can be disabled and buttons states are transmitted
as individual on/off bits of a bitmask
*/
                class MANUAL_CONTROL
                {
                    /**
The system to be controlled.
*/
                    byte target;

                    /**
X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
*/
                    short x;

                    /**
Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
*/
                    short y;

                    /**
Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
thrust.
*/
                    short z;

                    /**
R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
being -1000, and the yaw of a vehicle.
*/
                    short r;

                    /**
A bitfield corresponding to the joystick buttons' 0-15 current state, 1 for pressed, 0 for released. The
lowest bit corresponds to Button 1.
*/
                    ushort buttons;

                    /**
A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released.
The lowest bit corresponds to Button 16.
*/
                    ushort buttons2;

                    /**
Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit
1: roll.
*/
                    byte enabled_extensions;

                    /**
Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with
additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
*/
                    short s;

                    /**
Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional
degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
*/
                    short t;
                }

                /**
The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The standard
PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters
might violate this specification.  Note carefully the semantic differences between the first 8 channels
and the subsequent channels
*/
                class RC_CHANNELS_OVERRIDE
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan1_raw;

                    /**
RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan2_raw;

                    /**
RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan3_raw;

                    /**
RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan4_raw;

                    /**
RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan5_raw;

                    /**
RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan6_raw;

                    /**
RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan7_raw;

                    /**
RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this
channel back to the RC radio.
*/
                    ushort chan8_raw;

                    /**
RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan9_raw;

                    /**
RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan10_raw;

                    /**
RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan11_raw;

                    /**
RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan12_raw;

                    /**
RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan13_raw;

                    /**
RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan14_raw;

                    /**
RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan15_raw;

                    /**
RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan16_raw;

                    /**
RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan17_raw;

                    /**
RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means
to release this channel back to the RC radio.
*/
                    ushort chan18_raw;
                }

                /**
Message encoding a mission item. This message is emitted to announce
                the presence of a
mission item and to set a mission item on the system. The mission item can be either in x, y, z meters
(type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is
Z-up, right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate
optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value).
See also https://mavlink.io/en/services/mission.html.
*/
                class MISSION_ITEM_INT
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
sequence (0,1,2,3,4).
*/
                    ushort seq;

                    /**
The coordinate system of the waypoint.
*/
                    MAV_FRAME frame;

                    /**
The scheduled action for the waypoint.
*/
                    MAV_CMD command;

                    /**
false:0, true:1
*/
                    byte current;

                    /**
Autocontinue to next waypoint
*/
                    byte autocontinue;

                    /**
PARAM1, see MAV_CMD enum
*/
                    float param1;

                    /**
PARAM2, see MAV_CMD enum
*/
                    float param2;

                    /**
PARAM3, see MAV_CMD enum
*/
                    float param3;

                    /**
PARAM4, see MAV_CMD enum
*/
                    float param4;

                    /**
PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
*/
                    int x;

                    /**
PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
*/
                    int y;

                    /**
PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
*/
                    float z;

                    /**
Mission type.
*/
                    MAV_MISSION_TYPE mission_type;
                }

                /**
Metrics typically displayed on a HUD for fixed wing aircraft.
*/
                class VFR_HUD
                {
                    /**
Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated
airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall
speed.
*/
                    float airspeed;

                    /**
Current ground speed.
*/
                    float groundspeed;

                    /**
Current heading in compass units (0-360, 0=north).
*/
                    short heading;

                    /**
Current throttle setting (0 to 100).
*/
                    ushort throttle;

                    /**
Current altitude (MSL).
*/
                    float alt;

                    /**
Current climb rate.
*/
                    float climb;
                }

                /**
Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values
(e.g. to use the component's current latitude, yaw rather than a specific value). The command microservice
is documented at https://mavlink.io/en/services/command.html
*/
                class COMMAND_INT
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
The coordinate system of the COMMAND.
*/
                    MAV_FRAME frame;

                    /**
The scheduled action for the mission item.
*/
                    MAV_CMD command;

                    /**
Not used.
*/
                    byte current;

                    /**
Not used (set 0).
*/
                    byte autocontinue;

                    /**
PARAM1, see MAV_CMD enum
*/
                    float param1;

                    /**
PARAM2, see MAV_CMD enum
*/
                    float param2;

                    /**
PARAM3, see MAV_CMD enum
*/
                    float param3;

                    /**
PARAM4, see MAV_CMD enum
*/
                    float param4;

                    /**
PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
*/
                    int x;

                    /**
PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
*/
                    int y;

                    /**
PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
*/
                    float z;
                }

                /**
Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.html
*/
                class COMMAND_LONG
                {
                    /**
System which should execute the command
*/
                    byte target_system;

                    /**
Component which should execute the command, 0 for all components
*/
                    byte target_component;

                    /**
Command ID (of command to send).
*/
                    MAV_CMD command;

                    /**
0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
*/
                    byte confirmation;

                    /**
Parameter 1 (for the specific command).
*/
                    float param1;

                    /**
Parameter 2 (for the specific command).
*/
                    float param2;

                    /**
Parameter 3 (for the specific command).
*/
                    float param3;

                    /**
Parameter 4 (for the specific command).
*/
                    float param4;

                    /**
Parameter 5 (for the specific command).
*/
                    float param5;

                    /**
Parameter 6 (for the specific command).
*/
                    float param6;

                    /**
Parameter 7 (for the specific command).
*/
                    float param7;
                }

                /**
Report status of a command. Includes feedback whether the command was executed. The command microservice
is documented at https://mavlink.io/en/services/command.html
*/
                class COMMAND_ACK
                {
                    /**
Command ID (of acknowledged command).
*/
                    MAV_CMD command;

                    /**
Result of command.
*/
                    MAV_RESULT result;

                    /**
Also used as result_param1, it can be set with an enum containing the errors reasons of why the command
was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (UINT8_MAX if the progress
is unknown).
*/
                    byte progress;

                    /**
Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
*/
                    int result_param2;

                    /**
System ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK
is an acknowledgement.
*/
                    byte target_system;

                    /**
Component ID of the target recipient. This is the ID of the system that sent the command for which this
COMMAND_ACK is an acknowledgement.
*/
                    byte target_component;
                }

                /**
Cancel a long running command. The target system should respond with a COMMAND_ACK to the original command
with result=MAV_RESULT_CANCELLED if the long running process was cancelled. If it has already completed,
the cancel action can be ignored. The cancel action can be retried until some sort of acknowledgement
to the original command has been received. The command microservice is documented at https://mavlink.io/en/services/command.html
*/
                class COMMAND_CANCEL
                {
                    /**
System executing long running command. Should not be broadcast (0).
*/
                    byte target_system;

                    /**
Component executing long running command.
*/
                    byte target_component;

                    /**
Command ID (of command to cancel).
*/
                    MAV_CMD command;
                }

                /**
Setpoint in roll, pitch, yaw and thrust from the operator
*/
                class MANUAL_SETPOINT
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Desired roll rate
*/
                    float roll;

                    /**
Desired pitch rate
*/
                    float pitch;

                    /**
Desired yaw rate
*/
                    float yaw;

                    /**
Collective thrust, normalized to 0 .. 1
*/
                    float thrust;

                    /**
Flight mode switch position, 0.. 255
*/
                    byte mode_switch;

                    /**
Override mode switch position, 0.. 255
*/
                    byte manual_override_switch;
                }

                /**
Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
or other system).
*/
                class SET_ATTITUDE_TARGET
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Bitmap to indicate which dimensions should be ignored by the vehicle.
*/
                    ATTITUDE_TARGET_TYPEMASK type_mask;

                    /**
Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
*/
                    [Dims(+4)] float q;

                    /**
Body roll rate
*/
                    float body_roll_rate;

                    /**
Body pitch rate
*/
                    float body_pitch_rate;

                    /**
Body yaw rate
*/
                    float body_yaw_rate;

                    /**
Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
*/
                    float thrust;

                    /**
3D thrust setpoint in the body NED frame, normalized to -1 .. 1
*/
                    [Dims(+3)] float thrust_body;
                }

                /**
Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
*/
                class ATTITUDE_TARGET
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Bitmap to indicate which dimensions should be ignored by the vehicle.
*/
                    ATTITUDE_TARGET_TYPEMASK type_mask;

                    /**
Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
*/
                    [Dims(+4)] float q;

                    /**
Body roll rate
*/
                    float body_roll_rate;

                    /**
Body pitch rate
*/
                    float body_pitch_rate;

                    /**
Body yaw rate
*/
                    float body_yaw_rate;

                    /**
Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
*/
                    float thrust;
                }

                /**
Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
to command the vehicle (manual controller or other system).
*/
                class SET_POSITION_TARGET_LOCAL_NED
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
= 9
*/
                    MAV_FRAME coordinate_frame;

                    /**
Bitmap to indicate which dimensions should be ignored by the vehicle.
*/
                    POSITION_TARGET_TYPEMASK type_mask;

                    /**
X Position in NED frame
*/
                    float x;

                    /**
Y Position in NED frame
*/
                    float y;

                    /**
Z Position in NED frame (note, altitude is negative in NED)
*/
                    float z;

                    /**
X velocity in NED frame
*/
                    float vx;

                    /**
Y velocity in NED frame
*/
                    float vy;

                    /**
Z velocity in NED frame
*/
                    float vz;

                    /**
X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afx;

                    /**
Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afy;

                    /**
Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afz;

                    /**
yaw setpoint
*/
                    float yaw;

                    /**
yaw rate setpoint
*/
                    float yaw_rate;
                }

                /**
Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
this way.
*/
                class POSITION_TARGET_LOCAL_NED
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
= 9
*/
                    MAV_FRAME coordinate_frame;

                    /**
Bitmap to indicate which dimensions should be ignored by the vehicle.
*/
                    POSITION_TARGET_TYPEMASK type_mask;

                    /**
X Position in NED frame
*/
                    float x;

                    /**
Y Position in NED frame
*/
                    float y;

                    /**
Z Position in NED frame (note, altitude is negative in NED)
*/
                    float z;

                    /**
X velocity in NED frame
*/
                    float vx;

                    /**
Y velocity in NED frame
*/
                    float vy;

                    /**
Z velocity in NED frame
*/
                    float vz;

                    /**
X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afx;

                    /**
Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afy;

                    /**
Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afz;

                    /**
yaw setpoint
*/
                    float yaw;

                    /**
yaw rate setpoint
*/
                    float yaw_rate;
                }

                /**
Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
Used by an external controller to command the vehicle (manual controller or other system).
*/
                class SET_POSITION_TARGET_GLOBAL_INT
                {
                    /**
Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system
to compensate for the transport delay of the setpoint. This allows the system to compensate processing
latency.
*/
                    uint time_boot_ms;

                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
= 11
*/
                    MAV_FRAME coordinate_frame;

                    /**
Bitmap to indicate which dimensions should be ignored by the vehicle.
*/
                    POSITION_TARGET_TYPEMASK type_mask;

                    /**
X Position in WGS84 frame
*/
                    int lat_int;

                    /**
Y Position in WGS84 frame
*/
                    int lon_int;

                    /**
Altitude (MSL, Relative to home, or AGL - depending on frame)
*/
                    float alt;

                    /**
X velocity in NED frame
*/
                    float vx;

                    /**
Y velocity in NED frame
*/
                    float vy;

                    /**
Z velocity in NED frame
*/
                    float vz;

                    /**
X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afx;

                    /**
Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afy;

                    /**
Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afz;

                    /**
yaw setpoint
*/
                    float yaw;

                    /**
yaw rate setpoint
*/
                    float yaw_rate;
                }

                /**
Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
this way.
*/
                class POSITION_TARGET_GLOBAL_INT
                {
                    /**
Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system
to compensate for the transport delay of the setpoint. This allows the system to compensate processing
latency.
*/
                    uint time_boot_ms;

                    /**
Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
= 11
*/
                    MAV_FRAME coordinate_frame;

                    /**
Bitmap to indicate which dimensions should be ignored by the vehicle.
*/
                    POSITION_TARGET_TYPEMASK type_mask;

                    /**
X Position in WGS84 frame
*/
                    int lat_int;

                    /**
Y Position in WGS84 frame
*/
                    int lon_int;

                    /**
Altitude (MSL, AGL or relative to home altitude, depending on frame)
*/
                    float alt;

                    /**
X velocity in NED frame
*/
                    float vx;

                    /**
Y velocity in NED frame
*/
                    float vy;

                    /**
Z velocity in NED frame
*/
                    float vz;

                    /**
X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afx;

                    /**
Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afy;

                    /**
Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
*/
                    float afz;

                    /**
yaw setpoint
*/
                    float yaw;

                    /**
yaw rate setpoint
*/
                    float yaw_rate;
                }

                /**
The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
convention)
*/
                class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
X Position
*/
                    float x;

                    /**
Y Position
*/
                    float y;

                    /**
Z Position
*/
                    float z;

                    /**
Roll
*/
                    float roll;

                    /**
Pitch
*/
                    float pitch;

                    /**
Yaw
*/
                    float yaw;
                }

                /**
Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware
in the loop simulations.
*/
                class HIL_STATE
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Roll angle
*/
                    float roll;

                    /**
Pitch angle
*/
                    float pitch;

                    /**
Yaw angle
*/
                    float yaw;

                    /**
Body frame roll / phi angular speed
*/
                    float rollspeed;

                    /**
Body frame pitch / theta angular speed
*/
                    float pitchspeed;

                    /**
Body frame yaw / psi angular speed
*/
                    float yawspeed;

                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;

                    /**
Altitude
*/
                    int alt;

                    /**
Ground X Speed (Latitude)
*/
                    short vx;

                    /**
Ground Y Speed (Longitude)
*/
                    short vy;

                    /**
Ground Z Speed (Altitude)
*/
                    short vz;

                    /**
X acceleration
*/
                    short xacc;

                    /**
Y acceleration
*/
                    short yacc;

                    /**
Z acceleration
*/
                    short zacc;
                }

                /**
Sent from autopilot to simulation. Hardware in the loop control outputs
*/
                class HIL_CONTROLS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Control output -1 .. 1
*/
                    float roll_ailerons;

                    /**
Control output -1 .. 1
*/
                    float pitch_elevator;

                    /**
Control output -1 .. 1
*/
                    float yaw_rudder;

                    /**
Throttle 0 .. 1
*/
                    float throttle;

                    /**
Aux 1, -1 .. 1
*/
                    float aux1;

                    /**
Aux 2, -1 .. 1
*/
                    float aux2;

                    /**
Aux 3, -1 .. 1
*/
                    float aux3;

                    /**
Aux 4, -1 .. 1
*/
                    float aux4;

                    /**
System mode.
*/
                    MAV_MODE mode;

                    /**
Navigation mode (MAV_NAV_MODE)
*/
                    byte nav_mode;
                }

                /**
Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
violate this specification.
*/
                class HIL_RC_INPUTS_RAW
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
RC channel 1 value
*/
                    ushort chan1_raw;

                    /**
RC channel 2 value
*/
                    ushort chan2_raw;

                    /**
RC channel 3 value
*/
                    ushort chan3_raw;

                    /**
RC channel 4 value
*/
                    ushort chan4_raw;

                    /**
RC channel 5 value
*/
                    ushort chan5_raw;

                    /**
RC channel 6 value
*/
                    ushort chan6_raw;

                    /**
RC channel 7 value
*/
                    ushort chan7_raw;

                    /**
RC channel 8 value
*/
                    ushort chan8_raw;

                    /**
RC channel 9 value
*/
                    ushort chan9_raw;

                    /**
RC channel 10 value
*/
                    ushort chan10_raw;

                    /**
RC channel 11 value
*/
                    ushort chan11_raw;

                    /**
RC channel 12 value
*/
                    ushort chan12_raw;

                    /**
Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
*/
                    byte rssi;
                }

                /**
Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
*/
                class HIL_ACTUATOR_CONTROLS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
*/
                    [Dims(+16)] float controls;

                    /**
System mode. Includes arming state.
*/
                    MAV_MODE_FLAG mode;

                    /**
Flags as bitfield, 1: indicate simulation using lockstep.
*/
                    ulong flags;
                }

                /**
Optical flow from a flow sensor (e.g. optical mouse sensor)
*/
                class OPTICAL_FLOW
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Sensor ID
*/
                    byte sensor_id;

                    /**
Flow in x-sensor direction
*/
                    short flow_x;

                    /**
Flow in y-sensor direction
*/
                    short flow_y;

                    /**
Flow in x-sensor direction, angular-speed compensated
*/
                    float flow_comp_m_x;

                    /**
Flow in y-sensor direction, angular-speed compensated
*/
                    float flow_comp_m_y;

                    /**
Optical flow quality / confidence. 0: bad, 255: maximum quality
*/
                    byte quality;

                    /**
Ground distance. Positive value: distance known. Negative value: Unknown distance
*/
                    float ground_distance;

                    /**
Flow rate about X axis
*/
                    float flow_rate_x;

                    /**
Flow rate about Y axis
*/
                    float flow_rate_y;
                }

                /**
Global position/attitude estimate from a vision source.
*/
                class GLOBAL_VISION_POSITION_ESTIMATE
                {
                    /**
Timestamp (UNIX time or since system boot)
*/
                    ulong usec;

                    /**
Global X position
*/
                    float x;

                    /**
Global Y position
*/
                    float y;

                    /**
Global Z position
*/
                    float z;

                    /**
Roll angle
*/
                    float roll;

                    /**
Pitch angle
*/
                    float pitch;

                    /**
Yaw angle
*/
                    float yaw;

                    /**
Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x_global, y_global,
z_global, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW,
etc.). If unknown, assign NaN value to first element in the array.
*/
                    [Dims(+21)] float covariance;

                    /**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps.
*/
                    byte reset_counter;
                }

                /**
Local position/attitude estimate from a vision source.
*/
                class VISION_POSITION_ESTIMATE
                {
                    /**
Timestamp (UNIX time or time since system boot)
*/
                    ulong usec;

                    /**
Local X position
*/
                    float x;

                    /**
Local Y position
*/
                    float y;

                    /**
Local Z position
*/
                    float z;

                    /**
Roll angle
*/
                    float roll;

                    /**
Pitch angle
*/
                    float pitch;

                    /**
Yaw angle
*/
                    float yaw;

                    /**
Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll,
pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown,
assign NaN value to first element in the array.
*/
                    [Dims(+21)] float covariance;

                    /**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps.
*/
                    byte reset_counter;
                }

                /**
Speed estimate from a vision source.
*/
                class VISION_SPEED_ESTIMATE
                {
                    /**
Timestamp (UNIX time or time since system boot)
*/
                    ulong usec;

                    /**
Global X speed
*/
                    float x;

                    /**
Global Y speed
*/
                    float y;

                    /**
Global Z speed
*/
                    float z;

                    /**
Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries
- 1st row, etc.). If unknown, assign NaN value to first element in the array.
*/
                    [Dims(+9)] float covariance;

                    /**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps.
*/
                    byte reset_counter;
                }

                /**
Global position estimate from a Vicon motion system source.
*/
                class VICON_POSITION_ESTIMATE
                {
                    /**
Timestamp (UNIX time or time since system boot)
*/
                    ulong usec;

                    /**
Global X position
*/
                    float x;

                    /**
Global Y position
*/
                    float y;

                    /**
Global Z position
*/
                    float z;

                    /**
Roll angle
*/
                    float roll;

                    /**
Pitch angle
*/
                    float pitch;

                    /**
Yaw angle
*/
                    float yaw;

                    /**
Row-major representation of 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll,
pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown,
assign NaN value to first element in the array.
*/
                    [Dims(+21)] float covariance;
                }

                /**
The IMU readings in SI units in NED body frame
*/
                class HIGHRES_IMU
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
X acceleration
*/
                    float xacc;

                    /**
Y acceleration
*/
                    float yacc;

                    /**
Z acceleration
*/
                    float zacc;

                    /**
Angular speed around X axis
*/
                    float xgyro;

                    /**
Angular speed around Y axis
*/
                    float ygyro;

                    /**
Angular speed around Z axis
*/
                    float zgyro;

                    /**
X Magnetic field
*/
                    float xmag;

                    /**
Y Magnetic field
*/
                    float ymag;

                    /**
Z Magnetic field
*/
                    float zmag;

                    /**
Absolute pressure
*/
                    float abs_pressure;

                    /**
Differential pressure
*/
                    float diff_pressure;

                    /**
Altitude calculated from pressure
*/
                    float pressure_alt;

                    /**
Temperature
*/
                    float temperature;

                    /**
Bitmap for fields that have updated since last message
*/
                    HIGHRES_IMU_UPDATED_FLAGS fields_updated;

                    /**
Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
*/
                    byte id;
                }

                /**
Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
*/
                class OPTICAL_FLOW_RAD
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Sensor ID
*/
                    byte sensor_id;

                    /**
Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow.
The integration time also indicates the.
*/
                    uint integration_time_us;

                    /**
Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion
along the positive Y axis induces a negative flow.)
*/
                    float integrated_x;

                    /**
Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion
along the positive X axis induces a positive flow.)
*/
                    float integrated_y;

                    /**
RH rotation around X axis
*/
                    float integrated_xgyro;

                    /**
RH rotation around Y axis
*/
                    float integrated_ygyro;

                    /**
RH rotation around Z axis
*/
                    float integrated_zgyro;

                    /**
Temperature
*/
                    short temperature;

                    /**
Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
*/
                    byte quality;

                    /**
Time since the distance was sampled.
*/
                    uint time_delta_distance_us;

                    /**
Distance to the center of the flow field. Positive value (including zero): distance known. Negative value:
Unknown distance.
*/
                    float distance;
                }

                /**
The IMU readings in SI units in NED body frame
*/
                class HIL_SENSOR
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
X acceleration
*/
                    float xacc;

                    /**
Y acceleration
*/
                    float yacc;

                    /**
Z acceleration
*/
                    float zacc;

                    /**
Angular speed around X axis in body frame
*/
                    float xgyro;

                    /**
Angular speed around Y axis in body frame
*/
                    float ygyro;

                    /**
Angular speed around Z axis in body frame
*/
                    float zgyro;

                    /**
X Magnetic field
*/
                    float xmag;

                    /**
Y Magnetic field
*/
                    float ymag;

                    /**
Z Magnetic field
*/
                    float zmag;

                    /**
Absolute pressure
*/
                    float abs_pressure;

                    /**
Differential pressure (airspeed)
*/
                    float diff_pressure;

                    /**
Altitude calculated from pressure
*/
                    float pressure_alt;

                    /**
Temperature
*/
                    float temperature;

                    /**
Bitmap for fields that have updated since last message
*/
                    HIL_SENSOR_UPDATED_FLAGS fields_updated;

                    /**
Sensor ID (zero indexed). Used for multiple sensor inputs
*/
                    byte id;
                }

                /**
Status of simulation environment, if used
*/
                class SIM_STATE
                {
                    /**
True attitude quaternion component 1, w (1 in null-rotation)
*/
                    float q1;

                    /**
True attitude quaternion component 2, x (0 in null-rotation)
*/
                    float q2;

                    /**
True attitude quaternion component 3, y (0 in null-rotation)
*/
                    float q3;

                    /**
True attitude quaternion component 4, z (0 in null-rotation)
*/
                    float q4;

                    /**
Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
*/
                    float roll;

                    /**
Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
*/
                    float pitch;

                    /**
Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
*/
                    float yaw;

                    /**
X acceleration
*/
                    float xacc;

                    /**
Y acceleration
*/
                    float yacc;

                    /**
Z acceleration
*/
                    float zacc;

                    /**
Angular speed around X axis
*/
                    float xgyro;

                    /**
Angular speed around Y axis
*/
                    float ygyro;

                    /**
Angular speed around Z axis
*/
                    float zgyro;

                    /**
Latitude
*/
                    float lat;

                    /**
Longitude
*/
                    float lon;

                    /**
Altitude
*/
                    float alt;

                    /**
Horizontal position standard deviation
*/
                    float std_dev_horz;

                    /**
Vertical position standard deviation
*/
                    float std_dev_vert;

                    /**
True velocity in north direction in earth-fixed NED frame
*/
                    float vn;

                    /**
True velocity in east direction in earth-fixed NED frame
*/
                    float ve;

                    /**
True velocity in down direction in earth-fixed NED frame
*/
                    float vd;
                }

                /**
Status generated by radio and injected into MAVLink stream.
*/
                class RADIO_STATUS
                {
                    /**
Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254],
UINT8_MAX: invalid/unknown.
*/
                    byte rssi;

                    /**
Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254],
UINT8_MAX: invalid/unknown.
*/
                    byte remrssi;

                    /**
Remaining free transmitter buffer space.
*/
                    byte txbuf;

                    /**
Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios).
Values: [0-254], UINT8_MAX: invalid/unknown.
*/
                    byte noise;

                    /**
Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios).
Values: [0-254], UINT8_MAX: invalid/unknown.
*/
                    byte remnoise;

                    /**
Count of radio packet receive errors (since boot).
*/
                    ushort rxerrors;

                    /**
Count of error corrected radio packets (since boot).
*/
                    ushort Fixe;
                }

                /**
File transfer protocol message: https://mavlink.io/en/services/ftp.html.
*/
                class FILE_TRANSFER_PROTOCOL
                {
                    /**
Network ID (0 for broadcast)
*/
                    byte target_network;

                    /**
System ID (0 for broadcast)
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast)
*/
                    byte target_component;

                    /**
Variable length payload. The length is defined by the remaining message length when subtracting the header
and other fields. The content/format of this block is defined in https://mavlink.io/en/services/ftp.html.
*/
                    [Dims(+251)] byte payload;
                }

                /**
Time synchronization message.
*/
                class TIMESYNC
                {
                    /**
Time sync timestamp 1
*/
                    long tc1;

                    /**
Time sync timestamp 2
*/
                    long ts1;
                }

                /**
Camera-IMU triggering and synchronisation message.
*/
                class CAMERA_TRIGGER
                {
                    /**
Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp
format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Image frame sequence
*/
                    uint seq;
                }

                /**
The global position, as returned by the Global Positioning System (GPS). This is
                 NOT
the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION_INT
for the global position estimate.
*/
                class HIL_GPS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
at least two, so always correctly fill in the fix.
*/
                    byte fix_type;

                    /**
Latitude (WGS84)
*/
                    int lat;

                    /**
Longitude (WGS84)
*/
                    int lon;

                    /**
Altitude (MSL). Positive for up.
*/
                    int alt;

                    /**
GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
*/
                    ushort eph;

                    /**
GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
*/
                    ushort epv;

                    /**
GPS ground speed. If unknown, set to: UINT16_MAX
*/
                    ushort vel;

                    /**
GPS velocity in north direction in earth-fixed NED frame
*/
                    short vn;

                    /**
GPS velocity in east direction in earth-fixed NED frame
*/
                    short ve;

                    /**
GPS velocity in down direction in earth-fixed NED frame
*/
                    short vd;

                    /**
Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to:
UINT16_MAX
*/
                    ushort cog;

                    /**
Number of satellites visible. If unknown, set to UINT8_MAX
*/
                    byte satellites_visible;

                    /**
GPS ID (zero indexed). Used for multiple GPS inputs
*/
                    byte id;

                    /**
Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
*/
                    ushort yaw;
                }

                /**
Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
*/
                class HIL_OPTICAL_FLOW
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Sensor ID
*/
                    byte sensor_id;

                    /**
Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow.
The integration time also indicates the.
*/
                    uint integration_time_us;

                    /**
Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
motion along the positive Y axis induces a negative flow.)
*/
                    float integrated_x;

                    /**
Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
motion along the positive X axis induces a positive flow.)
*/
                    float integrated_y;

                    /**
RH rotation around X axis
*/
                    float integrated_xgyro;

                    /**
RH rotation around Y axis
*/
                    float integrated_ygyro;

                    /**
RH rotation around Z axis
*/
                    float integrated_zgyro;

                    /**
Temperature
*/
                    short temperature;

                    /**
Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
*/
                    byte quality;

                    /**
Time since the distance was sampled.
*/
                    uint time_delta_distance_us;

                    /**
Distance to the center of the flow field. Positive value (including zero): distance known. Negative value:
Unknown distance.
*/
                    float distance;
                }

                /**
Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
for high throughput applications such as hardware in the loop simulations.
*/
                class HIL_STATE_QUATERNION
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
*/
                    [Dims(+4)] float attitude_quaternion;

                    /**
Body frame roll / phi angular speed
*/
                    float rollspeed;

                    /**
Body frame pitch / theta angular speed
*/
                    float pitchspeed;

                    /**
Body frame yaw / psi angular speed
*/
                    float yawspeed;

                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;

                    /**
Altitude
*/
                    int alt;

                    /**
Ground X Speed (Latitude)
*/
                    short vx;

                    /**
Ground Y Speed (Longitude)
*/
                    short vy;

                    /**
Ground Z Speed (Altitude)
*/
                    short vz;

                    /**
Indicated airspeed
*/
                    ushort ind_airspeed;

                    /**
True airspeed
*/
                    ushort true_airspeed;

                    /**
X acceleration
*/
                    short xacc;

                    /**
Y acceleration
*/
                    short yacc;

                    /**
Z acceleration
*/
                    short zacc;
                }

                /**
The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
the described units
*/
                class SCALED_IMU2
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
X acceleration
*/
                    short xacc;

                    /**
Y acceleration
*/
                    short yacc;

                    /**
Z acceleration
*/
                    short zacc;

                    /**
Angular speed around X axis
*/
                    short xgyro;

                    /**
Angular speed around Y axis
*/
                    short ygyro;

                    /**
Angular speed around Z axis
*/
                    short zgyro;

                    /**
X Magnetic field
*/
                    short xmag;

                    /**
Y Magnetic field
*/
                    short ymag;

                    /**
Z Magnetic field
*/
                    short zmag;

                    /**
Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
*/
                    short temperature;
                }

                /**
Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
is called. If there are no log files available this request shall be answered with one LOG_ENTRY message
with id = 0 and num_logs = 0.
*/
                class LOG_REQUEST_LIST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
First log id (0 for first available)
*/
                    ushort start;

                    /**
Last log id (0xffff for last available)
*/
                    ushort end;
                }

                /**
Reply to LOG_REQUEST_LIST
*/
                class LOG_ENTRY
                {
                    /**
Log id
*/
                    ushort id;

                    /**
Total number of logs
*/
                    ushort num_logs;

                    /**
High log number
*/
                    ushort last_log_num;

                    /**
UTC timestamp of log since 1970, or 0 if not available
*/
                    uint time_utc;

                    /**
Size of the log (may be approximate)
*/
                    uint size;
                }

                /**
Request a chunk of a log
*/
                class LOG_REQUEST_DATA
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Log id (from LOG_ENTRY reply)
*/
                    ushort id;

                    /**
Offset into the log
*/
                    uint ofs;

                    /**
Number of bytes
*/
                    uint count;
                }

                /**
Reply to LOG_REQUEST_DATA
*/
                class LOG_DATA
                {
                    /**
Log id (from LOG_ENTRY reply)
*/
                    ushort id;

                    /**
Offset into the log
*/
                    uint ofs;

                    /**
Number of bytes (zero for end of log)
*/
                    byte count;

                    /**
log data
*/
                    [Dims(+90)] byte Dat;
                }

                /**
Erase all logs
*/
                class LOG_ERASE
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;
                }

                /**
Stop log transfer and resume normal logging
*/
                class LOG_REQUEST_END
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;
                }

                /**
Data for injecting into the onboard GPS (used for DGPS)
*/
                class GPS_INJECT_DATA
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Data length
*/
                    byte len;

                    /**
Raw data (110 is enough for 12 satellites of RTCMv2)
*/
                    [Dims(+110)] byte Dat;
                }

                /**
Second GPS data.
*/
                class GPS2_RAW
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
GPS fix type.
*/
                    GPS_FIX_TYPE fix_type;

                    /**
Latitude (WGS84)
*/
                    int lat;

                    /**
Longitude (WGS84)
*/
                    int lon;

                    /**
Altitude (MSL). Positive for up.
*/
                    int alt;

                    /**
GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
*/
                    ushort eph;

                    /**
GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
*/
                    ushort epv;

                    /**
GPS ground speed. If unknown, set to: UINT16_MAX
*/
                    ushort vel;

                    /**
Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to:
UINT16_MAX
*/
                    ushort cog;

                    /**
Number of satellites visible. If unknown, set to UINT8_MAX
*/
                    byte satellites_visible;

                    /**
Number of DGPS satellites
*/
                    byte dgps_numch;

                    /**
Age of DGPS info
*/
                    uint dgps_age;

                    /**
Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured
to provide yaw and is currently unable to provide it. Use 36000 for north.
*/
                    ushort yaw;

                    /**
Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
*/
                    int alt_ellipsoid;

                    /**
Position uncertainty.
*/
                    uint h_acc;

                    /**
Altitude uncertainty.
*/
                    uint v_acc;

                    /**
Speed uncertainty.
*/
                    uint vel_acc;

                    /**
Heading / track uncertainty
*/
                    uint hdg_acc;
                }

                /**
Power supply status
*/
                class POWER_STATUS
                {
                    /**
5V rail voltage.
*/
                    ushort Vcc;

                    /**
Servo rail voltage.
*/
                    ushort Vservo;

                    /**
Bitmap of power supply status flags.
*/
                    MAV_POWER_STATUS flags;
                }

                /**
Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
or change the devices settings. A message with zero bytes can be used to change just the baudrate.
*/
                class SERIAL_CONTROL
                {
                    /**
Serial control device type.
*/
                    SERIAL_CONTROL_DEV device;

                    /**
Bitmap of serial control flags.
*/
                    SERIAL_CONTROL_FLAG flags;

                    /**
Timeout for reply data
*/
                    ushort timeout;

                    /**
Baudrate of transfer. Zero means no change.
*/
                    uint baudrate;

                    /**
how many bytes in this transfer
*/
                    byte count;

                    /**
serial data
*/
                    [Dims(+70)] byte Dat;

                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;
                }

                /**
RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
*/
                class GPS_RTK
                {
                    /**
Time since boot of last baseline message received.
*/
                    uint time_last_baseline_ms;

                    /**
Identification of connected RTK receiver.
*/
                    byte rtk_receiver_id;

                    /**
GPS Week Number of last baseline
*/
                    ushort wn;

                    /**
GPS Time of Week of last baseline
*/
                    uint tow;

                    /**
GPS-specific health report for RTK data.
*/
                    byte rtk_health;

                    /**
Rate of baseline messages being received by GPS
*/
                    byte rtk_rate;

                    /**
Current number of sats used for RTK calculation.
*/
                    byte nsats;

                    /**
Coordinate system of baseline
*/
                    RTK_BASELINE_COORDINATE_SYSTEM baseline_coords_type;

                    /**
Current baseline in ECEF x or NED north component.
*/
                    int baseline_a_mm;

                    /**
Current baseline in ECEF y or NED east component.
*/
                    int baseline_b_mm;

                    /**
Current baseline in ECEF z or NED down component.
*/
                    int baseline_c_mm;

                    /**
Current estimate of baseline accuracy.
*/
                    uint accuracy;

                    /**
Current number of integer ambiguity hypotheses.
*/
                    int iar_num_hypotheses;
                }

                /**
RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
*/
                class GPS2_RTK
                {
                    /**
Time since boot of last baseline message received.
*/
                    uint time_last_baseline_ms;

                    /**
Identification of connected RTK receiver.
*/
                    byte rtk_receiver_id;

                    /**
GPS Week Number of last baseline
*/
                    ushort wn;

                    /**
GPS Time of Week of last baseline
*/
                    uint tow;

                    /**
GPS-specific health report for RTK data.
*/
                    byte rtk_health;

                    /**
Rate of baseline messages being received by GPS
*/
                    byte rtk_rate;

                    /**
Current number of sats used for RTK calculation.
*/
                    byte nsats;

                    /**
Coordinate system of baseline
*/
                    RTK_BASELINE_COORDINATE_SYSTEM baseline_coords_type;

                    /**
Current baseline in ECEF x or NED north component.
*/
                    int baseline_a_mm;

                    /**
Current baseline in ECEF y or NED east component.
*/
                    int baseline_b_mm;

                    /**
Current baseline in ECEF z or NED down component.
*/
                    int baseline_c_mm;

                    /**
Current estimate of baseline accuracy.
*/
                    uint accuracy;

                    /**
Current number of integer ambiguity hypotheses.
*/
                    int iar_num_hypotheses;
                }

                /**
The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
units
*/
                class SCALED_IMU3
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
X acceleration
*/
                    short xacc;

                    /**
Y acceleration
*/
                    short yacc;

                    /**
Z acceleration
*/
                    short zacc;

                    /**
Angular speed around X axis
*/
                    short xgyro;

                    /**
Angular speed around Y axis
*/
                    short ygyro;

                    /**
Angular speed around Z axis
*/
                    short zgyro;

                    /**
X Magnetic field
*/
                    short xmag;

                    /**
Y Magnetic field
*/
                    short ymag;

                    /**
Z Magnetic field
*/
                    short zmag;

                    /**
Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
*/
                    short temperature;
                }

                /**
Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol:
https://mavlink.io/en/services/image_transmission.html.
*/
                class DATA_TRANSMISSION_HANDSHAKE
                {
                    /**
Type of requested/acknowledged data.
*/
                    MAVLINK_DATA_STREAM_TYPE Typ;

                    /**
total data size (set on ACK only).
*/
                    uint size;

                    /**
Width of a matrix or image.
*/
                    ushort width;

                    /**
Height of a matrix or image.
*/
                    ushort height;

                    /**
Number of packets being sent (set on ACK only).
*/
                    ushort packets;

                    /**
Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
ACK only).
*/
                    byte payload;

                    /**
JPEG quality. Values: [1-100].
*/
                    byte jpg_quality;
                }

                /**
Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
*/
                class ENCAPSULATED_DATA
                {
                    /**
sequence number (starting with 0 on every transmission)
*/
                    ushort seqnr;

                    /**
image data bytes
*/
                    [Dims(+253)] byte Dat;
                }

                /**
Distance sensor information for an onboard rangefinder.
*/
                class DISTANCE_SENSOR
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Minimum distance the sensor can measure
*/
                    ushort min_distance;

                    /**
Maximum distance the sensor can measure
*/
                    ushort max_distance;

                    /**
Current distance reading
*/
                    ushort current_distance;

                    /**
Type of distance sensor.
*/
                    MAV_DISTANCE_SENSOR Typ;

                    /**
Onboard ID of the sensor
*/
                    byte id;

                    /**
Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing:
ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
*/
                    MAV_SENSOR_ORIENTATION orientation;

                    /**
Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.
*/
                    byte covariance;

                    /**
Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known.
Otherwise this is set to 0.
*/
                    float horizontal_fov;

                    /**
Vertical Field of View (angle) where the distance measurement is valid and the field of view is known.
Otherwise this is set to 0.
*/
                    float vertical_fov;

                    /**
Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0,
0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set
to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."
*/
                    [Dims(+4)] float quaternion;

                    /**
Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength
with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset
signal quality, 1 = invalid signal, 100 = perfect signal.
*/
                    byte signal_quality;
                }

                /**
Request for terrain data and terrain status. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
*/
                class TERRAIN_REQUEST
                {
                    /**
Latitude of SW corner of first grid
*/
                    int lat;

                    /**
Longitude of SW corner of first grid
*/
                    int lon;

                    /**
Grid spacing
*/
                    ushort grid_spacing;

                    /**
Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
*/
                    ulong mask;
                }

                /**
Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST.
See terrain protocol docs: https://mavlink.io/en/services/terrain.html
*/
                class TERRAIN_DATA
                {
                    /**
Latitude of SW corner of first grid
*/
                    int lat;

                    /**
Longitude of SW corner of first grid
*/
                    int lon;

                    /**
Grid spacing
*/
                    ushort grid_spacing;

                    /**
bit within the terrain request mask
*/
                    byte gridbit;

                    /**
Terrain data MSL
*/
                    [Dims(+16)] short Dat;
                }

                /**
Request that the vehicle report terrain height at the given location (expected response is a TERRAIN_REPORT).
Used by GCS to check if vehicle has all terrain data needed for a mission.
*/
                class TERRAIN_CHECK
                {
                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;
                }

                /**
Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or sent
as a response to a TERRAIN_CHECK request. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
*/
                class TERRAIN_REPORT
                {
                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;

                    /**
grid spacing (zero if terrain at this location unavailable)
*/
                    ushort spacing;

                    /**
Terrain height MSL
*/
                    float terrain_height;

                    /**
Current vehicle height above lat/lon terrain height
*/
                    float current_height;

                    /**
Number of 4x4 terrain blocks waiting to be received or read from disk
*/
                    ushort pending;

                    /**
Number of 4x4 terrain blocks in memory
*/
                    ushort loaded;
                }

                /**
Barometer readings for 2nd barometer
*/
                class SCALED_PRESSURE2
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Absolute pressure
*/
                    float press_abs;

                    /**
Differential pressure
*/
                    float press_diff;

                    /**
Absolute pressure temperature
*/
                    short temperature;

                    /**
Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
*/
                    short temperature_press_diff;
                }

                /**
Motion capture attitude and position
*/
                class ATT_POS_MOCAP
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
*/
                    [Dims(+4)] float q;

                    /**
X position (NED)
*/
                    float x;

                    /**
Y position (NED)
*/
                    float y;

                    /**
Z position (NED)
*/
                    float z;

                    /**
Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z,
roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If
unknown, assign NaN value to first element in the array.
*/
                    [Dims(+21)] float covariance;
                }

                /**
Set the vehicle attitude and body angular rates.
*/
                class SET_ACTUATOR_CONTROL_TARGET
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
this field to difference between instances.
*/
                    byte group_mlx;

                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
mixer to repurpose them as generic outputs.
*/
                    [Dims(+8)] float controls;
                }

                /**
Set the vehicle attitude and body angular rates.
*/
                class ACTUATOR_CONTROL_TARGET
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
this field to difference between instances.
*/
                    byte group_mlx;

                    /**
Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
mixer to repurpose them as generic outputs.
*/
                    [Dims(+8)] float controls;
                }

                /**
The current system altitude.
*/
                class ALTITUDE
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
local altitude change). The only guarantee on this field is that it will never be reset and is consistent
within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
time. This altitude will also drift and vary between flights.
*/
                    float altitude_monotonic;

                    /**
This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output MSL
by default and not the WGS84 altitude.
*/
                    float altitude_amsl;

                    /**
This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
to the coordinate origin (0, 0, 0). It is up-positive.
*/
                    float altitude_local;

                    /**
This is the altitude above the home position. It resets on each change of the current home position.
*/
                    float altitude_relative;

                    /**
This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
than -1000 should be interpreted as unknown.
*/
                    float altitude_terrain;

                    /**
This is not the altitude, but the clear space below the system according to the fused clearance estimate.
It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
target. A negative value indicates no measurement available.
*/
                    float bottom_clearance;
                }

                /**
The autopilot is requesting a resource (file, binary, other type of data)
*/
                class RESOURCE_REQUEST
                {
                    /**
Request ID. This ID should be re-used when sending back URI contents
*/
                    byte request_id;

                    /**
The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
*/
                    byte uri_type;

                    /**
The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
on the URI type enum)
*/
                    [Dims(+120)] byte uri;

                    /**
The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
*/
                    byte transfer_type;

                    /**
The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
has a storage associated (e.g. MAVLink FTP).
*/
                    [Dims(+120)] byte storage;
                }

                /**
Barometer readings for 3rd barometer
*/
                class SCALED_PRESSURE3
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Absolute pressure
*/
                    float press_abs;

                    /**
Differential pressure
*/
                    float press_diff;

                    /**
Absolute pressure temperature
*/
                    short temperature;

                    /**
Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
*/
                    short temperature_press_diff;
                }

                /**
Current motion information from a designated system
*/
                class FOLLOW_TARGET
                {
                    /**
Timestamp (time since system boot).
*/
                    ulong timestamp;

                    /**
bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
*/
                    byte est_capabilities;

                    /**
Latitude (WGS84)
*/
                    int lat;

                    /**
Longitude (WGS84)
*/
                    int lon;

                    /**
Altitude (MSL)
*/
                    float alt;

                    /**
target velocity (0,0,0) for unknown
*/
                    [Dims(+3)] float vel;

                    /**
linear target acceleration (0,0,0) for unknown
*/
                    [Dims(+3)] float acc;

                    /**
(0 0 0 0 for unknown)
*/
                    [Dims(+4)] float attitude_q;

                    /**
(0 0 0 for unknown)
*/
                    [Dims(+3)] float rates;

                    /**
eph epv
*/
                    [Dims(+3)] float position_cov;

                    /**
button states or switches of a tracker device
*/
                    ulong custom_state;
                }

                /**
The smoothed, monotonic system state used to feed the control loops of the system.
*/
                class CONTROL_SYSTEM_STATE
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
X acceleration in body frame
*/
                    float x_acc;

                    /**
Y acceleration in body frame
*/
                    float y_acc;

                    /**
Z acceleration in body frame
*/
                    float z_acc;

                    /**
X velocity in body frame
*/
                    float x_vel;

                    /**
Y velocity in body frame
*/
                    float y_vel;

                    /**
Z velocity in body frame
*/
                    float z_vel;

                    /**
X position in local frame
*/
                    float x_pos;

                    /**
Y position in local frame
*/
                    float y_pos;

                    /**
Z position in local frame
*/
                    float z_pos;

                    /**
Airspeed, set to -1 if unknown
*/
                    float airspeed;

                    /**
Variance of body velocity estimate
*/
                    [Dims(+3)] float vel_variance;

                    /**
Variance in local position
*/
                    [Dims(+3)] float pos_variance;

                    /**
The attitude, represented as Quaternion
*/
                    [Dims(+4)] float q;

                    /**
Angular rate in roll axis
*/
                    float roll_rate;

                    /**
Angular rate in pitch axis
*/
                    float pitch_rate;

                    /**
Angular rate in yaw axis
*/
                    float yaw_rate;
                }

                /**
Battery information. Updates GCS with flight controller battery status. Smart batteries also use this
message, but may additionally send SMART_BATTERY_INFO.
*/
                class BATTERY_STATUS
                {
                    /**
Battery ID
*/
                    byte id;

                    /**
Function of the battery
*/
                    MAV_BATTERY_FUNCTION battery_function;

                    /**
Type (chemistry) of the battery
*/
                    MAV_BATTERY_TYPE Typ;

                    /**
Temperature of the battery. INT16_MAX for unknown temperature.
*/
                    short temperature;

                    /**
Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid
cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown
or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all
others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0
should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple
cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
*/
                    [Dims(+10)] ushort voltages;

                    /**
Battery current, -1: autopilot does not measure the current
*/
                    short current_battery;

                    /**
Consumed charge, -1: autopilot does not provide consumption estimate
*/
                    int current_consumed;

                    /**
Consumed energy, -1: autopilot does not provide energy consumption estimate
*/
                    int energy_consumed;

                    /**
Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
*/
                    sbyte battery_remaining;

                    /**
Remaining battery time, 0: autopilot does not provide remaining battery time estimate
*/
                    int time_remaining;

                    /**
State for extent of discharge, provided by autopilot for warning or external reactions
*/
                    MAV_BATTERY_CHARGE_STATE charge_state;

                    /**
Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a value
of 0, where zero indicates not supported (note, this is different than for the voltages field and allows
empty byte truncation). If the measured value is 0 then 1 should be sent instead.
*/
                    [Dims(+4)] ushort voltages_ext;

                    /**
Battery mode. Default (0) is that battery mode reporting is not supported or battery is in normal-use
mode.
*/
                    MAV_BATTERY_MODE mode;

                    /**
Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED or
MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).
*/
                    MAV_BATTERY_FAULT fault_bitmask;
                }

                /**
Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.
*/
                class AUTOPILOT_VERSION
                {
                    /**
Bitmap of capabilities
*/
                    MAV_PROTOCOL_CAPABILITY capabilities;

                    /**
Firmware version number
*/
                    uint flight_sw_version;

                    /**
Middleware version number
*/
                    uint middleware_sw_version;

                    /**
Operating system version number
*/
                    uint os_sw_version;

                    /**
HW / board version (last 8 bits should be silicon ID, if any). The first 16 bits of this field specify
https://github.com/PX4/PX4-Bootloader/blob/master/board_types.txt
*/
                    uint board_version;

                    /**
Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
should allow to identify the commit using the main version number even for very large code bases.
*/
                    [Dims(+8)] byte flight_custom_version;

                    /**
Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
should allow to identify the commit using the main version number even for very large code bases.
*/
                    [Dims(+8)] byte middleware_custom_version;

                    /**
Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
should allow to identify the commit using the main version number even for very large code bases.
*/
                    [Dims(+8)] byte os_custom_version;

                    /**
ID of the board vendor
*/
                    ushort vendor_id;

                    /**
ID of the product
*/
                    ushort product_id;

                    /**
UID if provided by hardware (see uid2)
*/
                    ulong uid;

                    /**
UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
use uid)
*/
                    [Dims(+18)] byte uid2;
                }

                /**
The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
*/
                class LANDING_TARGET
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
The ID of the target if multiple targets are present
*/
                    byte target_num;

                    /**
Coordinate frame used for following fields.
*/
                    MAV_FRAME frame;

                    /**
X-axis angular offset of the target from the center of the image
*/
                    float angle_x;

                    /**
Y-axis angular offset of the target from the center of the image
*/
                    float angle_y;

                    /**
Distance to the target from the vehicle
*/
                    float distance;

                    /**
Size of target along x-axis
*/
                    float size_x;

                    /**
Size of target along y-axis
*/
                    float size_y;

                    /**
X Position of the landing target in MAV_FRAME
*/
                    float x;

                    /**
Y Position of the landing target in MAV_FRAME
*/
                    float y;

                    /**
Z Position of the landing target in MAV_FRAME
*/
                    float z;

                    /**
Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
*/
                    [Dims(+4)] float q;

                    /**
Type of landing target
*/
                    LANDING_TARGET_TYPE Typ;

                    /**
Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information
(valid: 1, invalid: 0). Default is 0 (invalid).
*/
                    byte position_valid;
                }

                /**
Status of geo-fencing. Sent in extended status stream when fencing enabled.
*/
                class FENCE_STATUS
                {
                    /**
Breach status (0 if currently inside fence, 1 if outside).
*/
                    byte breach_status;

                    /**
Number of fence breaches.
*/
                    ushort breach_count;

                    /**
Last breach type.
*/
                    FENCE_BREACH breach_type;

                    /**
Time (since boot) of last breach.
*/
                    uint breach_time;

                    /**
Active action to prevent fence breach
*/
                    FENCE_MITIGATE breach_mitigation;
                }

                /**
Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
*/
                class MAG_CAL_REPORT
                {
                    /**
Compass being calibrated.
*/
                    byte compass_id;

                    /**
Bitmask of compasses being calibrated.
*/
                    byte cal_mask;

                    /**
Calibration Status.
*/
                    MAG_CAL_STATUS cal_status;

                    /**
0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
*/
                    byte autosaved;

                    /**
RMS milligauss residuals.
*/
                    float fitness;

                    /**
X offset.
*/
                    float ofs_x;

                    /**
Y offset.
*/
                    float ofs_y;

                    /**
Z offset.
*/
                    float ofs_z;

                    /**
X diagonal (matrix 11).
*/
                    float diag_x;

                    /**
Y diagonal (matrix 22).
*/
                    float diag_y;

                    /**
Z diagonal (matrix 33).
*/
                    float diag_z;

                    /**
X off-diagonal (matrix 12 and 21).
*/
                    float offdiag_x;

                    /**
Y off-diagonal (matrix 13 and 31).
*/
                    float offdiag_y;

                    /**
Z off-diagonal (matrix 32 and 23).
*/
                    float offdiag_z;

                    /**
Confidence in orientation (higher is better).
*/
                    float orientation_confidence;

                    /**
orientation before calibration.
*/
                    MAV_SENSOR_ORIENTATION old_orientation;

                    /**
orientation after calibration.
*/
                    MAV_SENSOR_ORIENTATION new_orientation;

                    /**
field radius correction factor
*/
                    float scale_factor;
                }

                /**
EFI status output
*/
                class EFI_STATUS
                {
                    /**
EFI health status
*/
                    byte health;

                    /**
ECU index
*/
                    float ecu_index;

                    /**
RPM
*/
                    float rpm;

                    /**
Fuel consumed
*/
                    float fuel_consumed;

                    /**
Fuel flow rate
*/
                    float fuel_flow;

                    /**
Engine load
*/
                    float engine_load;

                    /**
Throttle position
*/
                    float throttle_position;

                    /**
Spark dwell time
*/
                    float spark_dwell_time;

                    /**
Barometric pressure
*/
                    float barometric_pressure;

                    /**
Intake manifold pressure(
*/
                    float intake_manifold_pressure;

                    /**
Intake manifold temperature
*/
                    float intake_manifold_temperature;

                    /**
Cylinder head temperature
*/
                    float cylinder_head_temperature;

                    /**
Ignition timing (Crank angle degrees)
*/
                    float ignition_timing;

                    /**
Injection time
*/
                    float injection_time;

                    /**
Exhaust gas temperature
*/
                    float exhaust_gas_temperature;

                    /**
Output throttle
*/
                    float throttle_out;

                    /**
Pressure/temperature compensation
*/
                    float pt_compensation;
                }

                /**
Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
enum definition for further information. The innovation test ratios show the magnitude of the sensor
innovation divided by the innovation check threshold. Under normal operation the innovation test ratios
should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal
operation and indicate that a measurement has been rejected by the filter. The user should be notified
if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between
0.5 and 1.0 should be optional and controllable by the user.
*/
                class ESTIMATOR_STATUS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Bitmap indicating which EKF outputs are valid.
*/
                    ESTIMATOR_STATUS_FLAGS flags;

                    /**
Velocity innovation test ratio
*/
                    float vel_ratio;

                    /**
Horizontal position innovation test ratio
*/
                    float pos_horiz_ratio;

                    /**
Vertical position innovation test ratio
*/
                    float pos_vert_ratio;

                    /**
Magnetometer innovation test ratio
*/
                    float mag_ratio;

                    /**
Height above terrain innovation test ratio
*/
                    float hagl_ratio;

                    /**
True airspeed innovation test ratio
*/
                    float tas_ratio;

                    /**
Horizontal position 1-STD accuracy relative to the EKF local origin
*/
                    float pos_horiz_accuracy;

                    /**
Vertical position 1-STD accuracy relative to the EKF local origin
*/
                    float pos_vert_accuracy;
                }

                /**
Wind covariance estimate from vehicle.
*/
                class WIND_COV
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Wind in X (NED) direction
*/
                    float wind_x;

                    /**
Wind in Y (NED) direction
*/
                    float wind_y;

                    /**
Wind in Z (NED) direction
*/
                    float wind_z;

                    /**
Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
*/
                    float var_horiz;

                    /**
Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
*/
                    float var_vert;

                    /**
Altitude (MSL) that this measurement was taken at
*/
                    float wind_alt;

                    /**
Horizontal speed 1-STD accuracy
*/
                    float horiz_accuracy;

                    /**
Vertical speed 1-STD accuracy
*/
                    float vert_accuracy;
                }

                /**
GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
estimate of the system.
*/
                class GPS_INPUT
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
ID of the GPS for multiple GPS inputs
*/
                    byte gps_id;

                    /**
Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
*/
                    GPS_INPUT_IGNORE_FLAGS ignore_flags;

                    /**
GPS time (from start of GPS week)
*/
                    uint time_week_ms;

                    /**
GPS week number
*/
                    ushort time_week;

                    /**
0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
*/
                    byte fix_type;

                    /**
Latitude (WGS84)
*/
                    int lat;

                    /**
Longitude (WGS84)
*/
                    int lon;

                    /**
Altitude (MSL). Positive for up.
*/
                    float alt;

                    /**
GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
*/
                    float hdop;

                    /**
GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
*/
                    float vdop;

                    /**
GPS velocity in north direction in earth-fixed NED frame
*/
                    float vn;

                    /**
GPS velocity in east direction in earth-fixed NED frame
*/
                    float ve;

                    /**
GPS velocity in down direction in earth-fixed NED frame
*/
                    float vd;

                    /**
GPS speed accuracy
*/
                    float speed_accuracy;

                    /**
GPS horizontal accuracy
*/
                    float horiz_accuracy;

                    /**
GPS vertical accuracy
*/
                    float vert_accuracy;

                    /**
Number of satellites visible.
*/
                    byte satellites_visible;

                    /**
Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
*/
                    ushort yaw;
                }

                /**
RTCM message for injecting into the onboard GPS (used for DGPS)
*/
                class GPS_RTCM_DATA
                {
                    /**
LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
corrupt RTCM data, and to recover from a unreliable transport delivery order.
*/
                    byte flags;

                    /**
data length
*/
                    byte len;

                    /**
RTCM message (may be fragmented)
*/
                    [Dims(+180)] byte Dat;
                }

                /**
Message appropriate for high latency connections like Iridium
*/
                class HIGH_LATENCY
                {
                    /**
Bitmap of enabled system modes.
*/
                    MAV_MODE_FLAG base_mode;

                    /**
A bitfield for use for autopilot-specific flags.
*/
                    uint custom_mode;

                    /**
The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
*/
                    MAV_LANDED_STATE landed_state;

                    /**
roll
*/
                    short roll;

                    /**
pitch
*/
                    short pitch;

                    /**
heading
*/
                    ushort heading;

                    /**
throttle (percentage)
*/
                    sbyte throttle;

                    /**
heading setpoint
*/
                    short heading_sp;

                    /**
Latitude
*/
                    int latitude;

                    /**
Longitude
*/
                    int longitude;

                    /**
Altitude above mean sea level
*/
                    short altitude_amsl;

                    /**
Altitude setpoint relative to the home position
*/
                    short altitude_sp;

                    /**
airspeed
*/
                    byte airspeed;

                    /**
airspeed setpoint
*/
                    byte airspeed_sp;

                    /**
groundspeed
*/
                    byte groundspeed;

                    /**
climb rate
*/
                    sbyte climb_rate;

                    /**
Number of satellites visible. If unknown, set to UINT8_MAX
*/
                    byte gps_nsat;

                    /**
GPS Fix type.
*/
                    GPS_FIX_TYPE gps_fix_type;

                    /**
Remaining battery (percentage)
*/
                    byte battery_remaining;

                    /**
Autopilot temperature (degrees C)
*/
                    sbyte temperature;

                    /**
Air temperature (degrees C) from airspeed sensor
*/
                    sbyte temperature_air;

                    /**
failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
bit3:GCS, bit4:fence)
*/
                    byte failsafe;

                    /**
current waypoint number
*/
                    byte wp_num;

                    /**
distance to target
*/
                    ushort wp_distance;
                }

                /**
Message appropriate for high latency connections like Iridium (version 2)
*/
                class HIGH_LATENCY2
                {
                    /**
Timestamp (milliseconds since boot or Unix epoch)
*/
                    uint timestamp;

                    /**
Type of the MAV (quadrotor, helicopter, etc.)
*/
                    MAV_TYPE Typ;

                    /**
Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
*/
                    MAV_AUTOPILOT autopilot;

                    /**
A bitfield for use for autopilot-specific flags (2 byte version).
*/
                    ushort custom_mode;

                    /**
Latitude
*/
                    int latitude;

                    /**
Longitude
*/
                    int longitude;

                    /**
Altitude above mean sea level
*/
                    short altitude;

                    /**
Altitude setpoint
*/
                    short target_altitude;

                    /**
Heading
*/
                    byte heading;

                    /**
Heading setpoint
*/
                    byte target_heading;

                    /**
Distance to target waypoint or position
*/
                    ushort target_distance;

                    /**
Throttle
*/
                    byte throttle;

                    /**
Airspeed
*/
                    byte airspeed;

                    /**
Airspeed setpoint
*/
                    byte airspeed_sp;

                    /**
Groundspeed
*/
                    byte groundspeed;

                    /**
Windspeed
*/
                    byte windspeed;

                    /**
Wind heading
*/
                    byte wind_heading;

                    /**
Maximum error horizontal position since last message
*/
                    byte eph;

                    /**
Maximum error vertical position since last message
*/
                    byte epv;

                    /**
Air temperature from airspeed sensor
*/
                    sbyte temperature_air;

                    /**
Maximum climb rate magnitude since last message
*/
                    sbyte climb_rate;

                    /**
Battery level (-1 if field not provided).
*/
                    sbyte battery;

                    /**
Current waypoint number
*/
                    ushort wp_num;

                    /**
Bitmap of failure flags.
*/
                    HL_FAILURE_FLAG failure_flags;

                    /**
Field for custom payload.
*/
                    sbyte custom0;

                    /**
Field for custom payload.
*/
                    sbyte custom1;

                    /**
Field for custom payload.
*/
                    sbyte custom2;
                }

                /**
Vibration levels and accelerometer clipping
*/
                class VIBRATION
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Vibration levels on X-axis
*/
                    float vibration_x;

                    /**
Vibration levels on Y-axis
*/
                    float vibration_y;

                    /**
Vibration levels on Z-axis
*/
                    float vibration_z;

                    /**
first accelerometer clipping count
*/
                    uint clipping_0;

                    /**
second accelerometer clipping count
*/
                    uint clipping_1;

                    /**
third accelerometer clipping count
*/
                    uint clipping_2;
                }

                /**
This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
will return to and land on. The position is set automatically by the system during the takeoff in case
it was not explicitly set by the operator before or after. The global and local positions encode the
position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.
Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft
to adjust the approach. The approach 3D vector describes the point to which the system should fly in
normal flight mode and then perform a landing sequence along the vector.
*/
                class HOME_POSITION
                {
                    /**
Latitude (WGS84)
*/
                    int latitude;

                    /**
Longitude (WGS84)
*/
                    int longitude;

                    /**
Altitude (MSL). Positive for up.
*/
                    int altitude;

                    /**
Local X position of this position in the local coordinate frame
*/
                    float x;

                    /**
Local Y position of this position in the local coordinate frame
*/
                    float y;

                    /**
Local Z position of this position in the local coordinate frame
*/
                    float z;

                    /**
World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
and slope of the ground
*/
                    [Dims(+4)] float q;

                    /**
Local X position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone.
*/
                    float approach_x;

                    /**
Local Y position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone.
*/
                    float approach_y;

                    /**
Local Z position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone.
*/
                    float approach_z;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;
                }

                /**
The position the system will return to and land on. The position is set automatically by the system during
the takeoff in case it was not explicitly set by the operator before or after. The global and local positions
encode the position in the respective coordinate frames, while the q parameter encodes the orientation
of the surface. Under normal conditions it describes the heading and terrain slope, which can be used
by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system
should fly in normal flight mode and then perform a landing sequence along the vector.
*/
                class SET_HOME_POSITION
                {
                    /**
System ID.
*/
                    byte target_system;

                    /**
Latitude (WGS84)
*/
                    int latitude;

                    /**
Longitude (WGS84)
*/
                    int longitude;

                    /**
Altitude (MSL). Positive for up.
*/
                    int altitude;

                    /**
Local X position of this position in the local coordinate frame
*/
                    float x;

                    /**
Local Y position of this position in the local coordinate frame
*/
                    float y;

                    /**
Local Z position of this position in the local coordinate frame
*/
                    float z;

                    /**
World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
and slope of the ground
*/
                    [Dims(+4)] float q;

                    /**
Local X position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone.
*/
                    float approach_x;

                    /**
Local Y position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone.
*/
                    float approach_y;

                    /**
Local Z position of the end of the approach vector. Multicopters should set this position based on their
takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
from the threshold / touchdown zone.
*/
                    float approach_z;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;
                }

                /**
The interval between messages for a particular MAVLink message ID. This message is the response to the
MAV_CMD_GET_MESSAGE_INTERVAL command. This interface replaces DATA_STREAM.
*/
                class MESSAGE_INTERVAL
                {
                    /**
The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
*/
                    ushort message_id;

                    /**
0 indicates the interval at which it is sent.
*/
                    int interval_us;
                }

                /**
Provides state for additional features
*/
                class EXTENDED_SYS_STATE
                {
                    /**
The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
*/
                    MAV_VTOL_STATE vtol_state;

                    /**
The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
*/
                    MAV_LANDED_STATE landed_state;
                }

                /**
The location and information of an ADSB vehicle
*/
                class ADSB_VEHICLE
                {
                    /**
ICAO address
*/
                    uint ICAO_address;

                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;

                    /**
ADSB altitude type.
*/
                    ADSB_ALTITUDE_TYPE altitude_type;

                    /**
Altitude(ASL)
*/
                    int altitude;

                    /**
Course over ground
*/
                    ushort heading;

                    /**
The horizontal velocity
*/
                    ushort hor_velocity;

                    /**
The vertical velocity. Positive is up
*/
                    short ver_velocity;

                    /**
The callsign, 8+null
*/
                    string callsign;

                    /**
ADSB emitter type.
*/
                    ADSB_EMITTER_TYPE emitter_type;

                    /**
Time since last communication in seconds
*/
                    byte tslc;

                    /**
Bitmap to indicate various statuses including valid data fields
*/
                    ADSB_FLAGS flags;

                    /**
Squawk code
*/
                    ushort squawk;
                }

                /**
Information about a potential collision
*/
                class COLLISION
                {
                    /**
Collision data source
*/
                    MAV_COLLISION_SRC Sr;

                    /**
Unique identifier, domain based on src field
*/
                    uint id;

                    /**
Action that is being taken to avoid this collision
*/
                    MAV_COLLISION_ACTION action;

                    /**
How concerned the aircraft is about this collision
*/
                    MAV_COLLISION_THREAT_LEVEL threat_level;

                    /**
Estimated time until collision occurs
*/
                    float time_to_minimum_delta;

                    /**
Closest vertical distance between vehicle and object
*/
                    float altitude_minimum_delta;

                    /**
Closest horizontal distance between vehicle and object
*/
                    float horizontal_minimum_delta;
                }

                /**
Message implementing parts of the V2 payload specs in V1 frames for transitional support.
*/
                class V2_EXTENSION
                {
                    /**
Network ID (0 for broadcast)
*/
                    byte target_network;

                    /**
System ID (0 for broadcast)
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast)
*/
                    byte target_component;

                    /**
A code that identifies the software component that understands this message (analogous to USB device classes
or mime type strings). If this code is less than 32768, it is considered a 'registered' protocol extension
and the corresponding entry should be added to https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml.
Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
Message_types greater than 32767 are considered local experiments and should not be checked in to any
widely distributed codebase.
*/
                    ushort message_type;

                    /**
Variable length payload. The length must be encoded in the payload as part of the message_type protocol,
e.g. by including the length as payload data, or by terminating the payload data with a non-zero marker.
This is required in order to reconstruct zero-terminated payloads that are (or otherwise would be) trimmed
by MAVLink 2 empty-byte truncation. The entire content of the payload block is opaque unless you understand
the encoding message_type. The particular encoding used can be extension specific and might not always
be documented as part of the MAVLink specification.
*/
                    [Dims(+249)] byte payload;
                }

                /**
Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
way for testing new messages and getting experimental debug output.
*/
                class MEMORY_VECT
                {
                    /**
Starting address of the debug variables
*/
                    ushort address;

                    /**
Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
*/
                    byte ver;

                    /**
Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
*/
                    byte Typ;

                    /**
Memory contents at specified address
*/
                    [Dims(+32)] sbyte Valu;
                }

                /**
To debug something using a named 3D vector.
*/
                class DEBUG_VECT
                {
                    /**
Name
*/
                    string name;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
x
*/
                    float x;

                    /**
y
*/
                    float y;

                    /**
z
*/
                    float z;
                }

                /**
Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
efficient way for testing new messages and getting experimental debug output.
*/
                class NAMED_VALUE_FLOAT
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Name of the debug variable
*/
                    string name;

                    /**
Floating point value
*/
                    float Valu;
                }

                /**
Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
efficient way for testing new messages and getting experimental debug output.
*/
                class NAMED_VALUE_INT
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Name of the debug variable
*/
                    string name;

                    /**
Signed integer value
*/
                    int Valu;
                }

                /**
Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
They consume quite some bandwidth, so use only for important status and error messages. If implemented
wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
*/
                class STATUSTEXT
                {
                    /**
Severity of status. Relies on the definitions within RFC-5424.
*/
                    MAV_SEVERITY severity;

                    /**
Status text message, without null termination character
*/
                    string text;

                    /**
Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext
message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence
and the message can be emitted immediately.
*/
                    ushort id;

                    /**
This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to
mean this was the last chunk.
*/
                    byte chunk_seq;
                }

                /**
Send a debug value. The index is used to discriminate between values. These values show up in the plot
of QGroundControl as DEBUG N.
*/
                class DEBUG
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
index of debug variable
*/
                    byte ind;

                    /**
DEBUG value
*/
                    float Valu;
                }

                /**
Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
signing
*/
                class SETUP_SIGNING
                {
                    /**
system id of the target
*/
                    byte target_system;

                    /**
component ID of the target
*/
                    byte target_component;

                    /**
signing key
*/
                    [Dims(+32)] byte secret_key;

                    /**
initial timestamp
*/
                    ulong initial_timestamp;
                }

                /**
Report button state change.
*/
                class BUTTON_CHANGE
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Time of last change of button state.
*/
                    uint last_change_ms;

                    /**
Bitmap for state of buttons.
*/
                    byte state;
                }

                /**
Control vehicle tone generation (buzzer).
*/
                class PLAY_TUNE
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
tune in board specific format
*/
                    string tune;

                    /**
tune extension (appended to tune)
*/
                    string tune2;
                }

                /**
Information about a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
*/
                class CAMERA_INFORMATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Name of the camera vendor
*/
                    [Dims(+32)] byte vendor_name;

                    /**
Name of the camera model
*/
                    [Dims(+32)] byte model_name;

                    /**
0xff)
*/
                    uint firmware_version;

                    /**
Focal length
*/
                    float focal_length;

                    /**
Image sensor size horizontal
*/
                    float sensor_size_h;

                    /**
Image sensor size vertical
*/
                    float sensor_size_v;

                    /**
Horizontal image resolution
*/
                    ushort resolution_h;

                    /**
Vertical image resolution
*/
                    ushort resolution_v;

                    /**
Reserved for a lens ID
*/
                    byte lens_id;

                    /**
Bitmap of camera capability flags.
*/
                    CAMERA_CAP_FLAGS flags;

                    /**
Camera definition version (iteration)
*/
                    ushort cam_definition_version;

                    /**
Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and
MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements
the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension
.xml.xz (a GCS that implements the protocol must support decompressing the file). The string needs to
be zero terminated.
*/
                    string cam_definition_uri;
                }

                /**
Settings of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
*/
                class CAMERA_SETTINGS
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Camera mode
*/
                    CAMERA_MODE mode_id;

                    /**
Current zoom level (0.0 to 100.0, NaN if not known)
*/
                    float zoomLevel;

                    /**
Current focus level (0.0 to 100.0, NaN if not known)
*/
                    float focusLevel;
                }

                /**
Information about a storage medium. This message is sent in response to a request with MAV_CMD_REQUEST_MESSAGE
and whenever the status of the storage changes (STORAGE_STATUS). Use MAV_CMD_REQUEST_MESSAGE.param2 to
indicate the index/id of requested storage: 0 for all, 1 for first, 2 for second, etc.
*/
                class STORAGE_INFORMATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Storage ID (1 for first, 2 for second, etc.)
*/
                    byte storage_id;

                    /**
Number of storage devices
*/
                    byte storage_count;

                    /**
Status of storage
*/
                    STORAGE_STATUS status;

                    /**
Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
*/
                    float total_capacity;

                    /**
Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
*/
                    float used_capacity;

                    /**
Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
*/
                    float available_capacity;

                    /**
Read speed.
*/
                    float read_speed;

                    /**
Write speed.
*/
                    float write_speed;

                    /**
Type of storage
*/
                    STORAGE_TYPE Typ;

                    /**
Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string.
If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type
is shown to the user.
*/
                    string name;

                    /**
This setting can then be overridden using MAV_CMD_SET_STORAGE_USAGE.
        If the media usage flags
are not set, a GCS may assume storage ID 1 is the default storage for all media types.
*/
                    STORAGE_USAGE_FLAG storage_usage;
                }

                /**
Information about the status of a capture. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
*/
                class CAMERA_CAPTURE_STATUS
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
set and capture in progress)
*/
                    byte image_status;

                    /**
Current status of video capturing (0: idle, 1: capture in progress)
*/
                    byte video_status;

                    /**
Image capture interval
*/
                    float image_interval;

                    /**
Elapsed time since recording started (0: Not supported/available). A GCS should compute recording time
and use non-zero values of this field to correct any discrepancy.
*/
                    uint recording_time_ms;

                    /**
Available storage capacity.
*/
                    float available_capacity;

                    /**
Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
*/
                    int image_count;
                }

                /**
set to -1 to send the message for the sequence number in param 2 and all the following sequence numbers,

        set to the sequence number of the final message in the range.
*/
                class CAMERA_IMAGE_CAPTURED
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
*/
                    ulong time_utc;

                    /**
Deprecated/unused. Component IDs are used to differentiate multiple cameras.
*/
                    byte camera_id;

                    /**
Latitude where image was taken
*/
                    int lat;

                    /**
Longitude where capture was taken
*/
                    int lon;

                    /**
Altitude (MSL) where image was taken
*/
                    int alt;

                    /**
Altitude above ground
*/
                    int relative_alt;

                    /**
Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
*/
                    [Dims(+4)] float q;

                    /**
Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)
*/
                    int image_index;

                    /**
Boolean indicating success (1) or failure (0) while capturing this image.
*/
                    sbyte capture_result;

                    /**
URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
*/
                    string file_url;
                }

                /**
Information about flight since last arming.
        This can be requested using MAV_CMD_REQUEST_MESSAGE.
*/
                class FLIGHT_INFORMATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Timestamp at arming (time since UNIX epoch) in UTC, 0 for unknown
*/
                    ulong arming_time_utc;

                    /**
Timestamp at takeoff (time since UNIX epoch) in UTC, 0 for unknown
*/
                    ulong takeoff_time_utc;

                    /**
Universally unique identifier (UUID) of flight, should correspond to name of log files
*/
                    ulong flight_uuid;
                }

                /**
Orientation of a mount
*/
                class MOUNT_ORIENTATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Roll in global frame (set to NaN for invalid).
*/
                    float roll;

                    /**
Pitch in global frame (set to NaN for invalid).
*/
                    float pitch;

                    /**
Yaw relative to vehicle (set to NaN for invalid).
*/
                    float yaw;

                    /**
Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
*/
                    float yaw_absolute;
                }

                /**
A message containing logged data (see also MAV_CMD_LOGGING_START)
*/
                class LOGGING_DATA
                {
                    /**
system ID of the target
*/
                    byte target_system;

                    /**
component ID of the target
*/
                    byte target_component;

                    /**
sequence number (can wrap)
*/
                    ushort sequence;

                    /**
data length
*/
                    byte length;

                    /**
offset into data where first message starts. This can be used for recovery, when a previous message got
lost (set to UINT8_MAX if no start exists).
*/
                    byte first_message_offset;

                    /**
logged data
*/
                    [Dims(+249)] byte Dat;
                }

                /**
A message containing logged data which requires a LOGGING_ACK to be sent back
*/
                class LOGGING_DATA_ACKED
                {
                    /**
system ID of the target
*/
                    byte target_system;

                    /**
component ID of the target
*/
                    byte target_component;

                    /**
sequence number (can wrap)
*/
                    ushort sequence;

                    /**
data length
*/
                    byte length;

                    /**
offset into data where first message starts. This can be used for recovery, when a previous message got
lost (set to UINT8_MAX if no start exists).
*/
                    byte first_message_offset;

                    /**
logged data
*/
                    [Dims(+249)] byte Dat;
                }

                /**
An ack for a LOGGING_DATA_ACKED message
*/
                class LOGGING_ACK
                {
                    /**
system ID of the target
*/
                    byte target_system;

                    /**
component ID of the target
*/
                    byte target_component;

                    /**
sequence number (must match the one in LOGGING_DATA_ACKED)
*/
                    ushort sequence;
                }

                /**
Information about video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE, where param2 indicates
the video stream id: 0 for all streams, 1 for first, 2 for second, etc.
*/
                class VIDEO_STREAM_INFORMATION
                {
                    /**
Video Stream ID (1 for first, 2 for second, etc.)
*/
                    byte stream_id;

                    /**
Number of streams available.
*/
                    byte count;

                    /**
Type of stream.
*/
                    VIDEO_STREAM_TYPE Typ;

                    /**
Bitmap of stream status flags.
*/
                    VIDEO_STREAM_STATUS_FLAGS flags;

                    /**
Frame rate.
*/
                    float framerate;

                    /**
Horizontal resolution.
*/
                    ushort resolution_h;

                    /**
Vertical resolution.
*/
                    ushort resolution_v;

                    /**
Bit rate.
*/
                    uint bitrate;

                    /**
Video image rotation clockwise.
*/
                    ushort rotation;

                    /**
Horizontal Field of view.
*/
                    ushort hfov;

                    /**
Stream name.
*/
                    string name;

                    /**
Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station
should listen to).
*/
                    string uri;
                }

                /**
Information about the status of a video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE.
*/
                class VIDEO_STREAM_STATUS
                {
                    /**
Video Stream ID (1 for first, 2 for second, etc.)
*/
                    byte stream_id;

                    /**
Bitmap of stream status flags
*/
                    VIDEO_STREAM_STATUS_FLAGS flags;

                    /**
Frame rate
*/
                    float framerate;

                    /**
Horizontal resolution
*/
                    ushort resolution_h;

                    /**
Vertical resolution
*/
                    ushort resolution_v;

                    /**
Bit rate
*/
                    uint bitrate;

                    /**
Video image rotation clockwise
*/
                    ushort rotation;

                    /**
Horizontal Field of view
*/
                    ushort hfov;
                }

                /**
Information about the field of view of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
*/
                class CAMERA_FOV_STATUS
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Latitude of camera (INT32_MAX if unknown).
*/
                    int lat_camera;

                    /**
Longitude of camera (INT32_MAX if unknown).
*/
                    int lon_camera;

                    /**
Altitude (MSL) of camera (INT32_MAX if unknown).
*/
                    int alt_camera;

                    /**
Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
*/
                    int lat_image;

                    /**
Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
*/
                    int lon_image;

                    /**
Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with
horizon).
*/
                    int alt_image;

                    /**
Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
*/
                    [Dims(+4)] float q;

                    /**
Horizontal field of view (NaN if unknown).
*/
                    float hfov;

                    /**
Vertical field of view (NaN if unknown).
*/
                    float vfov;
                }

                /**
Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message
interval.
*/
                class CAMERA_TRACKING_IMAGE_STATUS
                {
                    /**
Current tracking status
*/
                    CAMERA_TRACKING_STATUS_FLAGS tracking_status;

                    /**
Current tracking mode
*/
                    CAMERA_TRACKING_MODE tracking_mode;

                    /**
Defines location of target data
*/
                    CAMERA_TRACKING_TARGET_DATA target_data;

                    /**
Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right),
NAN if unknown
*/
                    float point_x;

                    /**
Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom),
NAN if unknown
*/
                    float point_y;

                    /**
Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right),
NAN if unknown
*/
                    float radius;

                    /**
Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1
is right), NAN if unknown
*/
                    float rec_top_x;

                    /**
Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1
is bottom), NAN if unknown
*/
                    float rec_top_y;

                    /**
Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left,
1 is right), NAN if unknown
*/
                    float rec_bottom_x;

                    /**
Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top,
1 is bottom), NAN if unknown
*/
                    float rec_bottom_y;
                }

                /**
Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message
interval.
*/
                class CAMERA_TRACKING_GEO_STATUS
                {
                    /**
Current tracking status
*/
                    CAMERA_TRACKING_STATUS_FLAGS tracking_status;

                    /**
Latitude of tracked object
*/
                    int lat;

                    /**
Longitude of tracked object
*/
                    int lon;

                    /**
Altitude of tracked object(AMSL, WGS84)
*/
                    float alt;

                    /**
Horizontal accuracy. NAN if unknown
*/
                    float h_acc;

                    /**
Vertical accuracy. NAN if unknown
*/
                    float v_acc;

                    /**
North velocity of tracked object. NAN if unknown
*/
                    float vel_n;

                    /**
East velocity of tracked object. NAN if unknown
*/
                    float vel_e;

                    /**
Down velocity of tracked object. NAN if unknown
*/
                    float vel_d;

                    /**
Velocity accuracy. NAN if unknown
*/
                    float vel_acc;

                    /**
Distance between camera and tracked object. NAN if unknown
*/
                    float dist;

                    /**
Heading in radians, in NED. NAN if unknown
*/
                    float hdg;

                    /**
Accuracy of heading, in NED. NAN if unknown
*/
                    float hdg_acc;
                }

                /**
Information about a high level gimbal manager. This message should be requested by a ground station using
MAV_CMD_REQUEST_MESSAGE.
*/
                class GIMBAL_MANAGER_INFORMATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Bitmap of gimbal capability flags.
*/
                    GIMBAL_MANAGER_CAP_FLAGS cap_flags;

                    /**
Gimbal device ID that this gimbal manager is responsible for.
*/
                    byte gimbal_device_id;

                    /**
Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
*/
                    float roll_min;

                    /**
Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
*/
                    float roll_max;

                    /**
Minimum pitch angle (positive: up, negative: down)
*/
                    float pitch_min;

                    /**
Maximum pitch angle (positive: up, negative: down)
*/
                    float pitch_max;

                    /**
Minimum yaw angle (positive: to the right, negative: to the left)
*/
                    float yaw_min;

                    /**
Maximum yaw angle (positive: to the right, negative: to the left)
*/
                    float yaw_max;
                }

                /**
Current status about a high level gimbal manager. This message should be broadcast at a low regular rate
(e.g. 5Hz).
*/
                class GIMBAL_MANAGER_STATUS
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
High level gimbal manager flags currently applied.
*/
                    GIMBAL_MANAGER_FLAGS flags;

                    /**
Gimbal device ID that this gimbal manager is responsible for.
*/
                    byte gimbal_device_id;

                    /**
System ID of MAVLink component with primary control, 0 for none.
*/
                    byte primary_control_sysid;

                    /**
Component ID of MAVLink component with primary control, 0 for none.
*/
                    byte primary_control_compid;

                    /**
System ID of MAVLink component with secondary control, 0 for none.
*/
                    byte secondary_control_sysid;

                    /**
Component ID of MAVLink component with secondary control, 0 for none.
*/
                    byte secondary_control_compid;
                }

                /**
High level message to control a gimbal's attitude. This message is to be sent to the gimbal manager (e.g.
from a ground station). Angles and rates can be set to NaN according to use case.
*/
                class GIMBAL_MANAGER_SET_ATTITUDE
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
High level gimbal manager flags to use.
*/
                    GIMBAL_MANAGER_FLAGS flags;

                    /**
Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).
*/
                    byte gimbal_device_id;

                    /**
Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag
GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
*/
                    [Dims(+4)] float q;

                    /**
X component of angular velocity, positive is rolling to the right, NaN to be ignored.
*/
                    float angular_velocity_x;

                    /**
Y component of angular velocity, positive is pitching up, NaN to be ignored.
*/
                    float angular_velocity_y;

                    /**
Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
*/
                    float angular_velocity_z;
                }

                /**
Information about a low level gimbal. This message should be requested by the gimbal manager or a ground
station using MAV_CMD_REQUEST_MESSAGE. The maximum angles and rates are the limits by hardware. However,
the limits by software used are likely different/smaller and dependent on mode/settings/etc..
*/
                class GIMBAL_DEVICE_INFORMATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Name of the gimbal vendor.
*/
                    string vendor_name;

                    /**
Name of the gimbal model.
*/
                    string model_name;

                    /**
Custom name of the gimbal given to it by the user.
*/
                    string custom_name;

                    /**
0xff).
*/
                    uint firmware_version;

                    /**
0xff).
*/
                    uint hardware_version;

                    /**
UID of gimbal hardware (0 if unknown).
*/
                    ulong uid;

                    /**
Bitmap of gimbal capability flags.
*/
                    GIMBAL_DEVICE_CAP_FLAGS cap_flags;

                    /**
Bitmap for use for gimbal-specific capability flags.
*/
                    ushort custom_cap_flags;

                    /**
Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
*/
                    float roll_min;

                    /**
Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
*/
                    float roll_max;

                    /**
Minimum hardware pitch angle (positive: up, negative: down)
*/
                    float pitch_min;

                    /**
Maximum hardware pitch angle (positive: up, negative: down)
*/
                    float pitch_max;

                    /**
Minimum hardware yaw angle (positive: to the right, negative: to the left)
*/
                    float yaw_min;

                    /**
Maximum hardware yaw angle (positive: to the right, negative: to the left)
*/
                    float yaw_max;
                }

                /**
Low level message to control a gimbal device's attitude. This message is to be sent from the gimbal manager
to the gimbal device component. Angles and rates can be set to NaN according to use case.
*/
                class GIMBAL_DEVICE_SET_ATTITUDE
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Low level gimbal flags.
*/
                    GIMBAL_DEVICE_FLAGS flags;

                    /**
Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag
GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, set all fields to NaN if only angular velocity should be used)
*/
                    [Dims(+4)] float q;

                    /**
X component of angular velocity, positive is rolling to the right, NaN to be ignored.
*/
                    float angular_velocity_x;

                    /**
Y component of angular velocity, positive is pitching up, NaN to be ignored.
*/
                    float angular_velocity_y;

                    /**
Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
*/
                    float angular_velocity_z;
                }

                /**
Message reporting the status of a gimbal device. This message should be broadcasted by a gimbal device
component. The angles encoded in the quaternion are relative to absolute North if the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK
is set (roll: positive is rolling to the right, pitch: positive is pitching up, yaw is turn to the right)
or relative to the vehicle heading if the flag is not set. This message should be broadcast at a low
regular rate (e.g. 10Hz).
*/
                class GIMBAL_DEVICE_ATTITUDE_STATUS
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
Current gimbal flags set.
*/
                    GIMBAL_DEVICE_FLAGS flags;

                    /**
Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag
GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
*/
                    [Dims(+4)] float q;

                    /**
X component of angular velocity (NaN if unknown)
*/
                    float angular_velocity_x;

                    /**
Y component of angular velocity (NaN if unknown)
*/
                    float angular_velocity_y;

                    /**
Z component of angular velocity (NaN if unknown)
*/
                    float angular_velocity_z;

                    /**
Failure flags (0 for no failure)
*/
                    GIMBAL_DEVICE_ERROR_FLAGS failure_flags;
                }

                /**
Low level message containing autopilot state relevant for a gimbal device. This message is to be sent
from the gimbal manager to the gimbal device component. The data of this message server for the gimbal's
estimator corrections in particular horizon compensation, as well as the autopilot's control intention
e.g. feed forward angular control in z-axis.
*/
                class AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Timestamp (time since system boot).
*/
                    ulong time_boot_us;

                    /**
Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
*/
                    [Dims(+4)] float q;

                    /**
Estimated delay of the attitude data.
*/
                    uint q_estimated_delay_us;

                    /**
X Speed in NED (North, East, Down).
*/
                    float vx;

                    /**
Y Speed in NED (North, East, Down).
*/
                    float vy;

                    /**
Z Speed in NED (North, East, Down).
*/
                    float vz;

                    /**
Estimated delay of the speed data.
*/
                    uint v_estimated_delay_us;

                    /**
Feed forward Z component of angular velocity, positive is yawing to the right, NaN to be ignored. This
is to indicate if the autopilot is actively yawing.
*/
                    float feed_forward_angular_velocity_z;

                    /**
Bitmap indicating which estimator outputs are valid.
*/
                    ESTIMATOR_STATUS_FLAGS estimator_status;

                    /**
The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
*/
                    MAV_LANDED_STATE landed_state;
                }

                /**
High level message to control a gimbal's pitch and yaw angles. This message is to be sent to the gimbal
manager (e.g. from a ground station). Angles and rates can be set to NaN according to use case.
*/
                class GIMBAL_MANAGER_SET_PITCHYAW
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
High level gimbal manager flags to use.
*/
                    GIMBAL_MANAGER_FLAGS flags;

                    /**
Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).
*/
                    byte gimbal_device_id;

                    /**
Pitch angle (positive: up, negative: down, NaN to be ignored).
*/
                    float pitch;

                    /**
Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
*/
                    float yaw;

                    /**
Pitch angular rate (positive: up, negative: down, NaN to be ignored).
*/
                    float pitch_rate;

                    /**
Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
*/
                    float yaw_rate;
                }

                /**
High level message to control a gimbal manually. The angles or angular rates are unitless; the actual
rates will depend on internal gimbal manager settings/configuration (e.g. set by parameters). This message
is to be sent to the gimbal manager (e.g. from a ground station). Angles and rates can be set to NaN
according to use case.
*/
                class GIMBAL_MANAGER_SET_MANUAL_CONTROL
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
High level gimbal manager flags.
*/
                    GIMBAL_MANAGER_FLAGS flags;

                    /**
Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).
*/
                    byte gimbal_device_id;

                    /**
Pitch angle unitless (-1..1, positive: up, negative: down, NaN to be ignored).
*/
                    float pitch;

                    /**
Yaw angle unitless (-1..1, positive: to the right, negative: to the left, NaN to be ignored).
*/
                    float yaw;

                    /**
Pitch angular rate unitless (-1..1, positive: up, negative: down, NaN to be ignored).
*/
                    float pitch_rate;

                    /**
Yaw angular rate unitless (-1..1, positive: to the right, negative: to the left, NaN to be ignored).
*/
                    float yaw_rate;
                }

                /**
ESC information for lower rate streaming. Recommended streaming rate 1Hz. See ESC_STATUS for higher-rate
ESC data.
*/
                class ESC_INFO
                {
                    /**
Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
*/
                    byte index;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number.
*/
                    ulong time_usec;

                    /**
Counter of data packets received.
*/
                    ushort counter;

                    /**
Total number of ESCs in all messages of this type. Message fields with an index higher than this should
be ignored because they contain invalid data.
*/
                    byte count;

                    /**
Connection type protocol for all ESC.
*/
                    ESC_CONNECTION_TYPE connection_type;

                    /**
Information regarding online/offline status of each ESC.
*/
                    byte info;

                    /**
Bitmap of ESC failure flags.
*/
                    ESC_FAILURE_FLAGS failure_flags;

                    /**
Number of reported errors by each ESC since boot.
*/
                    [Dims(+4)] uint error_count;

                    /**
Temperature of each ESC. INT16_MAX: if data not supplied by ESC.
*/
                    [Dims(+4)] short temperature;
                }

                /**
ESC information for higher rate streaming. Recommended streaming rate is ~10 Hz. Information that changes
more slowly is sent in ESC_INFO. It should typically only be streamed on high-bandwidth links (i.e. to
a companion computer).
*/
                class ESC_STATUS
                {
                    /**
Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
*/
                    byte index;

                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude the number.
*/
                    ulong time_usec;

                    /**
Reported motor RPM from each ESC (negative for reverse rotation).
*/
                    [Dims(+4)] int rpm;

                    /**
Voltage measured from each ESC.
*/
                    [Dims(+4)] float voltage;

                    /**
Current measured from each ESC.
*/
                    [Dims(+4)] float current;
                }

                /**
Configure WiFi AP SSID, password, and mode. This message is re-emitted as an acknowledgement by the AP.
The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE
*/
                class WIFI_CONFIG_AP
                {
                    /**
Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as
a response.
*/
                    string ssid;

                    /**
Password. Blank for an open AP. MD5 hash when message is sent back as a response.
*/
                    string password;

                    /**
WiFi Mode.
*/
                    WIFI_CONFIG_AP_MODE mode;

                    /**
Message acceptance response (sent back to GS).
*/
                    WIFI_CONFIG_AP_RESPONSE response;
                }

                /**
The location and information of an AIS vessel
*/
                class AIS_VESSEL
                {
                    /**
Mobile Marine Service Identifier, 9 decimal digits
*/
                    uint MMSI;

                    /**
Latitude
*/
                    int lat;

                    /**
Longitude
*/
                    int lon;

                    /**
Course over ground
*/
                    ushort COG;

                    /**
True heading
*/
                    ushort heading;

                    /**
Speed over ground
*/
                    ushort velocity;

                    /**
Turn rate
*/
                    sbyte turn_rate;

                    /**
Navigational status
*/
                    AIS_NAV_STATUS navigational_status;

                    /**
Type of vessels
*/
                    AIS_TYPE Typ;

                    /**
Distance from lat/lon location to bow
*/
                    ushort dimension_bow;

                    /**
Distance from lat/lon location to stern
*/
                    ushort dimension_stern;

                    /**
Distance from lat/lon location to port side
*/
                    byte dimension_port;

                    /**
Distance from lat/lon location to starboard side
*/
                    byte dimension_starboard;

                    /**
The vessel callsign
*/
                    string callsign;

                    /**
The vessel name
*/
                    string name;

                    /**
Time since last communication in seconds
*/
                    ushort tslc;

                    /**
Bitmask to indicate various statuses including valid data fields
*/
                    AIS_FLAGS flags;
                }

                /**
General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
for the background information. The UAVCAN specification is available at http://uavcan.org.
*/
                class UAVCAN_NODE_STATUS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Time since the start-up of the node.
*/
                    uint uptime_sec;

                    /**
Generalized node health status.
*/
                    UAVCAN_NODE_HEALTH health;

                    /**
Generalized operating mode.
*/
                    UAVCAN_NODE_MODE mode;

                    /**
Not used currently.
*/
                    byte sub_mode;

                    /**
Vendor-specific status information.
*/
                    ushort vendor_specific_status_code;
                }

                /**
General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
is available at http://uavcan.org.
*/
                class UAVCAN_NODE_INFO
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Time since the start-up of the node.
*/
                    uint uptime_sec;

                    /**
Node name string. For example, "sapog.px4.io".
*/
                    string name;

                    /**
Hardware major version number.
*/
                    byte hw_version_major;

                    /**
Hardware minor version number.
*/
                    byte hw_version_minor;

                    /**
Hardware unique 128-bit ID.
*/
                    [Dims(+16)] byte hw_unique_id;

                    /**
Software major version number.
*/
                    byte sw_version_major;

                    /**
Software minor version number.
*/
                    byte sw_version_minor;

                    /**
Version control system (VCS) revision identifier (e.g. git short commit hash). 0 if unknown.
*/
                    uint sw_vcs_commit;
                }

                /**
Request to read the value of a parameter with either the param_id string id or param_index. PARAM_EXT_VALUE
should be emitted in response.
*/
                class PARAM_EXT_REQUEST_READ
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as string
*/
                    string param_id;

                    /**
Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)
*/
                    short param_index;
                }

                /**
Request all parameters of this component. All parameters should be emitted in response as PARAM_EXT_VALUE.
*/
                class PARAM_EXT_REQUEST_LIST
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;
                }

                /**
Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
recipient to keep track of received parameters and allows them to re-request missing parameters after
a loss or timeout.
*/
                class PARAM_EXT_VALUE
                {
                    /**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as string
*/
                    string param_id;

                    /**
Parameter value
*/
                    string param_value;

                    /**
Parameter type.
*/
                    MAV_PARAM_EXT_TYPE param_type;

                    /**
Total number of parameters
*/
                    ushort param_count;

                    /**
Index of this parameter
*/
                    ushort param_index;
                }

                /**
Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
setting a parameter value and the new value is the same as the current value, you will immediately get
a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
a PARAM_ACK_IN_PROGRESS in response.
*/
                class PARAM_EXT_SET
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as string
*/
                    string param_id;

                    /**
Parameter value
*/
                    string param_value;

                    /**
Parameter type.
*/
                    MAV_PARAM_EXT_TYPE param_type;
                }

                /**
Response from a PARAM_EXT_SET message.
*/
                class PARAM_EXT_ACK
                {
                    /**
Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
ID is stored as string
*/
                    string param_id;

                    /**
Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
*/
                    string param_value;

                    /**
Parameter type.
*/
                    MAV_PARAM_EXT_TYPE param_type;

                    /**
Result code.
*/
                    PARAM_ACK param_result;
                }

                /**
Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
*/
                class OBSTACLE_DISTANCE
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Class id of the distance sensor type.
*/
                    MAV_DISTANCE_SENSOR sensor_type;

                    /**
Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless otherwise
specified in the frame. A value of 0 is valid and means that the obstacle is practically touching the
sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not
used. In a array element, one unit corresponds to 1cm.
*/
                    [Dims(+72)] ushort distances;

                    /**
Angular width in degrees of each array element. Increment direction is clockwise. This field is ignored
if increment_f is non-zero.
*/
                    byte increment;

                    /**
Minimum distance the sensor can measure.
*/
                    ushort min_distance;

                    /**
Maximum distance the sensor can measure.
*/
                    ushort max_distance;

                    /**
Angular width in degrees of each array element as a float. If non-zero then this value is used instead
of the uint8_t increment field. Positive is clockwise direction, negative is counter-clockwise.
*/
                    float increment_f;

                    /**
Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward.
Positive is clockwise direction, negative is counter-clockwise.
*/
                    float angle_offset;

                    /**
Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL,
which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.
*/
                    MAV_FRAME frame;
                }

                /**
Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard
for aerial vehicles (http://www.ros.org/reps/rep-0147.html).
*/
                class ODOMETRY
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Coordinate frame of reference for the pose data.
*/
                    MAV_FRAME frame_id;

                    /**
Coordinate frame of reference for the velocity in free space (twist) data.
*/
                    MAV_FRAME child_frame_id;

                    /**
X Position
*/
                    float x;

                    /**
Y Position
*/
                    float y;

                    /**
Z Position
*/
                    float z;

                    /**
Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
*/
                    [Dims(+4)] float q;

                    /**
X linear speed
*/
                    float vx;

                    /**
Y linear speed
*/
                    float vy;

                    /**
Z linear speed
*/
                    float vz;

                    /**
Roll angular speed
*/
                    float rollspeed;

                    /**
Pitch angular speed
*/
                    float pitchspeed;

                    /**
Yaw angular speed
*/
                    float yawspeed;

                    /**
Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z,
roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If
unknown, assign NaN value to first element in the array.
*/
                    [Dims(+21)] float pose_covariance;

                    /**
Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy,
vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second
ROW, etc.). If unknown, assign NaN value to first element in the array.
*/
                    [Dims(+21)] float velocity_covariance;

                    /**
Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position,
velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects
a loop-closure and the estimate jumps.
*/
                    byte reset_counter;

                    /**
Type of estimator that is providing the odometry.
*/
                    MAV_ESTIMATOR_TYPE estimator_type;
                }

                /**
Describe a trajectory using an array of up-to 5 waypoints in the local frame (MAV_FRAME_LOCAL_NED).
*/
                class TRAJECTORY_REPRESENTATION_WAYPOINTS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Number of valid points (up-to 5 waypoints are possible)
*/
                    byte valid_points;

                    /**
X-coordinate of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float pos_x;

                    /**
Y-coordinate of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float pos_y;

                    /**
Z-coordinate of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float pos_z;

                    /**
X-velocity of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float vel_x;

                    /**
Y-velocity of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float vel_y;

                    /**
Z-velocity of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float vel_z;

                    /**
X-acceleration of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float acc_x;

                    /**
Y-acceleration of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float acc_y;

                    /**
Z-acceleration of waypoint, set to NaN if not being used
*/
                    [Dims(+5)] float acc_z;

                    /**
Yaw angle, set to NaN if not being used
*/
                    [Dims(+5)] float pos_yaw;

                    /**
Yaw rate, set to NaN if not being used
*/
                    [Dims(+5)] float vel_yaw;

                    /**
MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
*/
                    MAV_CMD command;
                }

                /**
Describe a trajectory using an array of up-to 5 bezier control points in the local frame (MAV_FRAME_LOCAL_NED).
*/
                class TRAJECTORY_REPRESENTATION_BEZIER
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Number of valid control points (up-to 5 points are possible)
*/
                    byte valid_points;

                    /**
X-coordinate of bezier control points. Set to NaN if not being used
*/
                    [Dims(+5)] float pos_x;

                    /**
Y-coordinate of bezier control points. Set to NaN if not being used
*/
                    [Dims(+5)] float pos_y;

                    /**
Z-coordinate of bezier control points. Set to NaN if not being used
*/
                    [Dims(+5)] float pos_z;

                    /**
Bezier time horizon. Set to NaN if velocity/acceleration should not be incorporated
*/
                    [Dims(+5)] float delta;

                    /**
Yaw. Set to NaN for unchanged
*/
                    [Dims(+5)] float pos_yaw;
                }

                /**
Report current used cellular network status
*/
                class CELLULAR_STATUS
                {
                    /**
Cellular modem status
*/
                    CELLULAR_STATUS_FLAG status;

                    /**
Failure reason when status in in CELLUAR_STATUS_FAILED
*/
                    CELLULAR_NETWORK_FAILED_REASON failure_reason;

                    /**
Cellular network radio type: gsm, cdma, lte...
*/
                    CELLULAR_NETWORK_RADIO_TYPE Typ;

                    /**
Signal quality in percent. If unknown, set to UINT8_MAX
*/
                    byte quality;

                    /**
Mobile country code. If unknown, set to UINT16_MAX
*/
                    ushort mcc;

                    /**
Mobile network code. If unknown, set to UINT16_MAX
*/
                    ushort mnc;

                    /**
Location area code. If unknown, set to 0
*/
                    ushort lac;
                }

                /**
Status of the Iridium SBD link.
*/
                class ISBD_LINK_STATUS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong timestamp;

                    /**
Timestamp of the last successful sbd session. The receiving end can infer timestamp format (since 1.1.1970
or since system boot) by checking for the magnitude of the number.
*/
                    ulong last_heartbeat;

                    /**
Number of failed SBD sessions.
*/
                    ushort failed_sessions;

                    /**
Number of successful SBD sessions.
*/
                    ushort successful_sessions;

                    /**
Signal quality equal to the number of bars displayed on the ISU signal strength indicator. Range is 0
to 5, where 0 indicates no signal and 5 indicates maximum signal strength.
*/
                    byte signal_quality;

                    /**
1: Ring call pending, 0: No call pending.
*/
                    byte ring_pending;

                    /**
1: Transmission session pending, 0: No transmission session pending.
*/
                    byte tx_session_pending;

                    /**
1: Receiving session pending, 0: No receiving session pending.
*/
                    byte rx_session_pending;
                }

                /**
The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE.
*/
                class CELLULAR_CONFIG
                {
                    /**
Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a
response.
*/
                    byte enable_lte;

                    /**
Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when
sent back as a response.
*/
                    byte enable_pin;

                    /**
PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
*/
                    string pin;

                    /**
New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
*/
                    string new_pin;

                    /**
Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
*/
                    string apn;

                    /**
Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is
sent back as a response.
*/
                    string puk;

                    /**
Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back
as a response.
*/
                    byte roaming;

                    /**
Message acceptance response (sent back to GS).
*/
                    CELLULAR_CONFIG_RESPONSE response;
                }

                /**
RPM sensor data message.
*/
                class RAW_RPM
                {
                    /**
Index of this RPM sensor (0-indexed)
*/
                    byte index;

                    /**
Indicated rate
*/
                    float frequency;
                }

                /**
The global position resulting from GPS and sensor fusion.
*/
                class UTM_GLOBAL_POSITION
                {
                    /**
Time of applicability of position (microseconds since UNIX epoch).
*/
                    ulong time;

                    /**
Unique UAS ID.
*/
                    [Dims(+18)] byte uas_id;

                    /**
Latitude (WGS84)
*/
                    int lat;

                    /**
Longitude (WGS84)
*/
                    int lon;

                    /**
Altitude (WGS84)
*/
                    int alt;

                    /**
Altitude above ground
*/
                    int relative_alt;

                    /**
Ground X speed (latitude, positive north)
*/
                    short vx;

                    /**
Ground Y speed (longitude, positive east)
*/
                    short vy;

                    /**
Ground Z speed (altitude, positive down)
*/
                    short vz;

                    /**
Horizontal position uncertainty (standard deviation)
*/
                    ushort h_acc;

                    /**
Altitude uncertainty (standard deviation)
*/
                    ushort v_acc;

                    /**
Speed uncertainty (standard deviation)
*/
                    ushort vel_acc;

                    /**
Next waypoint, latitude (WGS84)
*/
                    int next_lat;

                    /**
Next waypoint, longitude (WGS84)
*/
                    int next_lon;

                    /**
Next waypoint, altitude (WGS84)
*/
                    int next_alt;

                    /**
Time until next update. Set to 0 if unknown or in data driven mode.
*/
                    ushort update_rate;

                    /**
Flight state
*/
                    UTM_FLIGHT_STATE flight_state;

                    /**
Bitwise OR combination of the data available flags.
*/
                    UTM_DATA_AVAIL_FLAGS flags;
                }

                /**
Large debug/prototyping array. The message uses the maximum available payload for data. The array_id and
name fields are used to discriminate between messages in code and in user interfaces (respectively).
Do not use in production code.
*/
                class DEBUG_FLOAT_ARRAY
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Name, for human-friendly display in a Ground Control Station
*/
                    string name;

                    /**
Unique ID used to discriminate between arrays
*/
                    ushort array_id;

                    /**
data
*/
                    [Dims(+58)] float Dat;
                }

                /**
Vehicle status report that is sent out while orbit execution is in progress (see MAV_CMD_DO_ORBIT).
*/
                class ORBIT_EXECUTION_STATUS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
*/
                    float radius;

                    /**
The coordinate system of the fields: x, y, z.
*/
                    MAV_FRAME frame;

                    /**
X coordinate of center point. Coordinate system depends on frame field: local = x position in meters *
1e4, global = latitude in degrees * 1e7.
*/
                    int x;

                    /**
Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters
* 1e4, global = latitude in degrees * 1e7.
*/
                    int y;

                    /**
Altitude of center point. Coordinate system depends on frame field.
*/
                    float z;
                }

                /**
Smart Battery information (static/infrequent update). Use for updates from: smart battery to flight stack,
flight stack to GCS. Use BATTERY_STATUS for smart battery frequent updates.
*/
                class SMART_BATTERY_INFO
                {
                    /**
Battery ID
*/
                    byte id;

                    /**
Function of the battery
*/
                    MAV_BATTERY_FUNCTION battery_function;

                    /**
Type (chemistry) of the battery
*/
                    MAV_BATTERY_TYPE Typ;

                    /**
Capacity when full according to manufacturer, -1: field not provided.
*/
                    int capacity_full_specification;

                    /**
Capacity when full (accounting for battery degradation), -1: field not provided.
*/
                    int capacity_full;

                    /**
Charge/discharge cycle count. UINT16_MAX: field not provided.
*/
                    ushort cycle_count;

                    /**
Serial number in ASCII characters, 0 terminated. All 0: field not provided.
*/
                    string serial_number;

                    /**
Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer
name then product name separated using an underscore.
*/
                    string device_name;

                    /**
Battery weight. 0: field not provided.
*/
                    ushort weight;

                    /**
Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
*/
                    ushort discharge_minimum_voltage;

                    /**
Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
*/
                    ushort charging_minimum_voltage;

                    /**
Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
*/
                    ushort resting_minimum_voltage;

                    /**
Maximum per-cell voltage when charged. 0: field not provided.
*/
                    ushort charging_maximum_voltage;

                    /**
Number of battery cells in series. 0: field not provided.
*/
                    byte cells_in_series;

                    /**
Maximum pack discharge current. 0: field not provided.
*/
                    uint discharge_maximum_current;

                    /**
Maximum pack discharge burst current. 0: field not provided.
*/
                    uint discharge_maximum_burst_current;

                    /**
Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
*/
                    string manufacture_date;
                }

                /**
Telemetry of power generation system. Alternator or mechanical generator.
*/
                class GENERATOR_STATUS
                {
                    /**
Status flags.
*/
                    MAV_GENERATOR_STATUS_FLAG status;

                    /**
Speed of electrical generator or alternator. UINT16_MAX: field not provided.
*/
                    ushort generator_speed;

                    /**
Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
*/
                    float battery_current;

                    /**
Current going to the UAV. If battery current not available this is the DC current from the generator.
Positive for out. Negative for in. NaN: field not provided
*/
                    float load_current;

                    /**
The power being generated. NaN: field not provided
*/
                    float power_generated;

                    /**
Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and
at a different voltage to main bus.
*/
                    float bus_voltage;

                    /**
The temperature of the rectifier or power converter. INT16_MAX: field not provided.
*/
                    short rectifier_temperature;

                    /**
The target battery current. Positive for out. Negative for in. NaN: field not provided
*/
                    float bat_current_setpoint;

                    /**
The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
*/
                    short generator_temperature;

                    /**
Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
*/
                    uint runtime;

                    /**
Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due.
INT32_MAX: field not provided.
*/
                    int time_until_maintenance;
                }

                /**
The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports). This message supersedes
SERVO_OUTPUT_RAW.
*/
                class ACTUATOR_OUTPUT_STATUS
                {
                    /**
Timestamp (since system boot).
*/
                    ulong time_usec;

                    /**
Active outputs
*/
                    uint active;

                    /**
Servo / motor output array values. Zero values indicate unused channels.
*/
                    [Dims(+32)] float actuator;
                }

                /**
Time/duration estimates for various events and actions given the current vehicle state and position.
*/
                class TIME_ESTIMATE_TO_TARGET
                {
                    /**
Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g.
RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.
*/
                    int safe_return;

                    /**
Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the
vehicle is landed, or that no time estimate available.
*/
                    int land;

                    /**
Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.
*/
                    int mission_next_item;

                    /**
Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.
*/
                    int mission_end;

                    /**
Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means
no action active and/or no estimate available.
*/
                    int commanded_action;
                }

                /**
Message for transporting "arbitrary" variable-length data from one component to another (broadcast is
not forbidden, but discouraged). The encoding of the data is usually extension specific, i.e. determined
by the source, and is usually not documented as part of the MAVLink specification.
*/
                class TUNNEL
                {
                    /**
System ID (can be 0 for broadcast, but this is discouraged)
*/
                    byte target_system;

                    /**
Component ID (can be 0 for broadcast, but this is discouraged)
*/
                    byte target_component;

                    /**
A code that identifies the content of the payload (0 for unknown, which is the default). If this code
is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the
MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater
than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
*/
                    MAV_TUNNEL_PAYLOAD_TYPE payload_type;

                    /**
Length of the data transported in payload
*/
                    byte payload_length;

                    /**
Variable length payload. The payload length is defined by payload_length. The entire content of this block
is opaque unless you understand the encoding specified by payload_type.
*/
                    [Dims(+128)] byte payload;
                }

                /**
A forwarded CAN frame as requested by MAV_CMD_CAN_FORWARD.
*/
                class CAN_FRAME
                {
                    /**
System ID.
*/
                    byte target_system;

                    /**
Component ID.
*/
                    byte target_component;

                    /**
Bus number
*/
                    byte bus;

                    /**
Frame length
*/
                    byte len;

                    /**
Frame ID
*/
                    uint id;

                    /**
Frame data
*/
                    [Dims(+8)] byte Dat;
                }

                /**
Hardware status sent by an onboard computer.
*/
                class ONBOARD_COMPUTER_STATUS
                {
                    /**
Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since
1.1.1970 or since system boot) by checking for the magnitude of the number.
*/
                    ulong time_usec;

                    /**
Time since system boot.
*/
                    uint uptime;

                    /**
Type of the onboard computer: 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer
backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.
*/
                    byte Typ;

                    /**
CPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused.
*/
                    [Dims(+8)] byte cpu_cores;

                    /**
Combined CPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load
that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field
is unused.
*/
                    [Dims(+10)] byte cpu_combined;

                    /**
GPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused.
*/
                    [Dims(+4)] byte gpu_cores;

                    /**
Combined GPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load
that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field
is unused.
*/
                    [Dims(+10)] byte gpu_combined;

                    /**
Temperature of the board. A value of INT8_MAX implies the field is unused.
*/
                    sbyte temperature_board;

                    /**
Temperature of the CPU core. A value of INT8_MAX implies the field is unused.
*/
                    [Dims(+8)] sbyte temperature_core;

                    /**
Fan speeds. A value of INT16_MAX implies the field is unused.
*/
                    [Dims(+4)] short fan_speed;

                    /**
Amount of used RAM on the component system. A value of UINT32_MAX implies the field is unused.
*/
                    uint ram_usage;

                    /**
Total amount of RAM on the component system. A value of UINT32_MAX implies the field is unused.
*/
                    uint ram_total;

                    /**
Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable). A value of
UINT32_MAX implies the field is unused.
*/
                    [Dims(+4)] uint storage_type;

                    /**
Amount of used storage space on the component system. A value of UINT32_MAX implies the field is unused.
*/
                    [Dims(+4)] uint storage_usage;

                    /**
Total amount of storage space on the component system. A value of UINT32_MAX implies the field is unused.
*/
                    [Dims(+4)] uint storage_total;

                    /**
Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh
proprietary
*/
                    [Dims(+6)] uint link_type;

                    /**
Network traffic from the component system. A value of UINT32_MAX implies the field is unused.
*/
                    [Dims(+6)] uint link_tx_rate;

                    /**
Network traffic to the component system. A value of UINT32_MAX implies the field is unused.
*/
                    [Dims(+6)] uint link_rx_rate;

                    /**
Network capacity from the component system. A value of UINT32_MAX implies the field is unused.
*/
                    [Dims(+6)] uint link_tx_max;

                    /**
Network capacity to the component system. A value of UINT32_MAX implies the field is unused.
*/
                    [Dims(+6)] uint link_rx_max;
                }

                /**
Note: Camera components should use CAMERA_INFORMATION instead, and autopilots may use both this message
and AUTOPILOT_VERSION.
*/
                class COMPONENT_INFORMATION
                {
                    /**
Timestamp (time since system boot).
*/
                    uint time_boot_ms;

                    /**
CRC32 of the general metadata file (general_metadata_uri).
*/
                    uint general_metadata_file_crc;

                    /**
MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with
xz. The file contains general component metadata, and may contain URI links for additional metadata (see
COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string
needs to be zero terminated.
*/
                    string general_metadata_uri;

                    /**
CRC32 of peripherals metadata file (peripherals_metadata_uri).
*/
                    uint peripherals_metadata_file_crc;

                    /**
(Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may
be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals
are in a separate file because the information must be generated dynamically at runtime. The string needs
to be zero terminated.
*/
                    string peripherals_metadata_uri;
                }

                /**
Play vehicle tone/tune (buzzer). Supersedes message PLAY_TUNE.
*/
                class PLAY_TUNE_V2
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Tune format
*/
                    TUNE_FORMAT format;

                    /**
Tune definition as a NULL-terminated string.
*/
                    string tune;
                }

                /**
Tune formats supported by vehicle. This should be emitted as response to MAV_CMD_REQUEST_MESSAGE.
*/
                class SUPPORTED_TUNES
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Bitfield of supported tune formats.
*/
                    TUNE_FORMAT format;
                }

                /**
Event message. Each new event from a particular component gets a new sequence number. The same message
might be sent multiple times if (re-)requested. Most events are broadcast, some can be specific to a
target component (as receivers keep track of the sequence for missed events, all events need to be broadcast.
Thus we use destination_component instead of target_component).
*/
                class EVENT
                {
                    /**
Component ID
*/
                    byte destination_component;

                    /**
System ID
*/
                    byte destination_system;

                    /**
Event ID (as defined in the component metadata)
*/
                    uint id;

                    /**
Timestamp (time since system boot when the event happened).
*/
                    uint event_time_boot_ms;

                    /**
Sequence number.
*/
                    ushort sequence;

                    /**
Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0,
Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled
= 9
*/
                    byte log_levels;

                    /**
Arguments (depend on event ID).
*/
                    [Dims(+40)] byte arguments;
                }

                /**
Regular broadcast for the current latest event sequence number for a component. This is used to check
for dropped events.
*/
                class CURRENT_EVENT_SEQUENCE
                {
                    /**
Sequence number.
*/
                    ushort sequence;

                    /**
Flag bitset.
*/
                    MAV_EVENT_CURRENT_SEQUENCE_FLAGS flags;
                }

                /**
Request one or more events to be (re-)sent. If first_sequence==last_sequence, only a single event is requested.
Note that first_sequence can be larger than last_sequence (because the sequence number can wrap). Each
sequence will trigger an EVENT or EVENT_ERROR response.
*/
                class REQUEST_EVENT
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
First sequence number of the requested event.
*/
                    ushort first_sequence;

                    /**
Last sequence number of the requested event.
*/
                    ushort last_sequence;
                }

                /**
Response to a REQUEST_EVENT in case of an error (e.g. the event is not available anymore).
*/
                class RESPONSE_EVENT_ERROR
                {
                    /**
System ID
*/
                    byte target_system;

                    /**
Component ID
*/
                    byte target_component;

                    /**
Sequence number.
*/
                    ushort sequence;

                    /**
Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
*/
                    ushort sequence_oldest_available;

                    /**
Error reason.
*/
                    MAV_EVENT_ERROR_REASON reason;
                }

                /**
A forwarded CANFD frame as requested by MAV_CMD_CAN_FORWARD. These are separated from CAN_FRAME as they
need different handling (eg. TAO handling)
*/
                class CANFD_FRAME
                {
                    /**
System ID.
*/
                    byte target_system;

                    /**
Component ID.
*/
                    byte target_component;

                    /**
bus number
*/
                    byte bus;

                    /**
Frame length
*/
                    byte len;

                    /**
Frame ID
*/
                    uint id;

                    /**
Frame data
*/
                    [Dims(+64)] byte Dat;
                }

                /**
Modify the filter of what CAN messages to forward over the mavlink. This can be used to make CAN forwarding
work well on low bandwith links. The filtering is applied on bits 8 to 24 of the CAN id (2nd and 3rd
bytes) which corresponds to the DroneCAN message ID for DroneCAN. Filters with more than 16 IDs can be
constructed by sending multiple CAN_FILTER_MODIFY messages.
*/
                class CAN_FILTER_MODIFY
                {
                    /**
System ID.
*/
                    byte target_system;

                    /**
Component ID.
*/
                    byte target_component;

                    /**
bus number
*/
                    byte bus;

                    /**
what operation to perform on the filter list. See CAN_FILTER_OP enum.
*/
                    CAN_FILTER_OP operation;

                    /**
number of IDs in filter list
*/
                    byte num_ids;

                    /**
filter IDs, length num_ids
*/
                    [Dims(+16)] ushort ids;
                }

                /**
Cumulative distance traveled for each reported wheel.
*/
                class WHEEL_DISTANCE
                {
                    /**
Timestamp (synced to UNIX time or since system boot).
*/
                    ulong time_usec;

                    /**
Number of wheels reported.
*/
                    byte count;

                    /**
Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease
them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions
must be agreed/understood by the endpoints.
*/
                    [Dims(+16)] double distance;
                }

                /**
Winch status.
*/
                class WINCH_STATUS
                {
                    /**
Timestamp (synced to UNIX time or since system boot).
*/
                    ulong time_usec;

                    /**
Length of line released. NaN if unknown
*/
                    float line_length;

                    /**
Speed line is being released or retracted. Positive values if being released, negative values if being
retracted, NaN if unknown
*/
                    float speed;

                    /**
Tension on the line. NaN if unknown
*/
                    float tension;

                    /**
Voltage of the battery supplying the winch. NaN if unknown
*/
                    float voltage;

                    /**
Current draw from the winch. NaN if unknown
*/
                    float current;

                    /**
Temperature of the motor. INT16_MAX if unknown
*/
                    short temperature;

                    /**
Status flags
*/
                    MAV_WINCH_STATUS_FLAG status;
                }

                /**
Data for filling the OpenDroneID Basic ID message. This and the below messages are primarily meant for
feeding data to/from an OpenDroneID implementation. E.g. https://github.com/opendroneid/opendroneid-core-c.
These messages are compatible with the ASTM Remote ID standard at https://www.astm.org/Standards/F3411.htm
and the ASD-STAN Direct Remote ID standard. The usage of these messages is documented at https://mavlink.io/en/services/opendroneid.html.
*/
                class OPEN_DRONE_ID_BASIC_ID
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
Indicates the format for the uas_id field of this message.
*/
                    MAV_ODID_ID_TYPE id_type;

                    /**
Indicates the type of UA (Unmanned Aircraft).
*/
                    MAV_ODID_UA_TYPE ua_type;

                    /**
UAS (Unmanned Aircraft System) ID following the format specified by id_type. Shall be filled with nulls
in the unused portion of the field.
*/
                    [Dims(+20)] byte uas_id;
                }

                /**
Data for filling the OpenDroneID Location message. The float data types are 32-bit IEEE 754. The Location
message provides the location, altitude, direction and speed of the aircraft.
*/
                class OPEN_DRONE_ID_LOCATION
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
Indicates whether the unmanned aircraft is on the ground or in the air.
*/
                    MAV_ODID_STATUS status;

                    /**
Direction over ground (not heading, but direction of movement) measured clockwise from true North: 0 -
35999 centi-degrees. If unknown: 36100 centi-degrees.
*/
                    ushort direction;

                    /**
Ground speed. Positive only. If unknown: 25500 cm/s. If speed is larger than 25425 cm/s, use 25425 cm/s.
*/
                    ushort speed_horizontal;

                    /**
The vertical speed. Up is positive. If unknown: 6300 cm/s. If speed is larger than 6200 cm/s, use 6200
cm/s. If lower than -6200 cm/s, use -6200 cm/s.
*/
                    short speed_vertical;

                    /**
Current latitude of the unmanned aircraft. If unknown: 0 (both Lat/Lon).
*/
                    int latitude;

                    /**
Current longitude of the unmanned aircraft. If unknown: 0 (both Lat/Lon).
*/
                    int longitude;

                    /**
The altitude calculated from the barometric pressue. Reference is against 29.92inHg or 1013.2mb. If unknown:
-1000 m.
*/
                    float altitude_barometric;

                    /**
The geodetic altitude as defined by WGS84. If unknown: -1000 m.
*/
                    float altitude_geodetic;

                    /**
Indicates the reference point for the height field.
*/
                    MAV_ODID_HEIGHT_REF height_reference;

                    /**
The current height of the unmanned aircraft above the take-off location or the ground as indicated by
height_reference. If unknown: -1000 m.
*/
                    float height;

                    /**
The accuracy of the horizontal position.
*/
                    MAV_ODID_HOR_ACC horizontal_accuracy;

                    /**
The accuracy of the vertical position.
*/
                    MAV_ODID_VER_ACC vertical_accuracy;

                    /**
The accuracy of the barometric altitude.
*/
                    MAV_ODID_VER_ACC barometer_accuracy;

                    /**
The accuracy of the horizontal and vertical speed.
*/
                    MAV_ODID_SPEED_ACC speed_accuracy;

                    /**
Seconds after the full hour with reference to UTC time. Typically the GPS outputs a time-of-week value
in milliseconds. First convert that to UTC and then convert for this field using ((float) (time_week_ms
% (60*60*1000))) / 1000. If unknown: 0xFFFF.
*/
                    float timestamp;

                    /**
The accuracy of the timestamps.
*/
                    MAV_ODID_TIME_ACC timestamp_accuracy;
                }

                /**
Data for filling the OpenDroneID Authentication message. The Authentication Message defines a field that
can provide a means of authenticity for the identity of the UAS (Unmanned Aircraft System). The Authentication
message can have two different formats. Five data pages are supported. For data page 0, the fields PageCount,
Length and TimeStamp are present and AuthData is only 17 bytes. For data page 1 through 15, PageCount,
Length and TimeStamp are not present and the size of AuthData is 23 bytes.
*/
                class OPEN_DRONE_ID_AUTHENTICATION
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
Indicates the type of authentication.
*/
                    MAV_ODID_AUTH_TYPE authentication_type;

                    /**
Allowed range is 0 - 15.
*/
                    byte data_page;

                    /**
This field is only present for page 0. Allowed range is 0 - 15. See the description of struct ODID_Auth_data
at https://github.com/opendroneid/opendroneid-core-c/blob/master/libopendroneid/opendroneid.h.
*/
                    byte last_page_index;

                    /**
This field is only present for page 0. Total bytes of authentication_data from all data pages. See the
description of struct ODID_Auth_data at https://github.com/opendroneid/opendroneid-core-c/blob/master/libopendroneid/opendroneid.h.
*/
                    byte length;

                    /**
This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
*/
                    uint timestamp;

                    /**
Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes.
Shall be filled with nulls in the unused portion of the field.
*/
                    [Dims(+23)] byte authentication_data;
                }

                /**
Data for filling the OpenDroneID Self ID message. The Self ID Message is an opportunity for the operator
to (optionally) declare their identity and purpose of the flight. This message can provide additional
information that could reduce the threat profile of a UA (Unmanned Aircraft) flying in a particular area
or manner.
*/
                class OPEN_DRONE_ID_SELF_ID
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
Indicates the type of the description field.
*/
                    MAV_ODID_DESC_TYPE description_type;

                    /**
Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused
portion of the field.
*/
                    string description;
                }

                /**
Data for filling the OpenDroneID System message. The System Message contains general system information
including the operator location and possible aircraft group information.
*/
                class OPEN_DRONE_ID_SYSTEM
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
Specifies the operator location type.
*/
                    MAV_ODID_OPERATOR_LOCATION_TYPE operator_location_type;

                    /**
Specifies the classification type of the UA.
*/
                    MAV_ODID_CLASSIFICATION_TYPE classification_type;

                    /**
Latitude of the operator. If unknown: 0 (both Lat/Lon).
*/
                    int operator_latitude;

                    /**
Longitude of the operator. If unknown: 0 (both Lat/Lon).
*/
                    int operator_longitude;

                    /**
Number of aircraft in the area, group or formation (default 1).
*/
                    ushort area_count;

                    /**
Radius of the cylindrical area of the group or formation (default 0).
*/
                    ushort area_radius;

                    /**
Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
*/
                    float area_ceiling;

                    /**
Area Operations Floor relative to WGS84. If unknown: -1000 m.
*/
                    float area_floor;

                    /**
When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
*/
                    MAV_ODID_CATEGORY_EU category_eu;

                    /**
When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
*/
                    MAV_ODID_CLASS_EU class_eu;

                    /**
Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
*/
                    float operator_altitude_geo;
                }

                /**
Data for filling the OpenDroneID Operator ID message, which contains the CAA (Civil Aviation Authority)
issued operator ID.
*/
                class OPEN_DRONE_ID_OPERATOR_ID
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
Indicates the type of the operator_id field.
*/
                    MAV_ODID_OPERATOR_ID_TYPE operator_id_type;

                    /**
Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused
portion of the field.
*/
                    string operator_id;
                }

                /**
An OpenDroneID message pack is a container for multiple encoded OpenDroneID messages (i.e. not in the
format given for the above messages descriptions but after encoding into the compressed OpenDroneID byte
format). Used e.g. when transmitting on Bluetooth 5.0 Long Range/Extended Advertising or on WiFi Neighbor
Aware Networking.
*/
                class OPEN_DRONE_ID_MESSAGE_PACK
                {
                    /**
System ID (0 for broadcast).
*/
                    byte target_system;

                    /**
Component ID (0 for broadcast).
*/
                    byte target_component;

                    /**
Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html.

*/
                    [Dims(+20)] byte id_or_mac;

                    /**
This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specificed
to have this length.
*/
                    byte single_message_size;

                    /**
Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
*/
                    byte msg_pack_size;

                    /**
Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the
field.
*/
                    [Dims(+225)] byte messages;
                }

                /**
Temperature and humidity from hygrometer.
*/
                class HYGROMETER_SENSOR
                {
                    /**
Hygrometer ID
*/
                    byte id;

                    /**
Temperature
*/
                    short temperature;

                    /**
Humidity
*/
                    ushort humidity;
                }

                /**
State flags for ADS-B transponder dynamic report
*/
                enum UAVIONIX_ADSB_OUT_DYNAMIC_STATE
                {
                    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE        = 1,
                    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED    = 2,
                    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4,
                    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND            = 8,
                    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT                = 16,
                }

                /**
Transceiver RF control flags for ADS-B transponder dynamic reports
*/
                enum UAVIONIX_ADSB_OUT_RF_SELECT
                {
                    UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = 0, UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1, UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2,
                }

                /**
Status for ADS-B transponder dynamic input
*/
                enum UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX
                {
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0,
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1,
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D     = 2,
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D     = 3,
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS   = 4,
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK    = 5,
                }

                /**
Status flags for ADS-B transponder dynamic output
*/
                enum UAVIONIX_ADSB_RF_HEALTH
                {
                    UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0,
                    UAVIONIX_ADSB_RF_HEALTH_OK           = 1,
                    UAVIONIX_ADSB_RF_HEALTH_FAIL_TX      = 2,
                    UAVIONIX_ADSB_RF_HEALTH_FAIL_RX      = 16,
                }

                /**
Definitions for aircraft size
*/
                enum UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE
                {
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA     = 0,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M   = 1,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = 2,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M     = 3,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M     = 4,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M     = 5,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M   = 6,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M     = 7,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M     = 8,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M     = 9,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M   = 10,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M     = 11,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M  = 12,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M    = 13,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M    = 14,
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M    = 15,
                }

                /**
GPS lataral offset encoding
*/
                enum UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT
                {
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA  = 0,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M  = 1,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M  = 2,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M  = 3,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6,
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7,
                }

                /**
GPS longitudinal offset encoding
*/
                enum UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON { UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = 0, UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1, }

                /**
Emergency status encoding
*/
                enum UAVIONIX_ADSB_EMERGENCY_STATUS
                {
                    UAVIONIX_ADSB_OUT_NO_EMERGENCY                    = 0,
                    UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY               = 1,
                    UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY             = 2,
                    UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY          = 3,
                    UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY               = 4,
                    UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5,
                    UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY       = 6,
                    UAVIONIX_ADSB_OUT_RESERVED                        = 7,
                }

                /**
Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec thereafter)
*/
                class UAVIONIX_ADSB_OUT_CFG
                {
                    /**
Vehicle address (24 bit)
*/
                    uint ICAO;

                    /**
Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
*/
                    string callsign;

                    /**
Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
*/
                    ADSB_EMITTER_TYPE emitterType;

                    /**
Aircraft length and width encoding (table 2-35 of DO-282B)
*/
                    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE aircraftSize;

                    /**
GPS antenna lateral offset (table 2-36 of DO-282B)
*/
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT gpsOffsetLat;

                    /**
GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
one] (table 2-37 DO-282B)
*/
                    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON gpsOffsetLon;

                    /**
Aircraft stall speed in cm/s
*/
                    ushort stallSpeed;

                    /**
ADS-B transponder reciever and transmit enable flags
*/
                    UAVIONIX_ADSB_OUT_RF_SELECT rfSelect;
                }

                /**
Dynamic data used to generate ADS-B out transponder data (send at 5Hz)
*/
                class UAVIONIX_ADSB_OUT_DYNAMIC
                {
                    /**
UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
*/
                    uint utcTime;

                    /**
Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
*/
                    int gpsLat;

                    /**
Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
*/
                    int gpsLon;

                    /**
Altitude (WGS84). UP +ve. If unknown set to INT32_MAX
*/
                    int gpsAlt;

                    /**
0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
*/
                    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX gpsFix;

                    /**
Number of satellites visible. If unknown set to UINT8_MAX
*/
                    byte numSats;

                    /**
Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected
altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
*/
                    int baroAltMSL;

                    /**
Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
*/
                    uint accuracyHor;

                    /**
Vertical accuracy in cm. If unknown set to UINT16_MAX
*/
                    ushort accuracyVert;

                    /**
Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
*/
                    ushort accuracyVel;

                    /**
GPS vertical speed in cm/s. If unknown set to INT16_MAX
*/
                    short velVert;

                    /**
North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
*/
                    short velNS;

                    /**
East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
*/
                    short VelEW;

                    /**
Emergency status
*/
                    UAVIONIX_ADSB_EMERGENCY_STATUS emergencyStatus;

                    /**
ADS-B transponder dynamic input state flags
*/
                    UAVIONIX_ADSB_OUT_DYNAMIC_STATE state;

                    /**
Mode A code (typically 1200 [0x04B0] for VFR)
*/
                    ushort squawk;
                }

                /**
Transceiver heartbeat with health report (updated every 10s)
*/
                class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
                {
                    /**
ADS-B transponder messages
*/
                    UAVIONIX_ADSB_RF_HEALTH rfHealth;
                }

                enum MAV_CMD
                {
                    /**
Navigate to waypoint.
*/
                    MAV_CMD_NAV_WAYPOINT = 16,

                    /**
Loiter around this waypoint an unlimited amount of time
*/
                    MAV_CMD_NAV_LOITER_UNLIM = 17,

                    /**
Loiter around this waypoint for X turns
*/
                    MAV_CMD_NAV_LOITER_TURNS = 18,

                    /**
Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles
stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing)
circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero
forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.
*/
                    MAV_CMD_NAV_LOITER_TIME = 19,

                    /**
Return to launch location
*/
                    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,

                    /**
Land at location.
*/
                    MAV_CMD_NAV_LAND = 21,

                    /**
Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should
take off using the currently configured mode.
*/
                    MAV_CMD_NAV_TAKEOFF = 22,

                    /**
Land at local position (local frame only)
*/
                    MAV_CMD_NAV_LAND_LOCAL = 23,

                    /**
Takeoff from local position (local frame only)
*/
                    MAV_CMD_NAV_TAKEOFF_LOCAL = 24,

                    /**
Vehicle following, i.e. this waypoint represents the position of a moving vehicle
*/
                    MAV_CMD_NAV_FOLLOW = 25,

                    /**
Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
*/
                    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,

                    /**
Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
 Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until
heading toward the next waypoint.
*/
                    MAV_CMD_NAV_LOITER_TO_ALT = 31,

                    /**
Begin following a target
*/
                    MAV_CMD_DO_FOLLOW = 32,

                    /**
Reposition the MAV after a follow target command has been sent
*/
                    MAV_CMD_DO_FOLLOW_REPOSITION = 33,

                    /**
Start orbiting on the circumference of a circle defined by the parameters. Setting values to NaN/INT32_MAX
(as appropriate) results in using defaults.
*/
                    MAV_CMD_DO_ORBIT = 34,

                    /**
Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
vehicle's control system to control the vehicle attitude and the attitude of various sensors such as
cameras.
*/
                    MAV_CMD_NAV_ROI = 80,

                    /**
Control autonomous path planning on the MAV.
*/
                    MAV_CMD_NAV_PATHPLANNING = 81,

                    /**
Navigate to waypoint using a spline path.
*/
                    MAV_CMD_NAV_SPLINE_WAYPOINT = 82,

                    /**
Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command
should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).
*/
                    MAV_CMD_NAV_VTOL_TAKEOFF = 84,

                    /**
Land using VTOL mode
*/
                    MAV_CMD_NAV_VTOL_LAND = 85,

                    /**
hand control over to an external controller
*/
                    MAV_CMD_NAV_GUIDED_ENABLE = 92,

                    /**
Delay the next navigation command a number of seconds or until a specified time
*/
                    MAV_CMD_NAV_DELAY = 93,

                    /**
Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload
has reached the ground, and then releases the payload. If ground is not detected before the reaching
the maximum descent value (param1), the command will complete without releasing the payload.
*/
                    MAV_CMD_NAV_PAYLOAD_PLACE = 94,

                    /**
NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
*/
                    MAV_CMD_NAV_LAST = 95,

                    /**
Delay mission state machine.
*/
                    MAV_CMD_CONDITION_DELAY = 112,

                    /**
Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude
reached.
*/
                    MAV_CMD_CONDITION_CHANGE_ALT = 113,

                    /**
Delay mission state machine until within desired distance of next NAV point.
*/
                    MAV_CMD_CONDITION_DISTANCE = 114,

                    /**
Reach a certain target angle.
*/
                    MAV_CMD_CONDITION_YAW = 115,

                    /**
NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
*/
                    MAV_CMD_CONDITION_LAST = 159,

                    /**
Set system mode.
*/
                    MAV_CMD_DO_SET_MODE = 176,

                    /**
Jump to the desired command in the mission list.  Repeat this action only the specified number of times
*/
                    MAV_CMD_DO_JUMP = 177,

                    /**
Change speed and/or throttle set points.
*/
                    MAV_CMD_DO_CHANGE_SPEED = 178,

                    /**
Changes the home location either to the current location or a specified location.
*/
                    MAV_CMD_DO_SET_HOME = 179,

                    /**
Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
of the parameter.
*/
                    MAV_CMD_DO_SET_PARAMETER = 180,

                    /**
Set a relay to a condition.
*/
                    MAV_CMD_DO_SET_RELAY = 181,

                    /**
Cycle a relay on and off for a desired number of cycles with a desired period.
*/
                    MAV_CMD_DO_REPEAT_RELAY = 182,

                    /**
Set a servo to a desired PWM value.
*/
                    MAV_CMD_DO_SET_SERVO = 183,

                    /**
Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
*/
                    MAV_CMD_DO_REPEAT_SERVO = 184,

                    /**
0.5); the ACK should be either MAV_RESULT_FAILED or MAV_RESULT_UNSUPPORTED.
        
*/
                    MAV_CMD_DO_FLIGHTTERMINATION = 185,

                    /**
Change altitude set point.
*/
                    MAV_CMD_DO_CHANGE_ALTITUDE = 186,

                    /**
Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g.
on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
*/
                    MAV_CMD_DO_SET_ACTUATOR = 187,

                    /**
Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
will be used to help find the closest landing sequence.
*/
                    MAV_CMD_DO_LAND_START = 189,

                    /**
Mission command to perform a landing from a rally point.
*/
                    MAV_CMD_DO_RALLY_LAND = 190,

                    /**
Mission command to safely abort an autonomous landing.
*/
                    MAV_CMD_DO_GO_AROUND = 191,

                    /**
Reposition the vehicle to a specific WGS84 global position.
*/
                    MAV_CMD_DO_REPOSITION = 192,

                    /**
If in a GPS controlled position mode, hold the current position or continue.
*/
                    MAV_CMD_DO_PAUSE_CONTINUE = 193,

                    /**
Set moving direction to forward or reverse.
*/
                    MAV_CMD_DO_SET_REVERSE = 194,

                    /**
Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system
to control the vehicle attitude and the attitude of various sensors such as cameras. This command can
be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message.
*/
                    MAV_CMD_DO_SET_ROI_LOCATION = 195,

                    /**
Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This
can then be used by the vehicle's control system to control the vehicle attitude and the attitude of
various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device.
A gimbal device is not to react to this message.
*/
                    MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,

                    /**
Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This
can then be used by the vehicle's control system to control the vehicle attitude and the attitude of
various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device.
A gimbal device is not to react to this message. After this command the gimbal manager should go back
to manual input if available, and otherwise assume a neutral position.
*/
                    MAV_CMD_DO_SET_ROI_NONE = 197,

                    /**
Mount tracks system with specified system ID. Determination of target vehicle position may be done with
GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal
device. A gimbal device is not to react to this message.
*/
                    MAV_CMD_DO_SET_ROI_SYSID = 198,

                    /**
Control onboard camera system.
*/
                    MAV_CMD_DO_CONTROL_VIDEO = 200,

                    /**
Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
vehicle's control system to control the vehicle attitude and the attitude of various sensors such as
cameras.
*/
                    MAV_CMD_DO_SET_ROI = 201,

                    /**
Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX
messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
*/
                    MAV_CMD_DO_DIGICAM_CONFIGURE = 202,

                    /**
Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX
messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
*/
                    MAV_CMD_DO_DIGICAM_CONTROL = 203,

                    /**
Mission command to configure a camera or antenna mount
*/
                    MAV_CMD_DO_MOUNT_CONFIGURE = 204,

                    /**
Mission command to control a camera or antenna mount
*/
                    MAV_CMD_DO_MOUNT_CONTROL = 205,

                    /**
Mission command to set camera trigger distance for this flight. The camera is triggered each time this
distance is exceeded. This command can also be used to set the shutter integration time for the camera.
*/
                    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,

                    /**
Mission command to enable the geofence
*/
                    MAV_CMD_DO_FENCE_ENABLE = 207,

                    /**
Mission item/command to release a parachute or enable/disable auto release.
*/
                    MAV_CMD_DO_PARACHUTE = 208,

                    /**
Command to perform motor test.
*/
                    MAV_CMD_DO_MOTOR_TEST = 209,

                    /**
Change to/from inverted flight.
*/
                    MAV_CMD_DO_INVERTED_FLIGHT = 210,

                    /**
Mission command to operate a gripper.
*/
                    MAV_CMD_DO_GRIPPER = 211,

                    /**
Enable/disable autotune.
*/
                    MAV_CMD_DO_AUTOTUNE_ENABLE = 212,

                    /**
Sets a desired vehicle turn angle and speed change.
*/
                    MAV_CMD_NAV_SET_YAW_SPEED = 213,

                    /**
Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
triggered each time this interval expires. This command can also be used to set the shutter integration
time for the camera.
*/
                    MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,

                    /**
Mission command to control a camera or antenna mount, using a quaternion as reference.
*/
                    MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,

                    /**
set id of master controller
*/
                    MAV_CMD_DO_GUIDED_MASTER = 221,

                    /**
Set limits for external control
*/
                    MAV_CMD_DO_GUIDED_LIMITS = 222,

                    /**
Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
state. It is intended for vehicles with internal combustion engines
*/
                    MAV_CMD_DO_ENGINE_CONTROL = 223,

                    /**
Set the mission item with sequence number seq as current item. This means that the MAV will continue to
this mission item on the shortest path (not following the mission items in-between).
*/
                    MAV_CMD_DO_SET_MISSION_CURRENT = 224,

                    /**
NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
*/
                    MAV_CMD_DO_LAST = 240,

                    /**
Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
Calibration, only one sensor should be set in a single message and all others should be zero.
*/
                    MAV_CMD_PREFLIGHT_CALIBRATION = 241,

                    /**
Set sensor offsets. This command will be only accepted if in pre-flight mode.
*/
                    MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,

                    /**
Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the
legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial
vehicle configuration (it is not a normal pre-flight command and has been poorly named).
*/
                    MAV_CMD_PREFLIGHT_UAVCAN = 243,

                    /**
Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
mode.
*/
                    MAV_CMD_PREFLIGHT_STORAGE = 245,

                    /**
Request the reboot or shutdown of system components.
*/
                    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,

                    /**
Override current mission with command to pause mission, pause mission and move to position, continue/resume
mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether
it holds in place or moves to another position.
*/
                    MAV_CMD_OVERRIDE_GOTO = 252,

                    /**
Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose).
The camera is triggered each time this distance is exceeded, then the mount moves to the next position.
Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles
automatically roll the camera between shots to emulate an oblique camera setup (providing an increased
HFOV). This command can also be used to set the shutter integration time for the camera.
*/
                    MAV_CMD_OBLIQUE_SURVEY = 260,

                    /**
start running a mission
*/
                    MAV_CMD_MISSION_START = 300,

                    /**
Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of output
functions, i.e. it is possible to test Motor1 independent from which output it is configured on. Autopilots
typically refuse this command while armed.
*/
                    MAV_CMD_ACTUATOR_TEST = 310,

                    /**
Actuator configuration command.
*/
                    MAV_CMD_CONFIGURE_ACTUATOR = 311,

                    /**
Arms / Disarms a component
*/
                    MAV_CMD_COMPONENT_ARM_DISARM = 400,

                    /**
Instructs system to run pre-arm checks. This command should return MAV_RESULT_TEMPORARILY_REJECTED in
the case the system is armed, otherwise MAV_RESULT_ACCEPTED. Note that the return value from executing
this command does not indicate whether the vehicle is armable or not, just whether the system has successfully
run/is currently running the checks.  The result of the checks is reflected in the SYS_STATUS message.
*/
                    MAV_CMD_RUN_PREARM_CHECKS = 401,

                    /**
Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external
to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system
itself, e.g. an indicator light).
*/
                    MAV_CMD_ILLUMINATOR_ON_OFF = 405,

                    /**
Request the home position from the vehicle.
*/
                    MAV_CMD_GET_HOME_POSITION = 410,

                    /**
Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection
before accepting this command such as a specific param setting.
*/
                    MAV_CMD_INJECT_FAILURE = 420,

                    /**
Starts receiver pairing.
*/
                    MAV_CMD_START_RX_PAIR = 500,

                    /**
Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the
command and then emit its response in a MESSAGE_INTERVAL message.
*/
                    MAV_CMD_GET_MESSAGE_INTERVAL = 510,

                    /**
Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.
*/
                    MAV_CMD_SET_MESSAGE_INTERVAL = 511,

                    /**
Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version
of MAV_CMD_SET_MESSAGE_INTERVAL).
*/
                    MAV_CMD_REQUEST_MESSAGE = 512,

                    /**
Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their
capabilities in an PROTOCOL_VERSION message
*/
                    MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,

                    /**
Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in
an AUTOPILOT_VERSION message
*/
                    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,

                    /**
Request camera information (CAMERA_INFORMATION).
*/
                    MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,

                    /**
Request camera settings (CAMERA_SETTINGS).
*/
                    MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,

                    /**
Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific
component's storage.
*/
                    MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,

                    /**
Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's
target_component to target a specific component's storage.
*/
                    MAV_CMD_STORAGE_FORMAT = 526,

                    /**
Request camera capture status (CAMERA_CAPTURE_STATUS)
*/
                    MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,

                    /**
Request flight information (FLIGHT_INFORMATION)
*/
                    MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,

                    /**
Reset all camera settings to Factory Default
*/
                    MAV_CMD_RESET_CAMERA_SETTINGS = 529,

                    /**
Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
command after a mode change if the camera supports video streaming.
*/
                    MAV_CMD_SET_CAMERA_MODE = 530,

                    /**
Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
*/
                    MAV_CMD_SET_CAMERA_ZOOM = 531,

                    /**
Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
*/
                    MAV_CMD_SET_CAMERA_FOCUS = 532,

                    /**
A target system can choose to not allow a particular storage to be set as preferred storage, in which
case it should ACK the command with MAV_RESULT_DENIED.
*/
                    MAV_CMD_SET_STORAGE_USAGE = 533,

                    /**
Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
*/
                    MAV_CMD_JUMP_TAG = 600,

                    /**
Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A
mission should contain a single matching tag for each jump. If this is not the case then a jump to a
missing tag should complete the mission, and a jump where there are multiple matching tags should always
select the one with the lowest mission sequence number.
*/
                    MAV_CMD_DO_JUMP_TAG = 601,

                    /**
High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations
of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle
at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used
to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager.
*/
                    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,

                    /**
Gimbal configuration to set which sysid/compid is in primary and secondary control.
*/
                    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001,

                    /**
Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.
*/
                    MAV_CMD_IMAGE_START_CAPTURE = 2000,

                    /**
Stop image capture sequence Use NaN for reserved values.
*/
                    MAV_CMD_IMAGE_STOP_CAPTURE = 2001,

                    /**
Re-request a CAMERA_IMAGE_CAPTURED message.
*/
                    MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,

                    /**
Enable or disable on-board camera triggering system.
*/
                    MAV_CMD_DO_TRIGGER_CONTROL = 2003,

                    /**
If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command
allows to initiate the tracking.
*/
                    MAV_CMD_CAMERA_TRACK_POINT = 2004,

                    /**
If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this
command allows to initiate the tracking.
*/
                    MAV_CMD_CAMERA_TRACK_RECTANGLE = 2005,

                    /**
Stops ongoing tracking.
*/
                    MAV_CMD_CAMERA_STOP_TRACKING = 2010,

                    /**
Starts video capture (recording).
*/
                    MAV_CMD_VIDEO_START_CAPTURE = 2500,

                    /**
Stop the current video capture (recording).
*/
                    MAV_CMD_VIDEO_STOP_CAPTURE = 2501,

                    /**
Start video streaming
*/
                    MAV_CMD_VIDEO_START_STREAMING = 2502,

                    /**
Stop the given video stream
*/
                    MAV_CMD_VIDEO_STOP_STREAMING = 2503,

                    /**
Request video stream information (VIDEO_STREAM_INFORMATION)
*/
                    MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,

                    /**
Request video stream status (VIDEO_STREAM_STATUS)
*/
                    MAV_CMD_REQUEST_VIDEO_STREAM_STATUS = 2505,

                    /**
Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
*/
                    MAV_CMD_LOGGING_START = 2510,

                    /**
Request to stop streaming log data over MAVLink
*/
                    MAV_CMD_LOGGING_STOP = 2511,
                    MAV_CMD_AIRFRAME_CONFIGURATION = 2520,

                    /**
Request to start/stop transmitting over the high latency telemetry
*/
                    MAV_CMD_CONTROL_HIGH_LATENCY = 2600,

                    /**
Create a panorama at the current position
*/
                    MAV_CMD_PANORAMA_CREATE = 2800,

                    /**
Request VTOL transition
*/
                    MAV_CMD_DO_VTOL_TRANSITION = 3000,

                    /**
Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request
all data that is needs from the vehicle before authorize or deny the request. If approved the progress
of command_ack message should be set with period of time that this authorization is valid in seconds
or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
       

*/
                    MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,

                    /**
This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position
and altitude and the user can input the desired velocities along all three axes.
                  
*/
                    MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,

                    /**
This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the
center of the circle. The user can input the velocity along the circle and change the radius. If no input
is given the vehicle will hold position.
                  
*/
                    MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,

                    /**
Delay mission state machine until gate has been reached.
*/
                    MAV_CMD_CONDITION_GATE = 4501,

                    /**
Fence return point (there can only be one such point in a geofence definition). If rally points are supported
they should be used instead.
*/
                    MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,

                    /**
Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay
within this area. Minimum of 3 vertices required.
        
*/
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,

                    /**
Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay
outside this area. Minimum of 3 vertices required.
        
*/
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,

                    /**
Circular fence area. The vehicle must stay inside this area.
        
*/
                    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,

                    /**
Circular fence area. The vehicle must stay outside this area.
        
*/
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,

                    /**
Rally point. You can have multiple rally points defined.
        
*/
                    MAV_CMD_NAV_RALLY_POINT = 5100,

                    /**
Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
node that is online. Note that some of the response messages can be lost, which the receiver can detect
easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
received earlier; if not, this command should be sent again in order to request re-transmission of the
node information messages.
*/
                    MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,

                    /**
Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic
Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds
by the hardware per the Mode A, C, and S transponder spec.
*/
                    MAV_CMD_DO_ADSB_OUT_IDENT = 10001,

                    /**
Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
position and velocity.
*/
                    MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,

                    /**
Control the payload deployment.
*/
                    MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,

                    /**
Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field
tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero
then use the current vehicle location.
*/
                    MAV_CMD_FIXED_MAG_CAL_YAW = 42006,

                    /**
Command to operate winch.
*/
                    MAV_CMD_DO_WINCH = 42600,

                    /**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*/
                    MAV_CMD_WAYPOINT_USER_1 = 31000,

                    /**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*/
                    MAV_CMD_WAYPOINT_USER_2 = 31001,

                    /**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*/
                    MAV_CMD_WAYPOINT_USER_3 = 31002,

                    /**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*/
                    MAV_CMD_WAYPOINT_USER_4 = 31003,

                    /**
User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
*/
                    MAV_CMD_WAYPOINT_USER_5 = 31004,

                    /**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.
*/
                    MAV_CMD_SPATIAL_USER_1 = 31005,

                    /**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.
*/
                    MAV_CMD_SPATIAL_USER_2 = 31006,

                    /**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.
*/
                    MAV_CMD_SPATIAL_USER_3 = 31007,

                    /**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.
*/
                    MAV_CMD_SPATIAL_USER_4 = 31008,

                    /**
User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.
*/
                    MAV_CMD_SPATIAL_USER_5 = 31009,

                    /**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.
*/
                    MAV_CMD_USER_1 = 31010,

                    /**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.
*/
                    MAV_CMD_USER_2 = 31011,

                    /**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.
*/
                    MAV_CMD_USER_3 = 31012,

                    /**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.
*/
                    MAV_CMD_USER_4 = 31013,

                    /**
User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.
*/
                    MAV_CMD_USER_5 = 31014,

                    /**
Request forwarding of CAN packets from the given CAN bus to this component. CAN Frames are sent using
CAN_FRAME and CANFD_FRAME messages
*/
                    MAV_CMD_CAN_FORWARD = 32000,
                }

                interface MAV_CMD_PARAMS
                {
                    interface MAV_CMD_NAV_WAYPOINT
                    {
                        public interface param_1
                        {
                            public const string label       = "Hold";
                            public const string minValue    = "0";
                            public const string description = @"Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Accept Radius";
                            public const string minValue    = "0";
                            public const string description = @"Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)";
                        }

                        public interface param_3
                        {
                            public const string label = "Pass Radius";

                            public const string description = @"0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit.
Allows trajectory control.";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw";

                            public const string description = @"Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw
towards next waypoint, yaw to home, etc.).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Navigate to waypoint.";
                    }

                    interface MAV_CMD_NAV_LOITER_UNLIM
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string label = "Radius";

                            public const string description = @"Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter
clockwise, else counter-clockwise";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw";

                            public const string description = @"Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw
to home, etc.).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Loiter around this waypoint an unlimited amount of time";
                    }

                    interface MAV_CMD_NAV_LOITER_TURNS
                    {
                        public interface param_1
                        {
                            public const string label       = "Turns";
                            public const string minValue    = "0";
                            public const string description = @"Number of turns.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Heading Required";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Leave loiter circle only once heading towards the next waypoint (0 = False)";
                        }

                        public interface param_3
                        {
                            public const string label = "Radius";

                            public const string description = @"Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter
clockwise, else counter-clockwise";
                        }

                        public interface param_4
                        {
                            public const string label = "Xtrack Location";

                            public const string description = @"Loiter circle exit location and/or path to next waypoint (""xtrack"") for forward-only moving vehicles (not
multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the
line between the centers of the current and next waypoint), 1 to converge to the direct line between
the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in
degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave
the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Loiter around this waypoint for X turns";
                    }

                    interface MAV_CMD_NAV_LOITER_TIME
                    {
                        public interface param_1
                        {
                            public const string label       = "Time";
                            public const string minValue    = "0";
                            public const string description = @"Loiter time (only starts once Lat, Lon and Alt is reached).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Heading Required";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Leave loiter circle only once heading towards the next waypoint (0 = False)";
                        }

                        public interface param_3
                        {
                            public const string label = "Radius";

                            public const string description = @"Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter
clockwise, else counter-clockwise.";
                        }

                        public interface param_4
                        {
                            public const string label = "Xtrack Location";

                            public const string description = @"Loiter circle exit location and/or path to next waypoint (""xtrack"") for forward-only moving vehicles (not
multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the
line between the centers of the current and next waypoint), 1 to converge to the direct line between
the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in
degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave
the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles
stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing)
circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero
forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.";
                    }

                    interface MAV_CMD_NAV_RETURN_TO_LAUNCH
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Return to launch location";
                    }

                    interface MAV_CMD_NAV_LAND
                    {
                        public interface param_1
                        {
                            public const string label       = "Abort Alt";
                            public const string description = @"Minimum target altitude if landing is aborted (0 = undefined/use system default).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Land Mode";
                            public const string Enum        = "PRECISION_LAND_MODE";
                            public const string description = @"Precision land mode.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw Angle";

                            public const string description = @"Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw
to home, etc.).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude.";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Landing altitude (ground level in current frame).";
                        }

                        public const string description = @"Land at location.";
                    }

                    interface MAV_CMD_NAV_TAKEOFF
                    {
                        public interface param_1
                        {
                            public const string label       = "Pitch";
                            public const string description = @"Minimum pitch (if airspeed sensor present), desired pitch without sensor";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw";

                            public const string description = @"Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading
mode (e.g. yaw towards next waypoint, yaw to home, etc.).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should
take off using the currently configured mode.";
                    }

                    interface MAV_CMD_NAV_LAND_LOCAL
                    {
                        public interface param_1
                        {
                            public const string label       = "Target";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Landing target number (if available)";
                        }

                        public interface param_2
                        {
                            public const string label    = "Offset";
                            public const string minValue = "0";

                            public const string description = @"Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates:
d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position
and the position where the vehicle is about to land";
                        }

                        public interface param_3
                        {
                            public const string label       = "Descend Rate";
                            public const string description = @"Landing descend rate";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Desired yaw angle";
                        }

                        public interface param_5
                        {
                            public const string label       = "Y Position";
                            public const string description = @"Y-axis position";
                        }

                        public interface param_6
                        {
                            public const string label       = "X Position";
                            public const string description = @"X-axis position";
                        }

                        public interface param_7
                        {
                            public const string label       = "Z Position";
                            public const string description = @"Z-axis / ground level position";
                        }

                        public const string description = @"Land at local position (local frame only)";
                    }

                    interface MAV_CMD_NAV_TAKEOFF_LOCAL
                    {
                        public interface param_1
                        {
                            public const string label       = "Pitch";
                            public const string description = @"Minimum pitch (if airspeed sensor present), desired pitch without sensor";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string label       = "Ascend Rate";
                            public const string description = @"Takeoff ascend rate";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these";
                        }

                        public interface param_5
                        {
                            public const string label       = "Y Position";
                            public const string description = @"Y-axis position";
                        }

                        public interface param_6
                        {
                            public const string label       = "X Position";
                            public const string description = @"X-axis position";
                        }

                        public interface param_7
                        {
                            public const string label       = "Z Position";
                            public const string description = @"Z-axis position";
                        }

                        public const string description = @"Takeoff from local position (local frame only)";
                    }

                    interface MAV_CMD_NAV_FOLLOW
                    {
                        public interface param_1
                        {
                            public const string label       = "Following";
                            public const string increment   = "1";
                            public const string description = @"Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation";
                        }

                        public interface param_2
                        {
                            public const string label       = "Ground Speed";
                            public const string description = @"Ground speed of vehicle to be followed";
                        }

                        public interface param_3
                        {
                            public const string label       = "Radius";
                            public const string description = @"Radius around waypoint. If positive loiter clockwise, else counter-clockwise";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Desired yaw angle.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Vehicle following, i.e. this waypoint represents the position of a moving vehicle";
                    }

                    interface MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
                    {
                        public interface param_1
                        {
                            public const string label     = "Action";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "2";

                            public const string description = @"Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing,
command completes when at or above this command's altitude, 2 = Descending, command completes when at
or below this command's altitude.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Desired altitude";
                        }

                        public const string description = @"Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.";
                    }

                    interface MAV_CMD_NAV_LOITER_TO_ALT
                    {
                        public interface param_1
                        {
                            public const string label       = "Heading Required";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Leave loiter circle only once heading towards the next waypoint (0 = False)";
                        }

                        public interface param_2
                        {
                            public const string label = "Radius";

                            public const string description = @"Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter
clockwise, negative counter-clockwise, 0 means no change to standard loiter.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string label     = "Xtrack Location";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "1";

                            public const string description = @"Loiter circle exit location and/or path to next waypoint (""xtrack"") for forward-only moving vehicles (not
multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the
line between the centers of the current and next waypoint), 1 to converge to the direct line between
the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in
degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave
the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
 Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until
heading toward the next waypoint.";
                    }

                    interface MAV_CMD_DO_FOLLOW
                    {
                        public interface param_1
                        {
                            public const string label     = "System ID";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "255";

                            public const string description = @"System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position
hold mode.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string label     = "Altitude Mode";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "2";

                            public const string description = @"Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude
above home.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude above home. (used if mode=2)";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_7
                        {
                            public const string label       = "Time to Land";
                            public const string minValue    = "0";
                            public const string description = @"Time to land in which the MAV should go to the default position hold mode after a message RX timeout.";
                        }

                        public const string description = @"Begin following a target";
                    }

                    interface MAV_CMD_DO_FOLLOW_REPOSITION
                    {
                        public interface param_1
                        {
                            public const string label       = "Camera Q1";
                            public const string description = @"Camera q1 (where 0 is on the ray from the camera to the tracking device)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Camera Q2";
                            public const string description = @"Camera q2";
                        }

                        public interface param_3
                        {
                            public const string label       = "Camera Q3";
                            public const string description = @"Camera q3";
                        }

                        public interface param_4
                        {
                            public const string label       = "Camera Q4";
                            public const string description = @"Camera q4";
                        }

                        public interface param_5
                        {
                            public const string label       = "Altitude Offset";
                            public const string description = @"altitude offset from target";
                        }

                        public interface param_6
                        {
                            public const string label       = "X Offset";
                            public const string description = @"X offset from target";
                        }

                        public interface param_7
                        {
                            public const string label       = "Y Offset";
                            public const string description = @"Y offset from target";
                        }

                        public const string description = @"Reposition the MAV after a follow target command has been sent";
                    }

                    interface MAV_CMD_DO_ORBIT
                    {
                        public interface param_1
                        {
                            public const string label = "Radius";

                            public const string description = @"Radius of the circle. Positive: orbit clockwise. Negative: orbit counter-clockwise. NaN: Use vehicle default
radius, or current radius if already orbiting.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Velocity";
                            public const string description = @"Tangential Velocity. NaN: Use vehicle default velocity, or current velocity if already orbiting.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Yaw Behavior";
                            public const string Enum        = "ORBIT_YAW_BEHAVIOUR";
                            public const string description = @"Yaw behavior of the vehicle.";
                        }

                        public interface param_4
                        {
                            public const string label    = "Orbits";
                            public const string minValue = "0";
                            public const string Default  = "0";

                            public const string description = @"Orbit around the centre point for this many radians (i.e. for a three-quarter orbit set 270*Pi/180). 0:
Orbit forever. NaN: Use vehicle default, or current value if already orbiting.";
                        }

                        public interface param_5
                        {
                            public const string label = "Latitude/X";

                            public const string description = @"Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. INT32_MAX (or
NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already orbiting.";
                        }

                        public interface param_6
                        {
                            public const string label = "Longitude/Y";

                            public const string description = @"Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. INT32_MAX (or
NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already orbiting.";
                        }

                        public interface param_7
                        {
                            public const string label = "Altitude/Z";

                            public const string description = @"Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use
current vehicle altitude.";
                        }

                        public const string description = @"Start orbiting on the circumference of a circle defined by the parameters. Setting values to NaN/INT32_MAX
(as appropriate) results in using defaults.";
                    }

                    interface MAV_CMD_NAV_ROI
                    {
                        public interface param_1
                        {
                            public const string label       = "ROI Mode";
                            public const string Enum        = "MAV_ROI";
                            public const string description = @"Region of interest mode.";
                        }

                        public interface param_2
                        {
                            public const string label       = "WP Index";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Waypoint index/ target ID. (see MAV_ROI enum)";
                        }

                        public interface param_3
                        {
                            public const string label       = "ROI Index";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"ROI index (allows a vehicle to manage multiple ROI's)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "X";
                            public const string description = @"x the location of the fixed ROI (see MAV_FRAME)";
                        }

                        public interface param_6
                        {
                            public const string label       = "Y";
                            public const string description = @"y";
                        }

                        public interface param_7
                        {
                            public const string label       = "Z";
                            public const string description = @"z";
                        }

                        public const string description = @"Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
vehicle's control system to control the vehicle attitude and the attitude of various sensors such as
cameras.";
                    }

                    interface MAV_CMD_NAV_PATHPLANNING
                    {
                        public interface param_1
                        {
                            public const string label     = "Local Ctrl";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "2";

                            public const string description = @"0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path
planning, 2: Enable and reset local path planning";
                        }

                        public interface param_2
                        {
                            public const string label     = "Global Ctrl";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "3";

                            public const string description = @"0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid,
3: Enable and reset planned route, but not occupancy grid";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Yaw angle at goal";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude/X";
                            public const string description = @"Latitude/X of goal";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude/Y";
                            public const string description = @"Longitude/Y of goal";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude/Z";
                            public const string description = @"Altitude/Z of goal";
                        }

                        public const string description = @"Control autonomous path planning on the MAV.";
                    }

                    interface MAV_CMD_NAV_SPLINE_WAYPOINT
                    {
                        public interface param_1
                        {
                            public const string label       = "Hold";
                            public const string minValue    = "0";
                            public const string description = @"Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude/X";
                            public const string description = @"Latitude/X of goal";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude/Y";
                            public const string description = @"Longitude/Y of goal";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude/Z";
                            public const string description = @"Altitude/Z of goal";
                        }

                        public const string description = @"Navigate to waypoint using a spline path.";
                    }

                    interface MAV_CMD_NAV_VTOL_TAKEOFF
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string label       = "Transition Heading";
                            public const string Enum        = "VTOL_TRANSITION_HEADING";
                            public const string description = @"Front transition heading.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw Angle";

                            public const string description = @"Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home,
etc.).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command
should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).";
                    }

                    interface MAV_CMD_NAV_VTOL_LAND
                    {
                        public interface param_1
                        {
                            public const string label       = "Land Options";
                            public const string Enum        = "NAV_VTOL_LAND_OPTIONS";
                            public const string description = @"Landing behaviour.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string label       = "Approach Altitude";
                            public const string description = @"Approach altitude (with the same reference as the Altitude field). NaN if unspecified.";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw";

                            public const string description = @"Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home,
etc.).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label = "Ground Altitude";

                            public const string description = @"Altitude (ground level) relative to the current coordinate frame. NaN to use system default landing altitude
(ignore value).";
                        }

                        public const string description = @"Land using VTOL mode";
                    }

                    interface MAV_CMD_NAV_GUIDED_ENABLE
                    {
                        public interface param_1
                        {
                            public const string label       = "Enable";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0.5f on)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"hand control over to an external controller";
                    }

                    interface MAV_CMD_NAV_DELAY
                    {
                        public interface param_1
                        {
                            public const string label       = "Delay";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"Delay (-1 to enable time-of-day fields)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Hour";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string maxValue    = "23";
                            public const string description = @"hour (24h format, UTC, -1 to ignore)";
                        }

                        public interface param_3
                        {
                            public const string label       = "Minute";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string maxValue    = "59";
                            public const string description = @"minute (24h format, UTC, -1 to ignore)";
                        }

                        public interface param_4
                        {
                            public const string label       = "Second";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string maxValue    = "59";
                            public const string description = @"second (24h format, UTC, -1 to ignore)";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Delay the next navigation command a number of seconds or until a specified time";
                    }

                    interface MAV_CMD_NAV_PAYLOAD_PLACE
                    {
                        public interface param_1
                        {
                            public const string label       = "Max Descent";
                            public const string minValue    = "0";
                            public const string description = @"Maximum distance to descend.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload
has reached the ground, and then releases the payload. If ground is not detected before the reaching
the maximum descent value (param1), the command will complete without releasing the payload.";
                    }

                    interface MAV_CMD_NAV_LAST
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration";
                    }

                    interface MAV_CMD_CONDITION_DELAY
                    {
                        public interface param_1
                        {
                            public const string label       = "Delay";
                            public const string minValue    = "0";
                            public const string description = @"Delay";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Delay mission state machine.";
                    }

                    interface MAV_CMD_CONDITION_CHANGE_ALT
                    {
                        public interface param_1
                        {
                            public const string label       = "Rate";
                            public const string description = @"Descent / Ascend rate.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Target Altitude";
                        }

                        public const string description = @"Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude
reached.";
                    }

                    interface MAV_CMD_CONDITION_DISTANCE
                    {
                        public interface param_1
                        {
                            public const string label       = "Distance";
                            public const string minValue    = "0";
                            public const string description = @"Distance.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Delay mission state machine until within desired distance of next NAV point.";
                    }

                    interface MAV_CMD_CONDITION_YAW
                    {
                        public interface param_1
                        {
                            public const string label       = "Angle";
                            public const string description = @"target angle, 0 is north";
                        }

                        public interface param_2
                        {
                            public const string label       = "Angular Speed";
                            public const string description = @"angular speed";
                        }

                        public interface param_3
                        {
                            public const string label       = "Direction";
                            public const string increment   = "2";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"direction: -1: counter clockwise, 1: clockwise";
                        }

                        public interface param_4
                        {
                            public const string label       = "Relative";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: absolute angle, 1: relative offset";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Reach a certain target angle.";
                    }

                    interface MAV_CMD_CONDITION_LAST
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration";
                    }

                    interface MAV_CMD_DO_SET_MODE
                    {
                        public interface param_1
                        {
                            public const string label       = "Mode";
                            public const string Enum        = "MAV_MODE";
                            public const string description = @"Mode";
                        }

                        public interface param_2
                        {
                            public const string label       = "Custom Mode";
                            public const string description = @"Custom mode - this is system specific, please refer to the individual autopilot specifications for details.";
                        }

                        public interface param_3
                        {
                            public const string label = "Custom Submode";

                            public const string description = @"Custom sub mode - this is system specific, please refer to the individual autopilot specifications for
details.";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set system mode.";
                    }

                    interface MAV_CMD_DO_JUMP
                    {
                        public interface param_1
                        {
                            public const string label       = "Number";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Sequence number";
                        }

                        public interface param_2
                        {
                            public const string label       = "Repeat";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Repeat count";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Jump to the desired command in the mission list.  Repeat this action only the specified number of times";
                    }

                    interface MAV_CMD_DO_CHANGE_SPEED
                    {
                        public interface param_1
                        {
                            public const string label       = "Speed Type";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "3";
                            public const string description = @"Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Speed";
                            public const string minValue    = "-1";
                            public const string description = @"Speed (-1 indicates no change)";
                        }

                        public interface param_3
                        {
                            public const string label       = "Throttle";
                            public const string minValue    = "-1";
                            public const string description = @"Throttle (-1 indicates no change)";
                        }

                        public interface param_4
                        {
                            public const string label       = "Relative";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: absolute, 1: relative";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Change speed and/or throttle set points.";
                    }

                    interface MAV_CMD_DO_SET_HOME
                    {
                        public interface param_1
                        {
                            public const string label       = "Use Current";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Use current (1=use current location, 0=use specified location)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Yaw angle. NaN to use default heading";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Changes the home location either to the current location or a specified location.";
                    }

                    interface MAV_CMD_DO_SET_PARAMETER
                    {
                        public interface param_1
                        {
                            public const string label       = "Number";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Parameter number";
                        }

                        public interface param_2
                        {
                            public const string label       = "Value";
                            public const string description = @"Parameter value";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
of the parameter.";
                    }

                    interface MAV_CMD_DO_SET_RELAY
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Relay instance number.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Setting";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Setting. (1=on, 0=off, others possible depending on system hardware)";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set a relay to a condition.";
                    }

                    interface MAV_CMD_DO_REPEAT_RELAY
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Relay instance number.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Count";
                            public const string increment   = "1";
                            public const string minValue    = "1";
                            public const string description = @"Cycle count.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Time";
                            public const string minValue    = "0";
                            public const string description = @"Cycle time.";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Cycle a relay on and off for a desired number of cycles with a desired period.";
                    }

                    interface MAV_CMD_DO_SET_SERVO
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Servo instance number.";
                        }

                        public interface param_2
                        {
                            public const string label       = "PWM";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Pulse Width Modulation.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set a servo to a desired PWM value.";
                    }

                    interface MAV_CMD_DO_REPEAT_SERVO
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Servo instance number.";
                        }

                        public interface param_2
                        {
                            public const string label       = "PWM";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Pulse Width Modulation.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Count";
                            public const string increment   = "1";
                            public const string minValue    = "1";
                            public const string description = @"Cycle count.";
                        }

                        public interface param_4
                        {
                            public const string label       = "Time";
                            public const string minValue    = "0";
                            public const string description = @"Cycle time.";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.";
                    }

                    interface MAV_CMD_DO_FLIGHTTERMINATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Terminate";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0.5. Otherwise not activated and ACK with MAV_RESULT_FAILED.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"0.5); the ACK should be either MAV_RESULT_FAILED or MAV_RESULT_UNSUPPORTED.
        ";
                    }

                    interface MAV_CMD_DO_CHANGE_ALTITUDE
                    {
                        public interface param_1
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Frame";
                            public const string Enum        = "MAV_FRAME";
                            public const string description = @"Frame of new altitude.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Change altitude set point.";
                    }

                    interface MAV_CMD_DO_SET_ACTUATOR
                    {
                        public interface param_1
                        {
                            public const string label       = "Actuator 1";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Actuator 2";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Actuator 3";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.";
                        }

                        public interface param_4
                        {
                            public const string label       = "Actuator 4";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Actuator 5";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Actuator 6";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.";
                        }

                        public interface param_7
                        {
                            public const string label       = "Index";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)";
                        }

                        public const string description = @"Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g.
on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).";
                    }

                    interface MAV_CMD_DO_LAND_START
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
will be used to help find the closest landing sequence.";
                    }

                    interface MAV_CMD_DO_RALLY_LAND
                    {
                        public interface param_1
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Break altitude";
                        }

                        public interface param_2
                        {
                            public const string label       = "Speed";
                            public const string description = @"Landing speed";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to perform a landing from a rally point.";
                    }

                    interface MAV_CMD_DO_GO_AROUND
                    {
                        public interface param_1
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to safely abort an autonomous landing.";
                    }

                    interface MAV_CMD_DO_REPOSITION
                    {
                        public interface param_1
                        {
                            public const string label       = "Speed";
                            public const string minValue    = "-1";
                            public const string description = @"Ground speed, less than 0 (-1) for default";
                        }

                        public interface param_2
                        {
                            public const string label       = "Bitmask";
                            public const string Enum        = "MAV_DO_REPOSITION_FLAGS";
                            public const string description = @"Bitmask of option flags.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string label = "Yaw";

                            public const string description = @"Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home,
etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Reposition the vehicle to a specific WGS84 global position.";
                    }

                    interface MAV_CMD_DO_PAUSE_CONTINUE
                    {
                        public interface param_1
                        {
                            public const string label     = "Continue";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "1";

                            public const string description = @"0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable
vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default
loiter radius.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"If in a GPS controlled position mode, hold the current position or continue.";
                    }

                    interface MAV_CMD_DO_SET_REVERSE
                    {
                        public interface param_1
                        {
                            public const string label       = "Reverse";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Direction (0=Forward, 1=Reverse)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set moving direction to forward or reverse.";
                    }

                    interface MAV_CMD_DO_SET_ROI_LOCATION
                    {
                        public interface param_1
                        {
                            public const string label = "Gimbal device ID";

                            public const string description = @"Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude of ROI location";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude of ROI location";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude of ROI location";
                        }

                        public const string description = @"Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system
to control the vehicle attitude and the attitude of various sensors such as cameras. This command can
be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message.";
                    }

                    interface MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
                    {
                        public interface param_1
                        {
                            public const string label = "Gimbal device ID";

                            public const string description = @"Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "Pitch Offset";
                            public const string description = @"Pitch offset from next waypoint, positive pitching up";
                        }

                        public interface param_6
                        {
                            public const string label       = "Roll Offset";
                            public const string description = @"Roll offset from next waypoint, positive rolling to the right";
                        }

                        public interface param_7
                        {
                            public const string label       = "Yaw Offset";
                            public const string description = @"Yaw offset from next waypoint, positive yawing to the right";
                        }

                        public const string description = @"Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This
can then be used by the vehicle's control system to control the vehicle attitude and the attitude of
various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device.
A gimbal device is not to react to this message.";
                    }

                    interface MAV_CMD_DO_SET_ROI_NONE
                    {
                        public interface param_1
                        {
                            public const string label = "Gimbal device ID";

                            public const string description = @"Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This
can then be used by the vehicle's control system to control the vehicle attitude and the attitude of
various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device.
A gimbal device is not to react to this message. After this command the gimbal manager should go back
to manual input if available, and otherwise assume a neutral position.";
                    }

                    interface MAV_CMD_DO_SET_ROI_SYSID
                    {
                        public interface param_1
                        {
                            public const string label       = "System ID";
                            public const string increment   = "1";
                            public const string minValue    = "1";
                            public const string maxValue    = "255";
                            public const string description = @"System ID";
                        }

                        public interface param_2
                        {
                            public const string label = "Gimbal device ID";

                            public const string description = @"Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).";
                        }

                        public const string description = @"Mount tracks system with specified system ID. Determination of target vehicle position may be done with
GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal
device. A gimbal device is not to react to this message.";
                    }

                    interface MAV_CMD_DO_CONTROL_VIDEO
                    {
                        public interface param_1
                        {
                            public const string label       = "ID";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"Camera ID (-1 for all)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Transmission";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "2";
                            public const string description = @"Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw";
                        }

                        public interface param_3
                        {
                            public const string label       = "Interval";
                            public const string minValue    = "0";
                            public const string description = @"0: single images every n seconds";
                        }

                        public interface param_4
                        {
                            public const string label       = "Recording";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "2";
                            public const string description = @"Recording: 0: disabled, 1: enabled compressed, 2: enabled raw";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Control onboard camera system.";
                    }

                    interface MAV_CMD_DO_SET_ROI
                    {
                        public interface param_1
                        {
                            public const string label       = "ROI Mode";
                            public const string Enum        = "MAV_ROI";
                            public const string description = @"Region of interest mode.";
                        }

                        public interface param_2
                        {
                            public const string label       = "WP Index";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Waypoint index/ target ID (depends on param 1).";
                        }

                        public interface param_3
                        {
                            public const string label       = "ROI Index";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Region of interest index. (allows a vehicle to manage multiple ROI's)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude";
                        }

                        public interface param_6
                        {
                            public const string description = @"MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude";
                        }

                        public interface param_7
                        {
                            public const string description = @"MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude";
                        }

                        public const string description = @"Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
vehicle's control system to control the vehicle attitude and the attitude of various sensors such as
cameras.";
                    }

                    interface MAV_CMD_DO_DIGICAM_CONFIGURE
                    {
                        public interface param_1
                        {
                            public const string label       = "Mode";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Modes: P, TV, AV, M, Etc.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Shutter Speed";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Shutter speed: Divisor number for one second.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Aperture";
                            public const string minValue    = "0";
                            public const string description = @"Aperture: F stop number.";
                        }

                        public interface param_4
                        {
                            public const string label       = "ISO";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"ISO number e.g. 80, 100, 200, Etc.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Exposure";
                            public const string description = @"Exposure type enumerator.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Command Identity";
                            public const string description = @"Command Identity.";
                        }

                        public interface param_7
                        {
                            public const string label       = "Engine Cut-off";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Main engine cut-off time before camera trigger. (0 means no cut-off)";
                        }

                        public const string description = @"Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX
messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).";
                    }

                    interface MAV_CMD_DO_DIGICAM_CONTROL
                    {
                        public interface param_1
                        {
                            public const string label       = "Session Control";
                            public const string description = @"Session control e.g. show/hide lens";
                        }

                        public interface param_2
                        {
                            public const string label       = "Zoom Absolute";
                            public const string description = @"Zoom's absolute position";
                        }

                        public interface param_3
                        {
                            public const string label       = "Zoom Relative";
                            public const string description = @"Zooming step value to offset zoom from the current position";
                        }

                        public interface param_4
                        {
                            public const string label       = "Focus";
                            public const string description = @"Focus Locking, Unlocking or Re-locking";
                        }

                        public interface param_5
                        {
                            public const string label       = "Shoot Command";
                            public const string description = @"Shooting Command";
                        }

                        public interface param_6
                        {
                            public const string label       = "Command Identity";
                            public const string description = @"Command Identity";
                        }

                        public interface param_7
                        {
                            public const string label = "Shot ID";

                            public const string description = @"Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame
count.";
                        }

                        public const string description = @"Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX
messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).";
                    }

                    interface MAV_CMD_DO_MOUNT_CONFIGURE
                    {
                        public interface param_1
                        {
                            public const string label       = "Mode";
                            public const string Enum        = "MAV_MOUNT_MODE";
                            public const string description = @"Mount operation mode";
                        }

                        public interface param_2
                        {
                            public const string label       = "Stabilize Roll";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"stabilize roll? (1 = yes, 0 = no)";
                        }

                        public interface param_3
                        {
                            public const string label       = "Stabilize Pitch";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"stabilize pitch? (1 = yes, 0 = no)";
                        }

                        public interface param_4
                        {
                            public const string label       = "Stabilize Yaw";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"stabilize yaw? (1 = yes, 0 = no)";
                        }

                        public interface param_5
                        {
                            public const string label       = "Roll Input Mode";
                            public const string description = @"roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)";
                        }

                        public interface param_6
                        {
                            public const string label       = "Pitch Input Mode";
                            public const string description = @"pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)";
                        }

                        public interface param_7
                        {
                            public const string label       = "Yaw Input Mode";
                            public const string description = @"yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)";
                        }

                        public const string description = @"Mission command to configure a camera or antenna mount";
                    }

                    interface MAV_CMD_DO_MOUNT_CONTROL
                    {
                        public interface param_1
                        {
                            public const string label       = "Pitch";
                            public const string description = @"pitch depending on mount mode (degrees or degrees/second depending on pitch input).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Roll";
                            public const string description = @"roll depending on mount mode (degrees or degrees/second depending on roll input).";
                        }

                        public interface param_3
                        {
                            public const string label       = "Yaw";
                            public const string description = @"yaw depending on mount mode (degrees or degrees/second depending on yaw input).";
                        }

                        public interface param_4
                        {
                            public const string label       = "Altitude";
                            public const string description = @"altitude depending on mount mode.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"latitude, set if appropriate mount mode.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"longitude, set if appropriate mount mode.";
                        }

                        public interface param_7
                        {
                            public const string label       = "Mode";
                            public const string Enum        = "MAV_MOUNT_MODE";
                            public const string description = @"Mount mode.";
                        }

                        public const string description = @"Mission command to control a camera or antenna mount";
                    }

                    interface MAV_CMD_DO_SET_CAM_TRIGG_DIST
                    {
                        public interface param_1
                        {
                            public const string label       = "Distance";
                            public const string minValue    = "0";
                            public const string description = @"Camera trigger distance. 0 to stop triggering.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Shutter";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"Camera shutter integration time. -1 or 0 to ignore";
                        }

                        public interface param_3
                        {
                            public const string label       = "Trigger";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Trigger camera once immediately. (0 = no trigger, 1 = trigger)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to set camera trigger distance for this flight. The camera is triggered each time this
distance is exceeded. This command can also be used to set the shutter integration time for the camera.";
                    }

                    interface MAV_CMD_DO_FENCE_ENABLE
                    {
                        public interface param_1
                        {
                            public const string label       = "Enable";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "2";
                            public const string description = @"enable? (0=disable, 1=enable, 2=disable_floor_only)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to enable the geofence";
                    }

                    interface MAV_CMD_DO_PARACHUTE
                    {
                        public interface param_1
                        {
                            public const string label       = "Action";
                            public const string Enum        = "PARACHUTE_ACTION";
                            public const string description = @"Action";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission item/command to release a parachute or enable/disable auto release.";
                    }

                    interface MAV_CMD_DO_MOTOR_TEST
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "1";
                            public const string description = @"Motor instance number (from 1 to max number of motors on the vehicle).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Throttle Type";
                            public const string Enum        = "MOTOR_TEST_THROTTLE_TYPE";
                            public const string description = @"Throttle type (whether the Throttle Value in param3 is a percentage, PWM value, etc.)";
                        }

                        public interface param_3
                        {
                            public const string label       = "Throttle";
                            public const string description = @"Throttle value.";
                        }

                        public interface param_4
                        {
                            public const string label       = "Timeout";
                            public const string minValue    = "0";
                            public const string description = @"Timeout between tests that are run in sequence.";
                        }

                        public interface param_5
                        {
                            public const string label     = "Motor Count";
                            public const string increment = "1";
                            public const string minValue  = "0";

                            public const string description = @"Motor count. Number of motors to test in sequence: 0/1=one motor, 2= two motors, etc. The Timeout (param4)
is used between tests.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Test Order";
                            public const string Enum        = "MOTOR_TEST_ORDER";
                            public const string description = @"Motor test order.";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Command to perform motor test.";
                    }

                    interface MAV_CMD_DO_INVERTED_FLIGHT
                    {
                        public interface param_1
                        {
                            public const string label       = "Inverted";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Inverted flight. (0=normal, 1=inverted)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Change to/from inverted flight.";
                    }

                    interface MAV_CMD_DO_GRIPPER
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "1";
                            public const string description = @"Gripper instance number.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Action";
                            public const string Enum        = "GRIPPER_ACTIONS";
                            public const string description = @"Gripper action to perform.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to operate a gripper.";
                    }

                    interface MAV_CMD_DO_AUTOTUNE_ENABLE
                    {
                        public interface param_1
                        {
                            public const string label       = "Enable";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Enable (1: enable, 0:disable).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Axis";
                            public const string Enum        = "AUTOTUNE_AXIS";
                            public const string description = @"Specify which axis are autotuned. 0 indicates autopilot default settings.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty.";
                        }

                        public const string description = @"Enable/disable autotune.";
                    }

                    interface MAV_CMD_NAV_SET_YAW_SPEED
                    {
                        public interface param_1
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Yaw angle to adjust steering by.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Speed";
                            public const string description = @"Speed.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Angle";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Final angle. (0=absolute, 1=relative)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Sets a desired vehicle turn angle and speed change.";
                    }

                    interface MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL
                    {
                        public interface param_1
                        {
                            public const string label       = "Trigger Cycle";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"Camera trigger cycle time. -1 or 0 to ignore.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Shutter Integration";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
triggered each time this interval expires. This command can also be used to set the shutter integration
time for the camera.";
                    }

                    interface MAV_CMD_DO_MOUNT_CONTROL_QUAT
                    {
                        public interface param_1
                        {
                            public const string label       = "Q1";
                            public const string description = @"quaternion param q1, w (1 in null-rotation)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Q2";
                            public const string description = @"quaternion param q2, x (0 in null-rotation)";
                        }

                        public interface param_3
                        {
                            public const string label       = "Q3";
                            public const string description = @"quaternion param q3, y (0 in null-rotation)";
                        }

                        public interface param_4
                        {
                            public const string label       = "Q4";
                            public const string description = @"quaternion param q4, z (0 in null-rotation)";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to control a camera or antenna mount, using a quaternion as reference.";
                    }

                    interface MAV_CMD_DO_GUIDED_MASTER
                    {
                        public interface param_1
                        {
                            public const string label       = "System ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "255";
                            public const string description = @"System ID";
                        }

                        public interface param_2
                        {
                            public const string label       = "Component ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "255";
                            public const string description = @"Component ID";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"set id of master controller";
                    }

                    interface MAV_CMD_DO_GUIDED_LIMITS
                    {
                        public interface param_1
                        {
                            public const string label       = "Timeout";
                            public const string minValue    = "0";
                            public const string description = @"Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.";
                        }

                        public interface param_2
                        {
                            public const string label = "Min Altitude";

                            public const string description = @"Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will
continue. 0 means no lower altitude limit.";
                        }

                        public interface param_3
                        {
                            public const string label = "Max Altitude";

                            public const string description = @"Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will
continue. 0 means no upper altitude limit.";
                        }

                        public interface param_4
                        {
                            public const string label    = "Horiz. Move Limit";
                            public const string minValue = "0";

                            public const string description = @"Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command
was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set limits for external control";
                    }

                    interface MAV_CMD_DO_ENGINE_CONTROL
                    {
                        public interface param_1
                        {
                            public const string label       = "Start Engine";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: Stop engine, 1:Start Engine";
                        }

                        public interface param_2
                        {
                            public const string label       = "Cold Start";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: Warm start, 1:Cold start. Controls use of choke where applicable";
                        }

                        public interface param_3
                        {
                            public const string label    = "Height Delay";
                            public const string minValue = "0";

                            public const string description = @"Height delay. This is for commanding engine start only after the vehicle has gained the specified height.
Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no
delay.";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
state. It is intended for vehicles with internal combustion engines";
                    }

                    interface MAV_CMD_DO_SET_MISSION_CURRENT
                    {
                        public interface param_1
                        {
                            public const string label       = "Number";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Mission sequence value to set";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Set the mission item with sequence number seq as current item. This means that the MAV will continue to
this mission item on the shortest path (not following the mission items in-between).";
                    }

                    interface MAV_CMD_DO_LAST
                    {
                        public interface param_1
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"NOP - This command is only used to mark the upper limit of the DO commands in the enumeration";
                    }

                    interface MAV_CMD_PREFLIGHT_CALIBRATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Gyro Temperature";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "3";
                            public const string description = @"1: gyro calibration, 3: gyro temperature calibration";
                        }

                        public interface param_2
                        {
                            public const string label       = "Magnetometer";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"1: magnetometer calibration";
                        }

                        public interface param_3
                        {
                            public const string label       = "Ground Pressure";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"1: ground pressure calibration";
                        }

                        public interface param_4
                        {
                            public const string label       = "Remote Control";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"1: radio RC calibration, 2: RC trim calibration";
                        }

                        public interface param_5
                        {
                            public const string label     = "Accelerometer";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "4";

                            public const string description = @"1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4:
simple accelerometer calibration";
                        }

                        public interface param_6
                        {
                            public const string label       = "Compmot or Airspeed";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "2";
                            public const string description = @"1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration";
                        }

                        public interface param_7
                        {
                            public const string label       = "ESC or Baro";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "3";
                            public const string description = @"1: ESC calibration, 3: barometer temperature calibration";
                        }

                        public const string description = @"Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
Calibration, only one sensor should be set in a single message and all others should be zero.";
                    }

                    interface MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
                    {
                        public interface param_1
                        {
                            public const string label     = "Sensor Type";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "6";

                            public const string description = @"Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical
flow, 5: second magnetometer, 6: third magnetometer";
                        }

                        public interface param_2
                        {
                            public const string label       = "X Offset";
                            public const string description = @"X axis offset (or generic dimension 1), in the sensor's raw units";
                        }

                        public interface param_3
                        {
                            public const string label       = "Y Offset";
                            public const string description = @"Y axis offset (or generic dimension 2), in the sensor's raw units";
                        }

                        public interface param_4
                        {
                            public const string label       = "Z Offset";
                            public const string description = @"Z axis offset (or generic dimension 3), in the sensor's raw units";
                        }

                        public interface param_5
                        {
                            public const string label       = "4th Dimension";
                            public const string description = @"Generic dimension 4, in the sensor's raw units";
                        }

                        public interface param_6
                        {
                            public const string label       = "5th Dimension";
                            public const string description = @"Generic dimension 5, in the sensor's raw units";
                        }

                        public interface param_7
                        {
                            public const string label       = "6th Dimension";
                            public const string description = @"Generic dimension 6, in the sensor's raw units";
                        }

                        public const string description = @"Set sensor offsets. This command will be only accepted if in pre-flight mode.";
                    }

                    interface MAV_CMD_PREFLIGHT_UAVCAN
                    {
                        public interface param_1
                        {
                            public const string label       = "Actuator ID";
                            public const string description = @"1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the
legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial
vehicle configuration (it is not a normal pre-flight command and has been poorly named).";
                    }

                    interface MAV_CMD_PREFLIGHT_STORAGE
                    {
                        public interface param_1
                        {
                            public const string label     = "Parameter Storage";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "3";

                            public const string description = @"Parameter storage: 0: Read from flash/EEPROM, 1: Write current parameter data to flash/EEPROM, 2: Reset
to defaults, 3: Reset sensor calibration parameter data to factory default (or firmware default if not
available)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Mission Storage";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "2";
                            public const string description = @"Mission storage: 0: Read from FLASH/EEPROM, 1: Write current data to flash/EEPROM, 2: Reset to defaults";
                        }

                        public interface param_3
                        {
                            public const string label       = "Logging Rate";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"1: logging rate (e.g. set to 1000 for 1000 Hz logging)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
mode.";
                    }

                    interface MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                    {
                        public interface param_1
                        {
                            public const string label     = "Autopilot";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "3";

                            public const string description = @"0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep
it in the bootloader until upgraded.";
                        }

                        public interface param_2
                        {
                            public const string label     = "Companion";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "3";

                            public const string description = @"0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot
onboard computer and keep it in the bootloader until upgraded.";
                        }

                        public interface param_3
                        {
                            public const string label     = "Component action";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "3";

                            public const string description = @"0: Do nothing for component, 1: Reboot component, 2: Shutdown component, 3: Reboot component and keep
it in the bootloader until upgraded";
                        }

                        public interface param_4
                        {
                            public const string label       = "Component ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "255";
                            public const string description = @"MAVLink Component ID targeted in param3 (0 for all components).";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_7
                        {
                            public const string description = @"WIP: ID (e.g. camera ID -1 for all IDs)";
                        }

                        public const string description = @"Request the reboot or shutdown of system components.";
                    }

                    interface MAV_CMD_OVERRIDE_GOTO
                    {
                        public interface param_1
                        {
                            public const string label = "Continue";
                            public const string Enum  = "MAV_GOTO";

                            public const string description = @"MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE:
resume mission.";
                        }

                        public interface param_2
                        {
                            public const string label = "Position";
                            public const string Enum  = "MAV_GOTO";

                            public const string description = @"MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold
at specified position.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Frame";
                            public const string Enum        = "MAV_FRAME";
                            public const string description = @"Coordinate frame of hold point.";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Desired yaw angle.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude/X";
                            public const string description = @"Latitude/X position.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude/Y";
                            public const string description = @"Longitude/Y position.";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude/Z";
                            public const string description = @"Altitude/Z position.";
                        }

                        public const string description = @"Override current mission with command to pause mission, pause mission and move to position, continue/resume
mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether
it holds in place or moves to another position.";
                    }

                    interface MAV_CMD_OBLIQUE_SURVEY
                    {
                        public interface param_1
                        {
                            public const string label       = "Distance";
                            public const string minValue    = "0";
                            public const string description = @"Camera trigger distance. 0 to stop triggering.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Shutter";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string Default     = "0";
                            public const string description = @"Camera shutter integration time. 0 to ignore";
                        }

                        public interface param_3
                        {
                            public const string label       = "Min Interval";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "10000";
                            public const string Default     = "0";
                            public const string description = @"The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.";
                        }

                        public interface param_4
                        {
                            public const string label     = "Positions";
                            public const string increment = "1";
                            public const string minValue  = "2";

                            public const string description = @"Total number of roll positions at which the camera will capture photos (images captures spread evenly
across the limits defined by param5).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Roll Angle";
                            public const string minValue    = "0";
                            public const string Default     = "0";
                            public const string description = @"Angle limits that the camera can be rolled to left and right of center.";
                        }

                        public interface param_6
                        {
                            public const string label       = "Pitch Angle";
                            public const string minValue    = "-180";
                            public const string maxValue    = "180";
                            public const string Default     = "0";
                            public const string description = @"Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose).
The camera is triggered each time this distance is exceeded, then the mount moves to the next position.
Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles
automatically roll the camera between shots to emulate an oblique camera setup (providing an increased
HFOV). This command can also be used to set the shutter integration time for the camera.";
                    }

                    interface MAV_CMD_MISSION_START
                    {
                        public interface param_1
                        {
                            public const string label       = "First Item";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"first_item: the first mission item to run";
                        }

                        public interface param_2
                        {
                            public const string label       = "Last Item";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"last_item:  the last mission item to run (after this item is run, the mission ends)";
                        }

                        public const string description = @"start running a mission";
                    }

                    interface MAV_CMD_ACTUATOR_TEST
                    {
                        public interface param_1
                        {
                            public const string label    = "Value";
                            public const string minValue = "-1";
                            public const string maxValue = "1";

                            public const string description = @"Output value: 1 means maximum positive output, 0 to center servos or minimum motor thrust (expected to
spin), -1 for maximum negative (if not supported by the motors, i.e. motor is not reversible, smaller
than 0 maps to NaN). And NaN maps to disarmed (stop the motors).";
                        }

                        public interface param_2
                        {
                            public const string label    = "Timeout";
                            public const string minValue = "0";
                            public const string maxValue = "3";

                            public const string description = @"Timeout after which the test command expires and the output is restored to the previous value. A timeout
has to be set for safety reasons. A timeout of 0 means to restore the previous value immediately.";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_5
                        {
                            public const string label       = "Output Function";
                            public const string Enum        = "ACTUATOR_OUTPUT_FUNCTION";
                            public const string description = @"Actuator Output function";
                        }

                        public interface param_6
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public const string description = @"Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of output
functions, i.e. it is possible to test Motor1 independent from which output it is configured on. Autopilots
typically refuse this command while armed.";
                    }

                    interface MAV_CMD_CONFIGURE_ACTUATOR
                    {
                        public interface param_1
                        {
                            public const string label       = "Configuration";
                            public const string Enum        = "ACTUATOR_CONFIGURATION";
                            public const string description = @"Actuator configuration action";
                        }

                        public interface param_2
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_5
                        {
                            public const string label       = "Output Function";
                            public const string Enum        = "ACTUATOR_OUTPUT_FUNCTION";
                            public const string description = @"Actuator Output function";
                        }

                        public interface param_6
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "0";
                        }

                        public const string description = @"Actuator configuration command.";
                    }

                    interface MAV_CMD_COMPONENT_ARM_DISARM
                    {
                        public interface param_1
                        {
                            public const string label       = "Arm";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: disarm, 1: arm";
                        }

                        public interface param_2
                        {
                            public const string label     = "Force";
                            public const string increment = "21196";
                            public const string minValue  = "0";
                            public const string maxValue  = "21196";

                            public const string description = @"0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g.
allow arming to override preflight checks and disarming in flight)";
                        }

                        public const string description = @"Arms / Disarms a component";
                    }

                    interface MAV_CMD_RUN_PREARM_CHECKS
                    {
                        public const string description = @"Instructs system to run pre-arm checks. This command should return MAV_RESULT_TEMPORARILY_REJECTED in
the case the system is armed, otherwise MAV_RESULT_ACCEPTED. Note that the return value from executing
this command does not indicate whether the vehicle is armable or not, just whether the system has successfully
run/is currently running the checks.  The result of the checks is reflected in the SYS_STATUS message.";
                    }

                    interface MAV_CMD_ILLUMINATOR_ON_OFF
                    {
                        public interface param_1
                        {
                            public const string label       = "Enable";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: Illuminators OFF, 1: Illuminators ON";
                        }

                        public const string description = @"Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external
to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system
itself, e.g. an indicator light).";
                    }

                    interface MAV_CMD_GET_HOME_POSITION
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Request the home position from the vehicle.";
                    }

                    interface MAV_CMD_INJECT_FAILURE
                    {
                        public interface param_1
                        {
                            public const string label       = "Failure unit";
                            public const string Enum        = "FAILURE_UNIT";
                            public const string description = @"The unit which is affected by the failure.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Failure type";
                            public const string Enum        = "FAILURE_TYPE";
                            public const string description = @"The type how the failure manifests itself.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Instance";
                            public const string description = @"Instance affected by failure (0 to signal all).";
                        }

                        public const string description = @"Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection
before accepting this command such as a specific param setting.";
                    }

                    interface MAV_CMD_START_RX_PAIR
                    {
                        public interface param_1
                        {
                            public const string label       = "Spektrum";
                            public const string description = @"0:Spektrum.";
                        }

                        public interface param_2
                        {
                            public const string label       = "RC Type";
                            public const string Enum        = "RC_TYPE";
                            public const string description = @"RC type.";
                        }

                        public const string description = @"Starts receiver pairing.";
                    }

                    interface MAV_CMD_GET_MESSAGE_INTERVAL
                    {
                        public interface param_1
                        {
                            public const string label       = "Message ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "16777215";
                            public const string description = @"The MAVLink message ID";
                        }

                        public const string description = @"Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the
command and then emit its response in a MESSAGE_INTERVAL message.";
                    }

                    interface MAV_CMD_SET_MESSAGE_INTERVAL
                    {
                        public interface param_1
                        {
                            public const string label       = "Message ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "16777215";
                            public const string description = @"The MAVLink message ID";
                        }

                        public interface param_2
                        {
                            public const string label       = "Interval";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"The interval between two messages. Set to -1 to disable and 0 to request default rate.";
                        }

                        public interface param_7
                        {
                            public const string label     = "Response Target";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "2";

                            public const string description = @"Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended),
1: address of requestor, 2: broadcast.";
                        }

                        public const string description = @"Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.";
                    }

                    interface MAV_CMD_REQUEST_MESSAGE
                    {
                        public interface param_1
                        {
                            public const string label       = "Message ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "16777215";
                            public const string description = @"The MAVLink message ID of the requested message.";
                        }

                        public interface param_2
                        {
                            public const string label = "Req Param 1";

                            public const string description = @"Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested
message. By default assumed not used (0).";
                        }

                        public interface param_3
                        {
                            public const string label = "Req Param 2";

                            public const string description = @"The use of this parameter (if any), must be defined in the requested message. By default assumed not used
(0).";
                        }

                        public interface param_4
                        {
                            public const string label = "Req Param 3";

                            public const string description = @"The use of this parameter (if any), must be defined in the requested message. By default assumed not used
(0).";
                        }

                        public interface param_5
                        {
                            public const string label = "Req Param 4";

                            public const string description = @"The use of this parameter (if any), must be defined in the requested message. By default assumed not used
(0).";
                        }

                        public interface param_6
                        {
                            public const string label = "Req Param 5";

                            public const string description = @"The use of this parameter (if any), must be defined in the requested message. By default assumed not used
(0).";
                        }

                        public interface param_7
                        {
                            public const string label     = "Response Target";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "2";

                            public const string description = @"Target address for requested message (if message has target address fields). 0: Flight-stack default,
1: address of requestor, 2: broadcast.";
                        }

                        public const string description = @"Request the target system(s) emit a single instance of a specified message (i.e. a ""one-shot"" version
of MAV_CMD_SET_MESSAGE_INTERVAL).";
                    }

                    interface MAV_CMD_REQUEST_PROTOCOL_VERSION
                    {
                        public interface param_1
                        {
                            public const string label       = "Protocol";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"1: Request supported protocol versions by all nodes on the network";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their
capabilities in an PROTOCOL_VERSION message";
                    }

                    interface MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                    {
                        public interface param_1
                        {
                            public const string label       = "Version";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"1: Request autopilot version";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in
an AUTOPILOT_VERSION message";
                    }

                    interface MAV_CMD_REQUEST_CAMERA_INFORMATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Capabilities";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: No action 1: Request camera capabilities";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request camera information (CAMERA_INFORMATION).";
                    }

                    interface MAV_CMD_REQUEST_CAMERA_SETTINGS
                    {
                        public interface param_1
                        {
                            public const string label       = "Settings";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: No Action 1: Request camera settings";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request camera settings (CAMERA_SETTINGS).";
                    }

                    interface MAV_CMD_REQUEST_STORAGE_INFORMATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Storage ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Storage ID (0 for all, 1 for first, 2 for second, etc.)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Information";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: No Action 1: Request storage information";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific
component's storage.";
                    }

                    interface MAV_CMD_STORAGE_FORMAT
                    {
                        public interface param_1
                        {
                            public const string label       = "Storage ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Storage ID (1 for first, 2 for second, etc.)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Format";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Format storage (and reset image log). 0: No action 1: Format storage";
                        }

                        public interface param_3
                        {
                            public const string label     = "Reset Image Log";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "1";

                            public const string description = @"Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count
and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's
target_component to target a specific component's storage.";
                    }

                    interface MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
                    {
                        public interface param_1
                        {
                            public const string label       = "Capture Status";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: No Action 1: Request camera capture status";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request camera capture status (CAMERA_CAPTURE_STATUS)";
                    }

                    interface MAV_CMD_REQUEST_FLIGHT_INFORMATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Flight Information";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"1: Request flight information";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Request flight information (FLIGHT_INFORMATION)";
                    }

                    interface MAV_CMD_RESET_CAMERA_SETTINGS
                    {
                        public interface param_1
                        {
                            public const string label       = "Reset";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"0: No Action 1: Reset all settings";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (all remaining params)";
                        }

                        public const string description = @"Reset all camera settings to Factory Default";
                    }

                    interface MAV_CMD_SET_CAMERA_MODE
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved (Set to 0)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Camera Mode";
                            public const string Enum        = "CAMERA_MODE";
                            public const string description = @"Camera mode";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
command after a mode change if the camera supports video streaming.";
                    }

                    interface MAV_CMD_SET_CAMERA_ZOOM
                    {
                        public interface param_1
                        {
                            public const string label       = "Zoom Type";
                            public const string Enum        = "CAMERA_ZOOM_TYPE";
                            public const string description = @"Zoom type";
                        }

                        public interface param_2
                        {
                            public const string label       = "Zoom Value";
                            public const string description = @"Zoom value. The range of valid values depend on the zoom type.";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).";
                    }

                    interface MAV_CMD_SET_CAMERA_FOCUS
                    {
                        public interface param_1
                        {
                            public const string label       = "Focus Type";
                            public const string Enum        = "SET_FOCUS_TYPE";
                            public const string description = @"Focus type";
                        }

                        public interface param_2
                        {
                            public const string label       = "Focus Value";
                            public const string description = @"Focus value";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).";
                    }

                    interface MAV_CMD_SET_STORAGE_USAGE
                    {
                        public interface param_1
                        {
                            public const string label       = "Storage ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Storage ID (1 for first, 2 for second, etc.)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Usage";
                            public const string Enum        = "STORAGE_USAGE_FLAG";
                            public const string description = @"Usage flags";
                        }

                        public const string description = @"A target system can choose to not allow a particular storage to be set as preferred storage, in which
case it should ACK the command with MAV_RESULT_DENIED.";
                    }

                    interface MAV_CMD_JUMP_TAG
                    {
                        public interface param_1
                        {
                            public const string label       = "Tag";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Tag.";
                        }

                        public const string description = @"Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.";
                    }

                    interface MAV_CMD_DO_JUMP_TAG
                    {
                        public interface param_1
                        {
                            public const string label       = "Tag";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Target tag to jump to.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Repeat";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Repeat count.";
                        }

                        public const string description = @"Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A
mission should contain a single matching tag for each jump. If this is not the case then a jump to a
missing tag should complete the mission, and a jump where there are multiple matching tags should always
select the one with the lowest mission sequence number.";
                    }

                    interface MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
                    {
                        public interface param_1
                        {
                            public const string label    = "Pitch angle";
                            public const string minValue = "-180";
                            public const string maxValue = "180";

                            public const string description = @"Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for
LOCK mode).";
                        }

                        public interface param_2
                        {
                            public const string label    = "Yaw angle";
                            public const string minValue = "-180";
                            public const string maxValue = "180";

                            public const string description = @"Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK
mode).";
                        }

                        public interface param_3
                        {
                            public const string label       = "Pitch rate";
                            public const string description = @"Pitch rate (positive to pitch up).";
                        }

                        public interface param_4
                        {
                            public const string label       = "Yaw rate";
                            public const string description = @"Yaw rate (positive to yaw to the right).";
                        }

                        public interface param_5
                        {
                            public const string label       = "Gimbal manager flags";
                            public const string Enum        = "GIMBAL_MANAGER_FLAGS";
                            public const string description = @"Gimbal manager flags to use.";
                        }

                        public interface param_7
                        {
                            public const string label = "Gimbal device ID";

                            public const string description = @"Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).";
                        }

                        public const string description = @"High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations
of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle
at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used
to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager.";
                    }

                    interface MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
                    {
                        public interface param_1
                        {
                            public const string label = "sysid primary control";

                            public const string description = @"Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions
where the own sysid is still unknown), -3: remove control if currently in control).";
                        }

                        public interface param_2
                        {
                            public const string label = "compid primary control";

                            public const string description = @"Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
missions where the own sysid is still unknown), -3: remove control if currently in control).";
                        }

                        public interface param_3
                        {
                            public const string label = "sysid secondary control";

                            public const string description = @"Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
missions where the own sysid is still unknown), -3: remove control if currently in control).";
                        }

                        public interface param_4
                        {
                            public const string label = "compid secondary control";

                            public const string description = @"Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
missions where the own sysid is still unknown), -3: remove control if currently in control).";
                        }

                        public interface param_7
                        {
                            public const string label = "Gimbal device ID";

                            public const string description = @"Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
Send command multiple times for more than one gimbal (but not all gimbals).";
                        }

                        public const string description = @"Gimbal configuration to set which sysid/compid is in primary and secondary control.";
                    }

                    interface MAV_CMD_IMAGE_START_CAPTURE
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved (Set to 0)";
                        }

                        public interface param_2
                        {
                            public const string label    = "Interval";
                            public const string minValue = "0";

                            public const string description = @"Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware
(typically greater than 2 seconds).";
                        }

                        public interface param_3
                        {
                            public const string label       = "Total Images";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.";
                        }

                        public interface param_4
                        {
                            public const string label     = "Sequence Number";
                            public const string increment = "1";
                            public const string minValue  = "1";

                            public const string description = @"Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise
set to 0. Increment the capture ID for each capture command to prevent double captures when a command
is re-transmitted.";
                        }

                        public interface param_5
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_6
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.";
                    }

                    interface MAV_CMD_IMAGE_STOP_CAPTURE
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved (Set to 0)";
                        }

                        public interface param_2
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Stop image capture sequence Use NaN for reserved values.";
                    }

                    interface MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE
                    {
                        public interface param_1
                        {
                            public const string label       = "Number";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Sequence number for missing CAMERA_IMAGE_CAPTURED message";
                        }

                        public interface param_2
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Re-request a CAMERA_IMAGE_CAPTURED message.";
                    }

                    interface MAV_CMD_DO_TRIGGER_CONTROL
                    {
                        public interface param_1
                        {
                            public const string label       = "Enable";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"Trigger enable/disable (0 for disable, 1 for start), -1 to ignore";
                        }

                        public interface param_2
                        {
                            public const string label       = "Reset";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"1 to reset the trigger sequence, -1 or 0 to ignore";
                        }

                        public interface param_3
                        {
                            public const string label       = "Pause";
                            public const string increment   = "2";
                            public const string minValue    = "-1";
                            public const string maxValue    = "1";
                            public const string description = @"1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore";
                        }

                        public const string description = @"Enable or disable on-board camera triggering system.";
                    }

                    interface MAV_CMD_CAMERA_TRACK_POINT
                    {
                        public interface param_1
                        {
                            public const string label       = "Point x";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Point to track x value (normalized 0..1, 0 is left, 1 is right).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Point y";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Point to track y value (normalized 0..1, 0 is top, 1 is bottom).";
                        }

                        public interface param_3
                        {
                            public const string label       = "Radius";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Point radius (normalized 0..1, 0 is image left, 1 is image right).";
                        }

                        public const string description = @"If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command
allows to initiate the tracking.";
                    }

                    interface MAV_CMD_CAMERA_TRACK_RECTANGLE
                    {
                        public interface param_1
                        {
                            public const string label       = "Top left corner x";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).";
                        }

                        public interface param_2
                        {
                            public const string label       = "Top left corner y";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).";
                        }

                        public interface param_3
                        {
                            public const string label       = "Bottom right corner x";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).";
                        }

                        public interface param_4
                        {
                            public const string label       = "Bottom right corner y";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).";
                        }

                        public const string description = @"If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this
command allows to initiate the tracking.";
                    }

                    interface MAV_CMD_CAMERA_STOP_TRACKING
                    {
                        public const string description = @"Stops ongoing tracking.";
                    }

                    interface MAV_CMD_VIDEO_START_CAPTURE
                    {
                        public interface param_1
                        {
                            public const string label       = "Stream ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Video Stream ID (0 for all streams)";
                        }

                        public interface param_2
                        {
                            public const string label    = "Status Frequency";
                            public const string minValue = "0";

                            public const string description = @"Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise
frequency)";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_5
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_6
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Starts video capture (recording).";
                    }

                    interface MAV_CMD_VIDEO_STOP_CAPTURE
                    {
                        public interface param_1
                        {
                            public const string label       = "Stream ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Video Stream ID (0 for all streams)";
                        }

                        public interface param_2
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_5
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_6
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public const string description = @"Stop the current video capture (recording).";
                    }

                    interface MAV_CMD_VIDEO_START_STREAMING
                    {
                        public interface param_1
                        {
                            public const string label       = "Stream ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)";
                        }

                        public const string description = @"Start video streaming";
                    }

                    interface MAV_CMD_VIDEO_STOP_STREAMING
                    {
                        public interface param_1
                        {
                            public const string label       = "Stream ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)";
                        }

                        public const string description = @"Stop the given video stream";
                    }

                    interface MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Stream ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)";
                        }

                        public const string description = @"Request video stream information (VIDEO_STREAM_INFORMATION)";
                    }

                    interface MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
                    {
                        public interface param_1
                        {
                            public const string label       = "Stream ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)";
                        }

                        public const string description = @"Request video stream status (VIDEO_STREAM_STATUS)";
                    }

                    interface MAV_CMD_LOGGING_START
                    {
                        public interface param_1
                        {
                            public const string label       = "Format";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Format: 0: ULog";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public const string description = @"Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)";
                    }

                    interface MAV_CMD_LOGGING_STOP
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public const string description = @"Request to stop streaming log data over MAVLink";
                    }

                    interface MAV_CMD_AIRFRAME_CONFIGURATION
                    {
                        public interface param_1
                        {
                            public const string label       = "Landing Gear ID";
                            public const string increment   = "1";
                            public const string minValue    = "-1";
                            public const string description = @"Landing gear ID (default: 0, -1 for all)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Landing Gear Position";
                            public const string description = @"Landing gear position (Down: 0, Up: 1, NaN for no change)";
                        }

                        public interface param_3
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_4
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_5
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_6
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }

                        public interface param_7
                        {
                            public const string reserved = "true";
                            public const string Default  = "NaN";
                        }
                    }

                    interface MAV_CMD_CONTROL_HIGH_LATENCY
                    {
                        public interface param_1
                        {
                            public const string label       = "Enable";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Control transmission over high latency telemetry (0: stop, 1: start)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty";
                        }

                        public const string description = @"Request to start/stop transmitting over the high latency telemetry";
                    }

                    interface MAV_CMD_PANORAMA_CREATE
                    {
                        public interface param_1
                        {
                            public const string label       = "Horizontal Angle";
                            public const string description = @"Viewing angle horizontal of the panorama (+- 0.5 the total angle)";
                        }

                        public interface param_2
                        {
                            public const string label       = "Vertical Angle";
                            public const string description = @"Viewing angle vertical of panorama.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Horizontal Speed";
                            public const string description = @"Speed of the horizontal rotation.";
                        }

                        public interface param_4
                        {
                            public const string label       = "Vertical Speed";
                            public const string description = @"Speed of the vertical rotation.";
                        }

                        public const string description = @"Create a panorama at the current position";
                    }

                    interface MAV_CMD_DO_VTOL_TRANSITION
                    {
                        public interface param_1
                        {
                            public const string label       = "State";
                            public const string Enum        = "MAV_VTOL_STATE";
                            public const string description = @"The target VTOL state. For normal transitions, only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.";
                        }

                        public interface param_2
                        {
                            public const string label = "Immediate";

                            public const string description = @"Force immediate transition to the specified MAV_VTOL_STATE. 1: Force immediate, 0: normal transition.
Can be used, for example, to trigger an emergency ""Quadchute"". Caution: Can be dangerous/damage vehicle,
depending on autopilot implementation of this command.";
                        }

                        public const string description = @"Request VTOL transition";
                    }

                    interface MAV_CMD_ARM_AUTHORIZATION_REQUEST
                    {
                        public interface param_1
                        {
                            public const string label       = "System ID";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "255";
                            public const string description = @"Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle";
                        }

                        public const string description = @"Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request
all data that is needs from the vehicle before authorize or deny the request. If approved the progress
of command_ack message should be set with period of time that this authorization is valid in seconds
or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
       
";
                    }

                    interface MAV_CMD_SET_GUIDED_SUBMODE_STANDARD
                    {
                        public const string description = @"This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position
and altitude and the user can input the desired velocities along all three axes.
                  ";
                    }

                    interface MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE
                    {
                        public interface param_1
                        {
                            public const string label       = "Radius";
                            public const string description = @"Radius of desired circle in CIRCLE_MODE";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Target latitude of center of circle in CIRCLE_MODE";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Target longitude of center of circle in CIRCLE_MODE";
                        }

                        public const string description = @"This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the
center of the circle. The user can input the velocity along the circle and change the radius. If no input
is given the vehicle will hold position.
                  ";
                    }

                    interface MAV_CMD_CONDITION_GATE
                    {
                        public interface param_1
                        {
                            public const string label       = "Geometry";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Geometry: 0: orthogonal to path between previous and next waypoint.";
                        }

                        public interface param_2
                        {
                            public const string label       = "UseAltitude";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string maxValue    = "1";
                            public const string description = @"Altitude: 0: ignore altitude";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Delay mission state machine until gate has been reached.";
                    }

                    interface MAV_CMD_NAV_FENCE_RETURN_POINT
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Fence return point (there can only be one such point in a geofence definition). If rally points are supported
they should be used instead.";
                    }

                    interface MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
                    {
                        public interface param_1
                        {
                            public const string label       = "Vertex Count";
                            public const string increment   = "1";
                            public const string minValue    = "3";
                            public const string description = @"Polygon vertex count";
                        }

                        public interface param_2
                        {
                            public const string label     = "Inclusion Group";
                            public const string increment = "1";
                            public const string minValue  = "0";

                            public const string description = @"Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group,
must be the same for all points in each polygon";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay
within this area. Minimum of 3 vertices required.
        ";
                    }

                    interface MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
                    {
                        public interface param_1
                        {
                            public const string label       = "Vertex Count";
                            public const string increment   = "1";
                            public const string minValue    = "3";
                            public const string description = @"Polygon vertex count";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay
outside this area. Minimum of 3 vertices required.
        ";
                    }

                    interface MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                    {
                        public interface param_1
                        {
                            public const string label       = "Radius";
                            public const string description = @"Radius.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Inclusion Group";
                            public const string increment   = "1";
                            public const string minValue    = "0";
                            public const string description = @"Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Circular fence area. The vehicle must stay inside this area.
        ";
                    }

                    interface MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                    {
                        public interface param_1
                        {
                            public const string label       = "Radius";
                            public const string description = @"Radius.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Circular fence area. The vehicle must stay outside this area.
        ";
                    }

                    interface MAV_CMD_NAV_RALLY_POINT
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude";
                        }

                        public const string description = @"Rally point. You can have multiple rally points defined.
        ";
                    }

                    interface MAV_CMD_UAVCAN_GET_NODE_INFO
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public const string description = @"Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
node that is online. Note that some of the response messages can be lost, which the receiver can detect
easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
received earlier; if not, this command should be sent again in order to request re-transmission of the
node information messages.";
                    }

                    interface MAV_CMD_DO_ADSB_OUT_IDENT
                    {
                        public interface param_1
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved (set to 0)";
                        }

                        public const string description = @"Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic
Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds
by the hardware per the Mode A, C, and S transponder spec.";
                    }

                    interface MAV_CMD_PAYLOAD_PREPARE_DEPLOY
                    {
                        public interface param_1
                        {
                            public const string label     = "Operation Mode";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "2";

                            public const string description = @"Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it.
1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing
abort). 2: add payload deploy to existing deployment list.";
                        }

                        public interface param_2
                        {
                            public const string label    = "Approach Vector";
                            public const string minValue = "-1";
                            public const string maxValue = "360";

                            public const string description = @"Desired approach vector in compass heading. A negative value indicates the system can define the approach
vector at will.";
                        }

                        public interface param_3
                        {
                            public const string label    = "Ground Speed";
                            public const string minValue = "-1";

                            public const string description = @"Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet
minimum airspeed. A negative value indicates the system can define the ground speed at will.";
                        }

                        public interface param_4
                        {
                            public const string label    = "Altitude Clearance";
                            public const string minValue = "-1";

                            public const string description = @"Minimum altitude clearance to the release position. A negative value indicates the system can define the
clearance at will.";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
position and velocity.";
                    }

                    interface MAV_CMD_PAYLOAD_CONTROL_DEPLOY
                    {
                        public interface param_1
                        {
                            public const string label     = "Operation Mode";
                            public const string increment = "1";
                            public const string minValue  = "0";
                            public const string maxValue  = "101";

                            public const string description = @"Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100:
delete first payload deployment request. 101: delete all payload deployment requests.";
                        }

                        public interface param_2
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_3
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_4
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_5
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_6
                        {
                            public const string description = @"Reserved";
                        }

                        public interface param_7
                        {
                            public const string description = @"Reserved";
                        }

                        public const string description = @"Control the payload deployment.";
                    }

                    interface MAV_CMD_FIXED_MAG_CAL_YAW
                    {
                        public interface param_1
                        {
                            public const string label       = "Yaw";
                            public const string description = @"Yaw of vehicle in earth frame.";
                        }

                        public interface param_2
                        {
                            public const string label       = "CompassMask";
                            public const string description = @"CompassMask, 0 for all.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude.";
                        }

                        public interface param_4
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude.";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty.";
                        }

                        public const string description = @"Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field
tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero
then use the current vehicle location.";
                    }

                    interface MAV_CMD_DO_WINCH
                    {
                        public interface param_1
                        {
                            public const string label       = "Instance";
                            public const string increment   = "1";
                            public const string minValue    = "1";
                            public const string description = @"Winch instance number.";
                        }

                        public interface param_2
                        {
                            public const string label       = "Action";
                            public const string Enum        = "WINCH_ACTIONS";
                            public const string description = @"Action to perform.";
                        }

                        public interface param_3
                        {
                            public const string label       = "Length";
                            public const string description = @"Length of line to release (negative to wind).";
                        }

                        public interface param_4
                        {
                            public const string label       = "Rate";
                            public const string description = @"Release rate (negative to wind).";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty.";
                        }

                        public const string description = @"Command to operate winch.";
                    }

                    interface MAV_CMD_WAYPOINT_USER_1
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined waypoint item. Ground Station will show the Vehicle as flying through this item.";
                    }

                    interface MAV_CMD_WAYPOINT_USER_2
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined waypoint item. Ground Station will show the Vehicle as flying through this item.";
                    }

                    interface MAV_CMD_WAYPOINT_USER_3
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined waypoint item. Ground Station will show the Vehicle as flying through this item.";
                    }

                    interface MAV_CMD_WAYPOINT_USER_4
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined waypoint item. Ground Station will show the Vehicle as flying through this item.";
                    }

                    interface MAV_CMD_WAYPOINT_USER_5
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined waypoint item. Ground Station will show the Vehicle as flying through this item.";
                    }

                    interface MAV_CMD_SPATIAL_USER_1
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.";
                    }

                    interface MAV_CMD_SPATIAL_USER_2
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.";
                    }

                    interface MAV_CMD_SPATIAL_USER_3
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.";
                    }

                    interface MAV_CMD_SPATIAL_USER_4
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.";
                    }

                    interface MAV_CMD_SPATIAL_USER_5
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string label       = "Latitude";
                            public const string description = @"Latitude unscaled";
                        }

                        public interface param_6
                        {
                            public const string label       = "Longitude";
                            public const string description = @"Longitude unscaled";
                        }

                        public interface param_7
                        {
                            public const string label       = "Altitude";
                            public const string description = @"Altitude (MSL)";
                        }

                        public const string description = @"User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
ROI item.";
                    }

                    interface MAV_CMD_USER_1
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_6
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_7
                        {
                            public const string description = @"User defined";
                        }

                        public const string description = @"User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.";
                    }

                    interface MAV_CMD_USER_2
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_6
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_7
                        {
                            public const string description = @"User defined";
                        }

                        public const string description = @"User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.";
                    }

                    interface MAV_CMD_USER_3
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_6
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_7
                        {
                            public const string description = @"User defined";
                        }

                        public const string description = @"User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.";
                    }

                    interface MAV_CMD_USER_4
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_6
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_7
                        {
                            public const string description = @"User defined";
                        }

                        public const string description = @"User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.";
                    }

                    interface MAV_CMD_USER_5
                    {
                        public interface param_1
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_2
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_3
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_4
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_5
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_6
                        {
                            public const string description = @"User defined";
                        }

                        public interface param_7
                        {
                            public const string description = @"User defined";
                        }

                        public const string description = @"User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
item.";
                    }

                    interface MAV_CMD_CAN_FORWARD
                    {
                        public interface param_1
                        {
                            public const string label       = "bus";
                            public const string description = @"Bus number (0 to disable forwarding, 1 for first bus, 2 for 2nd bus, 3 for 3rd bus).";
                        }

                        public interface param_2
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_3
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_4
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_5
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_6
                        {
                            public const string description = @"Empty.";
                        }

                        public interface param_7
                        {
                            public const string description = @"Empty.";
                        }

                        public const string description = @"Request forwarding of CAN packets from the given CAN bus to this component. CAN Frames are sent using
CAN_FRAME and CANFD_FRAME messages";
                    }
                }

                interface SI_Unit
                {
                    interface time
                    {
                        const string s   = "s";   // seconds
                        const string ds  = "ds";  // deciseconds
                        const string cs  = "cs";  // centiseconds
                        const string ms  = "ms";  // milliseconds
                        const string us  = "us";  // microseconds
                        const string Hz  = "Hz";  // Herz
                        const string MHz = "MHz"; // Mega-Herz
                    }

                    interface distance
                    {
                        const string km    = "km";    // kilometres
                        const string dam   = "dam";   // decametres
                        const string m     = "m";     // metres
                        const string m_s   = "m/s";   // metres per second
                        const string m_s_s = "m/s/s"; // metres per second per second
                        const string m_s_5 = "m/s*5"; // metres per second * 5 required from dagar for HIGH_LATENCY2 message
                        const string dm    = "dm";    // decimetres
                        const string dm_s  = "dm/s";  // decimetres per second
                        const string cm    = "cm";    // centimetres
                        const string cm_2  = "cm^2";  // centimetres squared (typically used in variance)
                        const string cm_s  = "cm/s";  // centimetres per second
                        const string mm    = "mm";    // millimetres
                        const string mm_s  = "mm/s";  // millimetres per second
                        const string mm_h  = "mm/h";  // millimetres per hour
                    }

                    interface temperature
                    {
                        const string K     = "K";     // Kelvin
                        const string degC  = "degC";  // degrees Celsius
                        const string cdegC = "cdegC"; // centi degrees Celsius
                    }

                    interface angle
                    {
                        const string rad    = "rad";    // radians
                        const string rad_s  = "rad/s";  // radians per second
                        const string mrad_s = "mrad/s"; // milli-radians per second
                        const string deg    = "deg";    // degrees
                        const string deg_2  = "deg/2";  // degrees/2 required from dagar for HIGH_LATENCY2 message
                        const string deg_s  = "deg/s";  // degrees per second
                        const string cdeg   = "cdeg";   // centidegrees
                        const string cdeg_s = "cdeg/s"; // centidegrees per second
                        const string degE5  = "degE5";  // degrees * 10E5
                        const string degE7  = "degE7";  // degrees * 10E7
                        const string rpm    = "rpm";    // rotations per minute
                    }

                    interface electricity
                    {
                        const string V   = "V";   // Volt
                        const string cV  = "cV";  // centi-Volt
                        const string mV  = "mV";  // milli-Volt
                        const string A   = "A";   // Ampere
                        const string cA  = "cA";  // centi-Ampere
                        const string mA  = "mA";  // milli-Ampere
                        const string mAh = "mAh"; // milli-Ampere hour
                    }

                    interface magnetism
                    {
                        const string mT     = "mT";     // milli-Tesla
                        const string gauss  = "gauss";  // Gauss
                        const string mgauss = "mgauss"; // milli-Gauss
                    }

                    interface energy
                    {
                        const string hJ = "hJ"; // hecto-Joule
                    }

                    interface power
                    {
                        const string W = "W"; // Watt
                    }

                    interface force
                    {
                        const string mG = "mG"; // milli-G
                    }

                    interface mass
                    {
                        const string g  = "g";  // grams
                        const string kg = "kg"; // kilograms
                    }

                    interface pressure
                    {
                        const string Pa   = "Pa";   // Pascal
                        const string hPa  = "hPa";  // hecto-Pascal
                        const string kPa  = "kPa";  // kilo-Pascal
                        const string mbar = "mbar"; // millibar
                    }

                    interface ratio
                    {
                        const string percent      = "%";   // percent
                        const string decipercent  = "d%";  // decipercent
                        const string centipercent = "c%";  // centipercent
                        const string dB           = "dB";  // Deci-Bell
                        const string dBm          = "dBm"; // Deci-Bell-milliwatts
                    }

                    interface digital
                    {
                        const string KiB     = "KiB";     // Kibibyte (1024 bytes)
                        const string KiB_s   = "KiB/s";   // Kibibyte (1024 bytes) per second
                        const string MiB     = "MiB";     // Mebibyte (1024*1024 bytes)
                        const string MiB_s   = "MiB/s";   // Mebibyte (1024*1024 bytes) per second
                        const string bytes   = "bytes";   // bytes
                        const string bytes_s = "bytes/s"; // bytes per second
                        const string bits_s  = "bits/s";  // bits per second
                        const string pix     = "pix";     // pixels
                        const string dpix    = "dpix";    // decipixels
                    }

                    interface flow
                    {
                        const string g_min    = "g/min";    // grams/minute
                        const string cm_3_min = "cm^3/min"; // cubic centimetres/minute
                    }

                    interface volume
                    {
                        const string cm_3 = "cm^3"; // cubic centimetres
                    }
                }
            }
        }

        class MicroAirVehicle : InCS, InCPP
        {
            public interface CommunicationInterface { }
        }
    }
}