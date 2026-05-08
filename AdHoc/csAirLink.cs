using System;
using org.unirail.Meta;

namespace org.mavlink {

    /**
        <see cref = 'AIRLINK_AUTH' id = '52000'/>
        <see cref = 'AIRLINK_AUTH_RESPONSE' id = '52001'/>
    */
    public interface csAirLink {
[Flags]
enum AIRLINK_AUTH_RESPONSE_TYPE{

/**
Login or password error
*/
AIRLINK_ERROR_LOGIN_OR_PASS = 0, 

/**
Auth successful
*/
AIRLINK_AUTH_OK = 1, 

}

/**
Authorization package
*/
class AIRLINK_AUTH{

/**
Login
*/
[D(+50)] string  login;

/**
Password
*/
[D(+50)] string  password;

}

/**
Response to the authorization request
*/
class AIRLINK_AUTH_RESPONSE{

/**
Response type
*/
AIRLINK_AUTH_RESPONSE_TYPE resp_type;

}
struct SI_Unit
    {
        struct time
        {
            const string s   = "s";   // seconds
            const string ds  = "ds";  // deciseconds
            const string cs  = "cs";  // centiseconds
            const string ms  = "ms";  // milliseconds
            const string us  = "us";  // microseconds
            const string Hz  = "Hz";  // Herz
            const string MHz = "MHz"; // Mega-Herz
        }

        struct distance
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

        struct temperature
        {
            const string K     = "K";     // Kelvin
            const string degC  = "degC";  // degrees Celsius
            const string cdegC = "cdegC"; // centi degrees Celsius
        }

        struct angle
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

        struct electricity
        {
            const string V   = "V";   // Volt
            const string cV  = "cV";  // centi-Volt
            const string mV  = "mV";  // milli-Volt
            const string A   = "A";   // Ampere
            const string cA  = "cA";  // centi-Ampere
            const string mA  = "mA";  // milli-Ampere
            const string mAh = "mAh"; // milli-Ampere hour
        }

        struct magnetism
        {
            const string mT     = "mT";     // milli-Tesla
            const string gauss  = "gauss";  // Gauss
            const string mgauss = "mgauss"; // milli-Gauss
        }

        struct energy
        {
            const string hJ = "hJ"; // hecto-Joule
        }

        struct power
        {
            const string W = "W"; // Watt
        }

        struct force
        {
            const string mG = "mG"; // milli-G
        }

        struct mass
        {
            const string g  = "g";  // grams
            const string kg = "kg"; // kilograms
        }

        struct pressure
        {
            const string Pa   = "Pa";   // Pascal
            const string hPa  = "hPa";  // hecto-Pascal
            const string kPa  = "kPa";  // kilo-Pascal
            const string mbar = "mbar"; // millibar
        }

        struct ratio
        {
            const string percent      = "%";   // percent
            const string decipercent  = "d%";  // decipercent
            const string centipercent = "c%";  // centipercent
            const string dB           = "dB";  // Deci-Bell
            const string dBm          = "dBm"; // Deci-Bell-milliwatts
        }

        struct digital
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

        struct flow
        {
            const string g_min    = "g/min";    // grams/minute
            const string cm_3_min = "cm^3/min"; // cubic centimetres/minute
        }

        struct volume
        {
            const string cm_3 = "cm^3"; // cubic centimetres
        }
    }        /**
        <see cref = 'InTS'/>
        <see cref = 'InJAVA'/>
        <see cref = 'InCS'/>
        <see cref = 'InCPP'/>
        <see cref = 'InGO'/>
        <see cref = 'InRS'/>
        */
        struct GroundControl : Host { }
        /**
        <see cref = 'InTS'/>
        <see cref = 'InJAVA'/>
        <see cref = 'InCS'/>
        <see cref = 'InCPP'/>
        <see cref = 'InGO'/>
        <see cref = 'InRS'/>
        */
        struct MicroAirVehicle : Host { }

        // Either side can send any MAVLink message — non-transitional, no Master.
        interface CommunicationChannel : Connects<GroundControl, MicroAirVehicle> {
            [_____lr_____<@csAirLink>]
            struct Start { }
        }
    }
}
