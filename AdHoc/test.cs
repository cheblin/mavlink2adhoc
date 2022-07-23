using System;
using org.unirail.Meta;
 namespace org.mavlink {
public interface test{
 
/**
Test all field types
*/
class TEST_TYPES{

/**
char
*/
 char  c;

/**
string
*/
string  s;

/**
uint8_t
*/
 byte  U;

/**
uint16_t
*/
 ushort  U1;

/**
uint32_t
*/
 uint  U3;

/**
uint64_t
*/
 ulong  U6;

/**
int8_t
*/
 sbyte  s8;

/**
int16_t
*/
 short  s16;

/**
int32_t
*/
 int  s32;

/**
int64_t
*/
 long  s64;

/**
float
*/
 float  f;

/**
double
*/
 double  d;

/**
uint8_t_array
*/
[Dims( +3 )]  byte  u8_array;

/**
uint16_t_array
*/
[Dims( +3 )]  ushort  u16_array;

/**
uint32_t_array
*/
[Dims( +3 )]  uint  u32_array;

/**
uint64_t_array
*/
[Dims( +3 )]  ulong  u64_array;

/**
int8_t_array
*/
[Dims( +3 )]  sbyte  s8_array;

/**
int16_t_array
*/
[Dims( +3 )]  short  s16_array;

/**
int32_t_array
*/
[Dims( +3 )]  int  s32_array;

/**
int64_t_array
*/
[Dims( +3 )]  long  s64_array;

/**
float_array
*/
[Dims( +3 )]  float  f_array;

/**
double_array
*/
[Dims( +3 )]  double  d_array;

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
    }       /**
       <see cref = 'InTS'/>
       <see cref = 'InJAVA'/>
       <see cref = 'InCS'/>
       <see cref = 'InCPP'/>
       <see cref = 'InGO'/>
       <see cref = 'InRS'/>
       */
       struct GroundControl : Host{
           public interface ToMicroAirVehicle :_<TEST_TYPES>           {}
}
       /**
       <see cref = 'InTS'/>
       <see cref = 'InJAVA'/>
       <see cref = 'InCS'/>
       <see cref = 'InCPP'/>
       <see cref = 'InGO'/>
       <see cref = 'InRS'/>
       */
       struct MicroAirVehicle : Host {
           public interface ToGroundControl : GroundControl.ToMicroAirVehicle  {}
       }
		interface CommunicationChannel : Communication_Channel_Of <GroundControl.ToMicroAirVehicle, MicroAirVehicle.ToGroundControl > {}

}
}
