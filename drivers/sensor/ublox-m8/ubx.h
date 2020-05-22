/**
 * @file ubx.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-21
 * 
 * @copyright Copyright (c) 2020
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */

#ifndef __GNSS_UBLOX_UBX_H__
#define __GNSS_UBLOX_UBX_H__

#include <zephyr/types.h>
#include <sys/util.h>

//Registers
enum ubx_header {
	UBX_SYNCH_1 = 0xB5,
	UBX_SYNCH_2 = 0x62,
};

//The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
enum ubx_class {
	UBX_CLASS_NAV = 0x01,	 //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
	UBX_CLASS_RXM = 0x02,	 //Receiver Manager Messages: Satellite Status, RTC Status
	UBX_CLASS_INF = 0x04,	 //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
	UBX_CLASS_ACK = 0x05,	 //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
	UBX_CLASS_CFG = 0x06,	 //Configuration Input Messages: Configure the receiver.
	UBX_CLASS_UPD = 0x09,	 //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
	UBX_CLASS_MON = 0x0A,	 //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
	UBX_CLASS_AID = 0x0B,	 //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
	UBX_CLASS_TIM = 0x0D,	 //Timing Messages: Time Pulse Output, Time Mark Results
	UBX_CLASS_ESF = 0x10,	 //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
	UBX_CLASS_MGA = 0x13,	 //Multiple GNSS Assistance Messages: Assistance data for various GNSS
	UBX_CLASS_LOG = 0x21,	 //Logging Messages: Log creation, deletion, info and retrieval
	UBX_CLASS_SEC = 0x27,	 //Security Feature Messages
	UBX_CLASS_HNR = 0x28,	 //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
	UBX_CLASS_NMEA = 0xF0, //NMEA Strings: standard NMEA strings
};


//The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
enum ubx_cfg {
	UBX_CFG_ANT = 0x13,		//Antenna Control Settings. Used to configure the antenna control settings
	UBX_CFG_BATCH = 0x93,		//Get/set data batching configuration.
	UBX_CFG_CFG = 0x09,		//Clear, Save, and Load Configurations. Used to save current configuration
	UBX_CFG_DAT = 0x06,		//Set User-defined Datum or The currently defined Datum
	UBX_CFG_DGNSS = 0x70,		//DGNSS configuration
	UBX_CFG_GEOFENCE = 0x69,	//Geofencing configuration. Used to configure a geofence
	UBX_CFG_GNSS = 0x3E,		//GNSS system configuration
	UBX_CFG_INF = 0x02,		//Depending on packet length, either: poll configuration for one protocol, or information message configuration
	UBX_CFG_ITFM = 0x39,		//Jamming/Interference Monitor configuration
	UBX_CFG_LOGFILTER = 0x47, //Data Logger Configuration
	UBX_CFG_MSG = 0x01,		//Poll a message configuration, or Set Message Rate(s), or Set Message Rate
	UBX_CFG_NAV5 = 0x24,		//Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
	UBX_CFG_NAVX5 = 0x23,		//Navigation Engine Expert Settings
	UBX_CFG_NMEA = 0x17,		//Extended NMEA protocol configuration V1
	UBX_CFG_ODO = 0x1E,		//Odometer, Low-speed COG Engine Settings
	UBX_CFG_PM2 = 0x3B,		//Extended power management configuration
	UBX_CFG_PMS = 0x86,		//Power mode setup
	UBX_CFG_PRT = 0x00,		//Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
	UBX_CFG_PWR = 0x57,		//Put receiver in a defined power state
	UBX_CFG_RATE = 0x08,		//Navigation/Measurement Rate Settings. Used to set port baud rates.
	UBX_CFG_RINV = 0x34,		//Contents of Remote Inventory
	UBX_CFG_RST = 0x04,		//Reset Receiver / Clear Backup Data Structures. Used to reset device.
	UBX_CFG_RXM = 0x11,		//RXM configuration
	UBX_CFG_SBAS = 0x16,		//SBAS configuration
	UBX_CFG_TMODE3 = 0x71,	//Time Mode Settings 3. Used to enable Survey In Mode
	UBX_CFG_TP5 = 0x31,		//Time Pulse Parameters
	UBX_CFG_USB = 0x1B,		//USB Configuration
	UBX_CFG_VALDEL = 0x8C,	//Used for config of higher version Ublox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
	UBX_CFG_VALGET = 0x8B,	//Used for config of higher version Ublox modules (ie protocol v27 and above). Configuration Items
	UBX_CFG_VALSET = 0x8A,	//Used for config of higher version Ublox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.
};

//The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
enum ubx_nmea {
	UBX_NMEA_MSB = 0xF0,	//All NMEA enable commands have 0xF0 as MSB
	UBX_NMEA_DTM = 0x0A,	//GxDTM (datum reference)
	UBX_NMEA_GAQ = 0x45,	//GxGAQ (poll a standard message (if the current talker ID is GA))
	UBX_NMEA_GBQ = 0x44,	//GxGBQ (poll a standard message (if the current Talker ID is GB))
	UBX_NMEA_GBS = 0x09,	//GxGBS (GNSS satellite fault detection)
	UBX_NMEA_GGA = 0x00,	//GxGGA (Global positioning system fix data)
	UBX_NMEA_GLL = 0x01,	//GxGLL (latitude and long, whith time of position fix and status)
	UBX_NMEA_GLQ = 0x43,	//GxGLQ (poll a standard message (if the current Talker ID is GL))
	UBX_NMEA_GNQ = 0x42,	//GxGNQ (poll a standard message (if the current Talker ID is GN))
	UBX_NMEA_GNS = 0x0D,	//GxGNS (GNSS fix data)
	UBX_NMEA_GPQ = 0x40,	//GxGPQ (poll a standard message (if the current Talker ID is GP))
	UBX_NMEA_GRS = 0x06,	//GxGRS (GNSS range residuals)
	UBX_NMEA_GSA = 0x02,	//GxGSA (GNSS DOP and Active satellites)
	UBX_NMEA_GST = 0x07,	//GxGST (GNSS Pseudo Range Error Statistics)
	UBX_NMEA_GSV = 0x03,	//GxGSV (GNSS satellites in view)
	UBX_NMEA_RMC = 0x04,	//GxRMC (Recommended minimum data)
	UBX_NMEA_TXT = 0x41,	//GxTXT (text transmission)
	UBX_NMEA_VLW = 0x0F,	//GxVLW (dual ground/water distance)
	UBX_NMEA_VTG = 0x05,	//GxVTG (course over ground and Ground speed)
	UBX_NMEA_ZDA = 0x08,	//GxZDA (Time and Date)
};

//The following are used to configure the NMEA protocol main talker ID and GSV talker ID
enum ubx_nmea_talker {
	UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00, //main talker ID is system dependent
	UBX_NMEA_MAINTALKERID_GP = 0x01,			  //main talker ID is GPS
	UBX_NMEA_MAINTALKERID_GL = 0x02,			  //main talker ID is GLONASS
	UBX_NMEA_MAINTALKERID_GN = 0x03,			  //main talker ID is combined receiver
	UBX_NMEA_MAINTALKERID_GA = 0x04,			  //main talker ID is Galileo
	UBX_NMEA_MAINTALKERID_GB = 0x05,			  //main talker ID is BeiDou
	UBX_NMEA_GSVTALKERID_GNSS = 0x00,			  //GNSS specific Talker ID (as defined by NMEA)
	UBX_NMEA_GSVTALKERID_MAIN = 0x01,			  //use the main Talker ID
};

//The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
enum ubx_inf {
	UBX_INF_CLASS = 0x04,		//All INF messages have 0x04 as the class
	UBX_INF_DEBUG = 0x04,		//ASCII output with debug contents
	UBX_INF_ERROR = 0x00,		//ASCII output with error contents
	UBX_INF_NOTICE = 0x02,		//ASCII output with informational contents
	UBX_INF_TEST = 0x03,		//ASCII output with test contents
	UBX_INF_WARNING = 0x01,		//ASCII output with warning contents
};

//The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
enum ubx_log {
	UBX_LOG_CREATE = 0x07,		   //Create Log File
	UBX_LOG_ERASE = 0x03,			   //Erase Logged Data
	UBX_LOG_FINDTIME = 0x0E,		   //Find index of a log entry based on a given time, or response to FINDTIME requested
	UBX_LOG_INFO = 0x08,			   //Poll for log information, or Log information
	UBX_LOG_RETRIEVEPOSEXTRA = 0x0F, //Odometer log entry
	UBX_LOG_RETRIEVEPOS = 0x0B,	   //Position fix log entry
	UBX_LOG_RETRIEVESTRING = 0x0D,   //Byte string log entry
	UBX_LOG_RETRIEVE = 0x09,		   //Request log data
	UBX_LOG_STRING = 0x04,		   //Store arbitrary string on on-board flash
};

//The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
enum ubx_mga {
	UBX_MGA_ACK_DATA0 = 0x60,		 //Multiple GNSS Acknowledge message
	UBX_MGA_BDS_EPH = 0x03,		 //BDS Ephemeris Assistance
	UBX_MGA_BDS_ALM = 0x03,		 //BDS Almanac Assistance
	UBX_MGA_BDS_HEALTH = 0x03,	 //BDS Health Assistance
	UBX_MGA_BDS_UTC = 0x03,		 //BDS UTC Assistance
	UBX_MGA_BDS_IONO = 0x03,		 //BDS Ionospheric Assistance
	UBX_MGA_DBD = 0x80,			 //Either: Poll the Navigation Database, or Navigation Database Dump Entry
	UBX_MGA_GAL_EPH = 0x02,		 //Galileo Ephemeris Assistance
	UBX_MGA_GAL_ALM = 0x02,		 //Galileo Almanac Assitance
	UBX_MGA_GAL_TIMOFFSET = 0x02,	 //Galileo GPS time offset assistance
	UBX_MGA_GAL_UTC = 0x02,		 //Galileo UTC Assistance
	UBX_MGA_GLO_EPH = 0x06,		 //GLONASS Ephemeris Assistance
	UBX_MGA_GLO_ALM = 0x06,		 //GLONASS Almanac Assistance
	UBX_MGA_GLO_TIMEOFFSET = 0x06, //GLONASS Auxiliary Time Offset Assistance
	UBX_MGA_GPS_EPH = 0x00,		 //GPS Ephemeris Assistance
	UBX_MGA_GPS_ALM = 0x00,		 //GPS Almanac Assistance
	UBX_MGA_GPS_HEALTH = 0x00,	 //GPS Health Assistance
	UBX_MGA_GPS_UTC = 0x00,		 //GPS UTC Assistance
	UBX_MGA_GPS_IONO = 0x00,		 //GPS Ionosphere Assistance
	UBX_MGA_INI_POS_XYZ = 0x40,	 //Initial Position Assistance
	UBX_MGA_INI_POS_LLH = 0x40,	 //Initial Position Assitance
	UBX_MGA_INI_TIME_UTC = 0x40,	 //Initial Time Assistance
	UBX_MGA_INI_TIME_GNSS = 0x40,	 //Initial Time Assistance
	UBX_MGA_INI_CLKD = 0x40,		 //Initial Clock Drift Assitance
	UBX_MGA_INI_FREQ = 0x40,		 //Initial Frequency Assistance
	UBX_MGA_INI_EOP = 0x40,		 //Earth Orientation Parameters Assistance
	UBX_MGA_QZSS_EPH = 0x05,		 //QZSS Ephemeris Assistance
	UBX_MGA_QZSS_ALM = 0x05,		 //QZSS Almanac Assistance
	UBX_MGA_QZAA_HEALTH = 0x05,	 //QZSS Health Assistance
};

//The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
enum ubx_mon {
	UBX_MON_COMMS = 0x36, //Comm port information
	UBX_MON_GNSS = 0x28,	//Information message major GNSS selection
	UBX_MON_HW2 = 0x0B,	//Extended Hardware Status
	UBX_MON_HW3 = 0x37,	//HW I/O pin information
	UBX_MON_HW = 0x09,	//Hardware Status
	UBX_MON_IO = 0x02,	//I/O Subsystem Status
	UBX_MON_MSGPP = 0x06, //Message Parse and Process Status
	UBX_MON_PATCH = 0x27, //Output information about installed patches
	UBX_MON_RF = 0x38,	//RF information
	UBX_MON_RXBUF = 0x07, //Receiver Buffer Status
	UBX_MON_RXR = 0x21,	//Receiver Status Information
	UBX_MON_TXBUF = 0x08, //Transmitter Buffer Status. Used for query tx buffer size/state.
	UBX_MON_VER = 0x04,	//Receiver/Software Version. Used for obtaining Protocol Version.
};

//The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
enum ubx_nav {
	UBX_NAV_ATT = 0x05,		//Vehicle "Attitude" Solution
	UBX_NAV_CLOCK = 0x22,		//Clock Solution
	UBX_NAV_DOP = 0x04,		//Dilution of precision
	UBX_NAV_EOE = 0x61,		//End of Epoch
	UBX_NAV_GEOFENCE = 0x39,	//Geofencing status. Used to poll the geofence status
	UBX_NAV_HPPOSECEF = 0x13, //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
	UBX_NAV_HPPOSLLH = 0x14,	//High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
	UBX_NAV_ODO = 0x09,		//Odometer Solution
	UBX_NAV_ORB = 0x34,		//GNSS Orbit Database Info
	UBX_NAV_POSECEF = 0x01,	//Position Solution in ECEF
	UBX_NAV_POSLLH = 0x02,	//Geodetic Position Solution
	UBX_NAV_PVT = 0x07,		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
	UBX_NAV_RELPOSNED = 0x3C, //Relative Positioning Information in NED frame
	UBX_NAV_RESETODO = 0x10,	//Reset odometer
	UBX_NAV_SAT = 0x35,		//Satellite Information
	UBX_NAV_SIG = 0x43,		//Signal Information
	UBX_NAV_STATUS = 0x03,	//Receiver Navigation Status
	UBX_NAV_SVIN = 0x3B,		//Survey-in data. Used for checking Survey In status
	UBX_NAV_TIMEBDS = 0x24,	//BDS Time Solution
	UBX_NAV_TIMEGAL = 0x25,	//Galileo Time Solution
	UBX_NAV_TIMEGLO = 0x23,	//GLO Time Solution
	UBX_NAV_TIMEGPS = 0x20,	//GPS Time Solution
	UBX_NAV_TIMELS = 0x26,	//Leap second event information
	UBX_NAV_TIMEUTC = 0x21,	//UTC Time Solution
	UBX_NAV_VELECEF = 0x11,	//Velocity Solution in ECEF
	UBX_NAV_VELNED = 0x12,	//Velocity Solution in NED
};

//The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
enum ubx_rxm {
	UBX_RXM_MEASX = 0x14, //Satellite Measurements for RRLP
	UBX_RXM_PMREQ = 0x41, //Requests a Power Management task (two differenent packet sizes)
	UBX_RXM_RAWX = 0x15,	//Multi-GNSS Raw Measurement Data
	UBX_RXM_RLM = 0x59,	//Galileo SAR Short-RLM report (two different packet sizes)
	UBX_RXM_RTCM = 0x32,	//RTCM input status
	UBX_RXM_SFRBX = 0x13, //Boradcast Navigation Data Subframe
};

//The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
enum ubx_sec {
	UBX_SEC_UNIQID = 0x03, //Unique chip ID
};

//The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
enum ubx_tim {
	UBX_TIM_TM2 = 0x03,  //Time mark data
	UBX_TIM_TP = 0x01,   //Time Pulse Timedata
	UBX_TIM_VRFY = 0x06, //Sourced Time Verification
};

//The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
enum ubx_upd {
	UBX_UPD_SOS = 0x14, //Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup
};

//The following are used to enable RTCM messages
enum ubx_rtcm {
	UBX_RTCM_MSB = 0xF5,	  //All RTCM enable commands have 0xF5 as MSB
	UBX_RTCM_1005 = 0x05,	  //Stationary RTK reference ARP
	UBX_RTCM_1074 = 0x4A,	  //GPS MSM4
	UBX_RTCM_1077 = 0x4D,	  //GPS MSM7
	UBX_RTCM_1084 = 0x54,	  //GLONASS MSM4
	UBX_RTCM_1087 = 0x57,	  //GLONASS MSM7
	UBX_RTCM_1094 = 0x5E,	  //Galileo MSM4
	UBX_RTCM_1097 = 0x61,	  //Galileo MSM7
	UBX_RTCM_1124 = 0x7C,	  //BeiDou MSM4
	UBX_RTCM_1127 = 0x7F,	  //BeiDou MSM7
	UBX_RTCM_1230 = 0xE6,	  //GLONASS code-phase biases, set to once every 10 seconds
	UBX_RTCM_4072_0 = 0xFE, //Reference station PVT (ublox proprietary RTCM message)
	UBX_RTCM_4072_1 = 0xFD, //Additional reference station information (ublox proprietary RTCM message)
};

enum ubx_ack {
	UBX_ACK_NACK = 0x00,
	UBX_ACK_ACK = 0x01,
	UBX_ACK_NONE = 0x02, //Not a real value
};

// The following constants are used to get External Sensor Measurements and Status
// Information.
enum ubx_esf {
	UBX_ESF_MEAS = 0x02,
	UBX_ESF_RAW = 0x03,
	UBX_ESF_STATUS = 0x10,
	UBX_ESF_INS = 0x15, //36 bytes
};

// ubxPacket validity
enum ublox_packet_validity {
	UBLOX_PACKET_VALIDITY_NOT_VALID,
	UBLOX_PACKET_VALIDITY_VALID,
	UBLOX_PACKET_VALIDITY_NOT_DEFINED,
	UBLOX_PACKET_NOTACKNOWLEDGED, // This indicates that we received a NACK
};

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
enum ublox_packet_buffer {
	UBLOX_PACKET_PACKETCFG,
	UBLOX_PACKET_PACKETACK,
	UBLOX_PACKET_PACKETBUF,
};


//Depending on the ubx binary response class, store binary responses into different places
// enum ubx_class {
// 	CLASS_NONE = 0,
// 	CLASS_ACK,
// 	CLASS_NOT_AN_ACK,
// };


//-=-=-=-=- UBX binary specific variables
struct ubx_packet {
	u8_t cls;
	u8_t id;
	u16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	u16_t counter;	   //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	u16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	u8_t *payload;
	u8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	u8_t checksumB;
	enum ublox_packet_validity valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	enum ublox_packet_validity classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

struct ubx_frame {
	enum ubx_header header[2];
	enum ubx_class class;
	u8_t id;
	u16_t len;
	u8_t *payload;
	u8_t checksumA;
	u8_t checksumB;
};

#endif /* __GNSS_UBLOX_UBX_H__ */
