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
//The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
enum ubx_class {
	ubx_class_nav = 0x01,	 //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
	ubx_class_rxm = 0x02,	 //Receiver Manager Messages: Satellite Status, RTC Status
	ubx_class_inf = 0x04,	 //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
	ubx_class_ack = 0x05,	 //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
	ubx_class_cfg = 0x06,	 //Configuration Input Messages: Configure the receiver.
	ubx_class_upd = 0x09,	 //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
	ubx_class_mon = 0x0A,	 //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
	ubx_class_aid = 0x0B,	 //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
	ubx_class_tim = 0x0D,	 //Timing Messages: Time Pulse Output, Time Mark Results
	ubx_class_esf = 0x10,	 //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
	ubx_class_mga = 0x13,	 //Multiple GNSS Assistance Messages: Assistance data for various GNSS
	ubx_class_log = 0x21,	 //Logging Messages: Log creation, deletion, info and retrieval
	ubx_class_sec = 0x27,	 //Security Feature Messages
	ubx_class_hnr = 0x28,	 //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
	ubx_class_nmea = 0xF0,	 //NMEA Strings: standard NMEA strings
} __packed;


//The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
enum ubx_cfg_id {
	ubx_cfg_id_ant = 0x13,		//Antenna Control Settings. Used to configure the antenna control settings
	ubx_cfg_id_batch = 0x93,		//Get/set data batching configuration.
	ubx_cfg_id_cfg = 0x09,		//Clear, Save, and Load Configurations. Used to save current configuration
	ubx_cfg_id_dat = 0x06,		//Set User-defined Datum or The currently defined Datum
	ubx_cfg_id_dgnss = 0x70,		//DGNSS configuration
	ubx_cfg_id_geofence = 0x69,	//Geofencing configuration. Used to configure a geofence
	ubx_cfg_id_gnss = 0x3E,		//GNSS system configuration
	ubx_cfg_id_inf = 0x02,		//Depending on packet length, either: poll configuration for one protocol, or information message configuration
	ubx_cfg_id_itfm = 0x39,		//Jamming/Interference Monitor configuration
	ubx_cfg_id_logfilter = 0x47, //Data Logger Configuration
	ubx_cfg_id_msg = 0x01,		//Poll a message configuration, or Set Message Rate(s), or Set Message Rate
	ubx_cfg_id_nav5 = 0x24,		//Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
	ubx_cfg_id_navx5 = 0x23,		//Navigation Engine Expert Settings
	ubx_cfg_id_nmea = 0x17,		//Extended NMEA protocol configuration V1
	ubx_cfg_id_odo = 0x1E,		//Odometer, Low-speed COG Engine Settings
	ubx_cfg_id_pm2 = 0x3B,		//Extended power management configuration
	ubx_cfg_id_pms = 0x86,		//Power mode setup
	ubx_cfg_id_prt = 0x00,		//Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
	ubx_cfg_id_pwr = 0x57,		//Put receiver in a defined power state
	ubx_cfg_id_rate = 0x08,		//Navigation/Measurement Rate Settings. Used to set port baud rates.
	ubx_cfg_id_rinv = 0x34,		//Contents of Remote Inventory
	ubx_cfg_id_rst = 0x04,		//Reset Receiver / Clear Backup Data Structures. Used to reset device.
	ubx_cfg_id_rxm = 0x11,		//RXM configuration
	ubx_cfg_id_sbas = 0x16,		//SBAS configuration
	ubx_cfg_id_tmode3 = 0x71,	//Time Mode Settings 3. Used to enable Survey In Mode
	ubx_cfg_id_tp5 = 0x31,		//Time Pulse Parameters
	ubx_cfg_id_usb = 0x1B,		//USB Configuration
	ubx_cfg_id_valdel = 0x8C,	//Used for config of higher version Ublox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
	ubx_cfg_id_valget = 0x8B,	//Used for config of higher version Ublox modules (ie protocol v27 and above). Configuration Items
	ubx_cfg_id_valset = 0x8A,	//Used for config of higher version Ublox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.
} __packed;

//The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
enum nmea_message {
	nmea_msb = 0xF0,	//All NMEA enable commands have 0xF0 as MSB
	nmea_dtm = 0x0A,	//GxDTM (datum reference)
	nmea_gaq = 0x45,	//GxGAQ (poll a standard message (if the current talker ID is GA))
	nmea_gbq = 0x44,	//GxGBQ (poll a standard message (if the current Talker ID is GB))
	nmea_gbs = 0x09,	//GxGBS (GNSS satellite fault detection)
	nmea_gga = 0x00,	//GxGGA (Global positioning system fix data)
	nmea_gll = 0x01,	//GxGLL (latitude and long, whith time of position fix and status)
	nmea_glq = 0x43,	//GxGLQ (poll a standard message (if the current Talker ID is GL))
	nmea_gnq = 0x42,	//GxGNQ (poll a standard message (if the current Talker ID is GN))
	nmea_gns = 0x0D,	//GxGNS (GNSS fix data)
	nmea_gpq = 0x40,	//GxGPQ (poll a standard message (if the current Talker ID is GP))
	nmea_grs = 0x06,	//GxGRS (GNSS range residuals)
	nmea_gsa = 0x02,	//GxGSA (GNSS DOP and Active satellites)
	nmea_gst = 0x07,	//GxGST (GNSS Pseudo Range Error Statistics)
	nmea_gsv = 0x03,	//GxGSV (GNSS satellites in view)
	nmea_rmc = 0x04,	//GxRMC (Recommended minimum data)
	nmea_txt = 0x41,	//GxTXT (text transmission)
	nmea_vlw = 0x0F,	//GxVLW (dual ground/water distance)
	nmea_vtg = 0x05,	//GxVTG (course over ground and Ground speed)
	nmea_zda = 0x08,	//GxZDA (Time and Date)
} __packed;

//The following are used to configure the NMEA protocol main talker ID and GSV talker ID
enum nmea_talker {
	nmea_maintalkerid_notoverridden = 0x00,		//main talker ID is system dependent
	nmea_maintalkerid_gp = 0x01,			//main talker ID is GPS
	nmea_maintalkerid_gl = 0x02,			//main talker ID is GLONASS
	nmea_maintalkerid_gn = 0x03,			//main talker ID is combined receiver
	nmea_maintalkerid_ga = 0x04,			//main talker ID is Galileo
	nmea_maintalkerid_gb = 0x05,			//main talker ID is BeiDou
	nmea_gsvtalkerid_gnss = 0x00,			//GNSS specific Talker ID (as defined by NMEA)
	nmea_gsvtalkerid_main = 0x01,			//use the main Talker ID
} __packed;

//The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
enum ubx_inf_id {
	ubx_inf_id_class = 0x04,		//All INF messages have 0x04 as the class
	ubx_inf_id_debug = 0x04,		//ASCII output with debug contents
	ubx_inf_id_error = 0x00,		//ASCII output with error contents
	ubx_inf_id_notice = 0x02,		//ASCII output with informational contents
	ubx_inf_id_test = 0x03,		//ASCII output with test contents
	ubx_inf_id_warning = 0x01,		//ASCII output with warning contents
} __packed;

//The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
enum ubx_log_id {
	ubx_log_id_create = 0x07,		   //Create Log File
	ubx_log_id_erase = 0x03,			   //Erase Logged Data
	ubx_log_id_findtime = 0x0E,		   //Find index of a log entry based on a given time, or response to FINDTIME requested
	ubx_log_id_info = 0x08,			   //Poll for log information, or Log information
	ubx_log_id_retrieveposextra = 0x0F, //Odometer log entry
	ubx_log_id_retrievepos = 0x0B,	   //Position fix log entry
	ubx_log_id_retrievestring = 0x0D,   //Byte string log entry
	ubx_log_id_retrieve = 0x09,		   //Request log data
	ubx_log_id_string = 0x04,		   //Store arbitrary string on on-board flash
} __packed;

//The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
enum ubx_mga_id {
	ubx_mga_id_ack_data0 = 0x60,		 //Multiple GNSS Acknowledge message
	ubx_mga_id_bds_eph = 0x03,		 //BDS Ephemeris Assistance
	ubx_mga_id_bds_alm = 0x03,		 //BDS Almanac Assistance
	ubx_mga_id_bds_health = 0x03,	 //BDS Health Assistance
	ubx_mga_id_bds_utc = 0x03,		 //BDS UTC Assistance
	ubx_mga_id_bds_iono = 0x03,		 //BDS Ionospheric Assistance
	ubx_mga_id_dbd = 0x80,			 //Either: Poll the Navigation Database, or Navigation Database Dump Entry
	ubx_mga_id_gal_eph = 0x02,		 //Galileo Ephemeris Assistance
	ubx_mga_id_gal_alm = 0x02,		 //Galileo Almanac Assitance
	ubx_mga_id_gal_timoffset = 0x02,	 //Galileo GPS time offset assistance
	ubx_mga_id_gal_utc = 0x02,		 //Galileo UTC Assistance
	ubx_mga_id_glo_eph = 0x06,		 //GLONASS Ephemeris Assistance
	ubx_mga_id_glo_alm = 0x06,		 //GLONASS Almanac Assistance
	ubx_mga_id_glo_timeoffset = 0x06, //GLONASS Auxiliary Time Offset Assistance
	ubx_mga_id_gps_eph = 0x00,		 //GPS Ephemeris Assistance
	ubx_mga_id_gps_alm = 0x00,		 //GPS Almanac Assistance
	ubx_mga_id_gps_health = 0x00,	 //GPS Health Assistance
	ubx_mga_id_gps_utc = 0x00,		 //GPS UTC Assistance
	ubx_mga_id_gps_iono = 0x00,		 //GPS Ionosphere Assistance
	ubx_mga_id_ini_pos_xyz = 0x40,	 //Initial Position Assistance
	ubx_mga_id_ini_pos_llh = 0x40,	 //Initial Position Assitance
	ubx_mga_id_ini_time_utc = 0x40,	 //Initial Time Assistance
	ubx_mga_id_ini_time_gnss = 0x40,	 //Initial Time Assistance
	ubx_mga_id_ini_clkd = 0x40,		 //Initial Clock Drift Assitance
	ubx_mga_id_ini_freq = 0x40,		 //Initial Frequency Assistance
	ubx_mga_id_ini_eop = 0x40,		 //Earth Orientation Parameters Assistance
	ubx_mga_id_qzss_eph = 0x05,		 //QZSS Ephemeris Assistance
	ubx_mga_id_qzss_alm = 0x05,		 //QZSS Almanac Assistance
	ubx_mga_id_qzaa_health = 0x05,	 //QZSS Health Assistance
} __packed;

//The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
enum ubx_mon_id {
	ubx_mon_id_comms = 0x36, //Comm port information
	ubx_mon_id_gnss = 0x28,	//Information message major GNSS selection
	ubx_mon_id_hw2 = 0x0B,	//Extended Hardware Status
	ubx_mon_id_hw3 = 0x37,	//HW I/O pin information
	ubx_mon_id_hw = 0x09,	//Hardware Status
	ubx_mon_id_io = 0x02,	//I/O Subsystem Status
	ubx_mon_id_msgpp = 0x06, //Message Parse and Process Status
	ubx_mon_id_patch = 0x27, //Output information about installed patches
	ubx_mon_id_rf = 0x38,	//RF information
	ubx_mon_id_rxbuf = 0x07, //Receiver Buffer Status
	ubx_mon_id_rxr = 0x21,	//Receiver Status Information
	ubx_mon_id_txbuf = 0x08, //Transmitter Buffer Status. Used for query tx buffer size/state.
	ubx_mon_id_ver = 0x04,	//Receiver/Software Version. Used for obtaining Protocol Version.
} __packed;

//The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
enum ubx_nav_id {
	ubx_nav_id_att = 0x05,		//Vehicle "Attitude" Solution
	ubx_nav_id_clock = 0x22,		//Clock Solution
	ubx_nav_id_dop = 0x04,		//Dilution of precision
	ubx_nav_id_eoe = 0x61,		//End of Epoch
	ubx_nav_id_geofence = 0x39,	//Geofencing status. Used to poll the geofence status
	ubx_nav_id_hpposecef = 0x13, //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
	ubx_nav_id_hpposllh = 0x14,	//High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
	ubx_nav_id_odo = 0x09,		//Odometer Solution
	ubx_nav_id_orb = 0x34,		//GNSS Orbit Database Info
	ubx_nav_id_posecef = 0x01,	//Position Solution in ECEF
	ubx_nav_id_posllh = 0x02,	//Geodetic Position Solution
	ubx_nav_id_pvt = 0x07,		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
	ubx_nav_id_relposned = 0x3C, //Relative Positioning Information in NED frame
	ubx_nav_id_resetodo = 0x10,	//Reset odometer
	ubx_nav_id_sat = 0x35,		//Satellite Information
	ubx_nav_id_sig = 0x43,		//Signal Information
	ubx_nav_id_status = 0x03,	//Receiver Navigation Status
	ubx_nav_id_svin = 0x3B,		//Survey-in data. Used for checking Survey In status
	ubx_nav_id_timebds = 0x24,	//BDS Time Solution
	ubx_nav_id_timegal = 0x25,	//Galileo Time Solution
	ubx_nav_id_timeglo = 0x23,	//GLO Time Solution
	ubx_nav_id_timegps = 0x20,	//GPS Time Solution
	ubx_nav_id_timels = 0x26,	//Leap second event information
	ubx_nav_id_timeutc = 0x21,	//UTC Time Solution
	ubx_nav_id_velecef = 0x11,	//Velocity Solution in ECEF
	ubx_nav_id_velned = 0x12,	//Velocity Solution in NED
} __packed;

//The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
enum ubx_rxm_id {
	ubx_rxm_id_measx = 0x14, //Satellite Measurements for RRLP
	ubx_rxm_id_pmreq = 0x41, //Requests a Power Management task (two differenent packet sizes)
	ubx_rxm_id_rawx = 0x15,	//Multi-GNSS Raw Measurement Data
	ubx_rxm_id_rlm = 0x59,	//Galileo SAR Short-RLM report (two different packet sizes)
	ubx_rxm_id_rtcm = 0x32,	//RTCM input status
	ubx_rxm_id_sfrbx = 0x13, //Boradcast Navigation Data Subframe
} __packed;

//The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
enum ubx_sec_id {
	ubx_sec_id_uniqid = 0x03, //Unique chip ID
} __packed;

//The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
enum ubx_tim_id {
	ubx_tim_id_tm2 = 0x03,  //Time mark data
	ubx_tim_id_tp = 0x01,   //Time Pulse Timedata
	ubx_tim_id_vrfy = 0x06, //Sourced Time Verification
};

//The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
enum ubx_upd_id {
	ubx_upd_id_sos = 0x14, //Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup
} __packed;

//The following are used to enable RTCM messages
enum rtcm {
	rtcm_msb = 0xF5,	  //All RTCM enable commands have 0xF5 as MSB
	rtcm_1005 = 0x05,	  //Stationary RTK reference ARP
	rtcm_1074 = 0x4A,	  //GPS MSM4
	rtcm_1077 = 0x4D,	  //GPS MSM7
	rtcm_1084 = 0x54,	  //GLONASS MSM4
	rtcm_1087 = 0x57,	  //GLONASS MSM7
	rtcm_1094 = 0x5E,	  //Galileo MSM4
	rtcm_1097 = 0x61,	  //Galileo MSM7
	rtcm_1124 = 0x7C,	  //BeiDou MSM4
	rtcm_1127 = 0x7F,	  //BeiDou MSM7
	rtcm_1230 = 0xE6,	  //GLONASS code-phase biases, set to once every 10 seconds
	rtcm_4072_0 = 0xFE, //Reference station PVT (ublox proprietary RTCM message)
	rtcm_4072_1 = 0xFD, //Additional reference station information (ublox proprietary RTCM message)
} __packed;

enum ubx_ack_id {
	ubx_ack_id_nak = 0x00,
	ubx_ack_id_ack = 0x01,
	ubx_ack_id_none = 0x02, //Not a real value
} __packed;

// The following constants are used to get External Sensor Measurements and Status
// Information.
enum ubx_esf_id {
	ubx_esf_id_meas = 0x02,
	ubx_esf_id_raw = 0x03,
	ubx_esf_id_status = 0x10,
	ubx_esf_id_ins = 0x15, //36 bytes
} __packed;

enum ubx_sync_id {
	ubx_sync_1 = 0xB5,
	ubx_sync_2 = 0x62,
} __packed;

enum ubx_frame_state {
	UBX_FRAME_STATE_UNINITIALIZED,
	UBX_FRAME_STATE_INITIALIZED,
	UBX_FRAME_STATE_SYNC,
	UBX_FRAME_STATE_CLASS,
	UBX_FRAME_STATE_ID,
	UBX_FRAME_STATE_LEN,
	UBX_FRAME_STATE_PAYLOAD,
	UBX_FRAME_STATE_CHECKSUM,
	UBX_FRAME_STATE_ACK,
	UBX_FRAME_STATE_NAK,
	UBX_FRAME_STATE_CHECKSUM_ERROR,
	UBX_FRAME_STATE_TIMEOUT,
} __packed;

enum ubx_response {
	UBX_RESPONSE_NONE,
	UBX_RESPONSE_NAK,
	UBX_RESPONSE_ACK,
	UBX_RESPONSE_POLL,
	UBX_RESPONSE_GET,
	UBX_RESPONSE_OUTPUT,
	UBX_RESPONSE_PERIODIC,
} __packed;

enum ubx_message {
	UBX_MESSAGE_COMMAND,
	UBX_MESSAGE_POLL,
	UBX_MESSAGE_GET,
	UBX_MESSAGE_SET,
	UBX_MESSAGE_PERIODIC,
	UBX_MESSAGE_INPUT,
	UBX_MESSAGE_OUTPUT,
} __packed;

union ubx_cfg_prt_txready {
	struct {
		u16_t en : 1;
		u16_t pol : 1;
		u16_t pin : 5;
		u16_t thres : 9;
	} bit;
	u16_t reg;
};

union ubx_cfg_prt_output_protocol {
	struct {
		u16_t ubx : 1;
		u16_t nmea : 1;
		u16_t reserved : 3;
		u16_t rtcm3 : 1;
	} bit;
	u16_t reg;
};

union ubx_cfg_prt_input_protocol {
	struct {
		u16_t ubx : 1;
		u16_t nmea : 1;
		u16_t rtcm : 1;
		u16_t reserved : 2;
		u16_t rtcm3 : 1;
	} bit;
	u16_t reg;
};

struct ubx_header {
	enum ubx_sync_id sync1;
	enum ubx_sync_id sync2;
	enum ubx_class class;
	u8_t id;
	u8_t len[2];
} __packed;

struct ubx_checksum {
	u8_t A;
	u8_t B;
} __packed;

// ubx frame status
struct ubx_frame_status {
	enum ubx_response response_request;
	enum ubx_response response_received;
	bool checksum_valid;
	bool error;
};

//-=-=-=-=- UBX binary specific variables
struct ubx_frame_req {
	struct ubx_header header;
	struct ubx_checksum checksum;
};

struct ubx_frame {
	struct ubx_header header;
	struct ubx_checksum checksum;
	const u8_t *payload;
	u16_t len;
	struct ubx_frame_status status;
	enum ubx_frame_state state;
	enum ubx_message type;
};

struct ubx_payload_cfg_cfg {
	struct {
		u32_t ioPort : 1;
		u32_t msgConf : 1;
		u32_t infMsg : 1;
		u32_t navConf : 1;
		u32_t rxmConf : 1;
		u32_t reserved : 3;
		u32_t senConf : 1;
		u32_t rinvConf : 1;
		u32_t antConf : 1;
		u32_t logConf : 1;
		u32_t ftsConf : 1;
	} clearMask;
	struct {
		u32_t ioPort : 1;
		u32_t msgConf : 1;
		u32_t infMsg : 1;
		u32_t navConf : 1;
		u32_t rxmConf : 1;
		u32_t reserved : 3;
		u32_t senConf : 1;
		u32_t rinvConf : 1;
		u32_t antConf : 1;
		u32_t logConf : 1;
		u32_t ftsConf : 1;
	} saveMask;
	struct {
		u32_t ioPort : 1;
		u32_t msgConf : 1;
		u32_t infMsg : 1;
		u32_t navConf : 1;
		u32_t rxmConf : 1;
		u32_t reserved : 3;
		u32_t senConf : 1;
		u32_t rinvConf : 1;
		u32_t antConf : 1;
		u32_t logConf : 1;
		u32_t ftsConf : 1;
	} loadMask;
	struct {
		u8_t devBBR : 1;
		u8_t devFlash : 1;
		u8_t devEEPROM : 1;
		u8_t reserved : 1;
		u8_t devSpiFlash : 1;
	} deviceMask;
} __packed;

struct ubx_payload_cfg_pm2 {
	u8_t version;
	u8_t reserved1;
	u8_t maxStartupStateDur;	// s, Maximum time to spend in Acquisition state
	u8_t reserved2;
	struct {
		u32_t reserved1 : 1;
		u32_t optTarget : 3;
		u32_t extintSel : 1;
		u32_t extintWake : 1;
		u32_t extintBackup : 1;
		u32_t extintInactive : 1;
		u32_t limitPeakCurr : 2;
		u32_t waitTimeFix : 1;
		u32_t updateRTC : 1;
		u32_t updateEPH : 1;
		u32_t reserved2 : 3;
		u32_t doNotEnterOff : 1;
		u32_t mode : 2;
	} flags;			// PSM configuration flags
	u32_t updatePeriod;		// ms, position update period
	u32_t searchPeriod;		// ms, acquisition retry period if previously UBX_FRAME_STATE_INITIALIZED
	u32_t gridOffset;		// ms, grid offset relative to GPS start of timeOfWeek
	u16_t onTime;			// s, Time to stay i Tracking state
	u16_t minAcqTime;		// s, Minimal search time
	u8_t reserved3[20];
	u32_t extintInactivityMs;	// ms, inactivity time out on EXTINT pin if enabled
} __packed;

struct ubx_payload_cfg_pwr {
	u8_t version;
	u8_t reserved1[3];
	u32_t state;
} __packed;

union ubx_payload_cfg_prt {
	struct {
		u8_t portID;
		u8_t reserved1;
		struct {
			u16_t en : 1;
			u16_t pol : 1;
			u16_t pin : 5;
			u16_t thres : 9;
		} txReady;			// txReady pin configuration
		struct {
			u32_t reserved1 : 1;
			u32_t slaveAddr : 7;
		} mode;
		u8_t reserved2[4];
		struct {
			u16_t inUbx : 1;
			u16_t inNmea : 1;
			u16_t inRtcm : 1;
			u16_t reserved1 : 2;
			u16_t inRtcm3 : 1;
		} inProtoMask;
		struct {
			u16_t outUbx : 1;
			u16_t outNmea : 1;
			u16_t reserved1 : 3;
			u16_t inRtcm3 : 1;
		} outProtoMask;
		struct {
			u16_t reserved1 : 1;
			u16_t extendedTxTimeout : 1;
		} flags;
		u8_t reserved3;
	} ddc;
	struct {
		u8_t portID;
		u8_t reserved1;
		struct {
			u16_t en : 1;
			u16_t pol : 1;
			u16_t pin : 5;
			u16_t thres : 9;
		} txReady;			// txReady pin configuration
		struct {
			u32_t reserved1 : 6;
			u32_t charLen : 2;
			u32_t reserved2 : 1;
			u32_t parity : 3;
			u32_t nStopBits : 2;
		} mode;
		u32_t baudRate;			// Bits/s
		struct {
			u16_t inUbx : 1;
			u16_t inNmea : 1;
			u16_t inRtcm : 1;
			u16_t reserved1 : 2;
			u16_t inRtcm3 : 1;
		} inProtoMask;
		struct {
			u16_t outUbx : 1;
			u16_t outNmea : 1;
			u16_t reserved1 : 3;
			u16_t inRtcm3 : 1;
		} outProtoMask;
		struct {
			u16_t reserved1 : 1;
			u16_t extendedTxTimeout : 1;
		} flags;
		u8_t reserved2[2];
	} uart;
} __packed;

struct ubx_payload_cfg_rate {
	u16_t measRate;			// ms, elapsed time between GNSS measurements.  Should be greater than 50ms
	u16_t navRate;			// cycles, ratio between the number of measurements and teh number of navigation solutions
	u16_t timeReg;			// the time system to which measurements are aligned, 0=UTC, 1=GPS, 2=GLOSNASS, 3=BeiDou, 4=Galileo
} __packed;

struct ubx_payload_cfg_rxm {
	u8_t reserved1;
	u8_t lpMode;			// Low power mode
} __packed;

struct ubx_payload_cfg_tp5 {
	u8_t tpIdx;			// Time pulse selection (0 = TIMPULSE, 1 = TIMEPULSE2)
	u8_t version;
	u8_t reserved1[2];
	s16_t antCableDelay;		// ns
	s16_t rfGroupDelay;		// ns
	u32_t freqPeriod;		// Frequency or period time, depending on 'isFreq' bit
	u32_t freqPeriodLock;		// Frequency or period time when locked to GPS time
	u32_t pulseLenRatio;		// Pulse length or duty cycle, depending on 'isLength'
	u32_t pulseLenRatioLock;	// Pulse length or duty cycle when locked to GPS time
	s32_t userConfigDelay;		// ns, User configurable time pulse delay
	struct {
		u32_t active : 1;
		u32_t lockGpsFreq : 1;
		u32_t lockedOtherSet : 1;
		u32_t isFreq : 1;
		u32_t isLength : 1;
		u32_t alignToTow : 1;
		u32_t polarity : 1;
		u32_t gridUtcGps : 1;
	} flags;			// Configuration flags
} __packed;

struct ubx_payload_nav_pvt {
	/* time */
	u32_t timeOfWeek;		// ms
	u16_t gpsYear;
	u8_t gpsMonth;
	u8_t gpsDay;
	u8_t gpsHour;
	u8_t gpsMinute;
	u8_t gpsSecond;
	u8_t valid;
	u32_t accuracy;
	s32_t gpsNanosecond;
	/* position */
	u8_t fixType;			// Tells us when we have a solution aka lock
	u8_t fixflags;
	u8_t flags2;
	u8_t SIV;			// Number of satellites used in position solution
	s32_t longitude;		// Degrees * 10^-7 (more accurate than floats)
	s32_t latitude;			// Degrees * 10^-7 (more accurate than floats)
	s32_t altitude;			// Number of mm above ellipsoid
	s32_t altitudeMSL;		// Number of mm above Mean Sea Level
	u32_t horizontalAccuracy;	// mm * 10^-1 (i.e. 0.1mm)
	u32_t verticalAccuracy;		// mm * 10^-1 (i.e. 0.1mm)
	/* velocity */
	s32_t north;			// mm/s
	s32_t east;			// mm/s
	s32_t down;			// mm/s
	s32_t ground_speed;		// mm/s
	s32_t heading_of_motion;	// degrees * 10^-5
	u32_t speed_accuracy;
	u32_t heading_accuracy;		// degrees * 10^-5
	u16_t pDOP;			// Position dilution of precision 10^-2
	u8_t flags3;
	u8_t reserved1[5];
	s32_t headVeh;			// degrees * 10^-5
	s16_t magDec;			// degrees * 10^-2
	u16_t magAcc;			// degrees * 10^-2
} __packed;

struct ubx_payload_rxm_pmreq {
	u8_t version;			// message version
	u8_t reserved1[3];
	u32_t duration;			// duration of the requested task
	struct {
		u32_t reserved : 1;
		u32_t backup : 1;
		u32_t force : 1;
	} flags;			// task flags
	struct {
		u32_t reserved1 : 3;
		u32_t uartrx : 1;
		u32_t reserved2 : 1;
		u32_t extint0 : 1;
		u32_t extint1 : 1;
		u32_t spics : 1;
	} wakeupSources;		// rising or falling edge wakes the receiver
} __packed;

struct ubx_payload_sec_uniqid {
	u8_t version;			// message version
	u8_t reserved1[3];
	u8_t uniqueId[5];		// unique id,
} __packed;

#define UBX_FRAME_SYNC_SIZE 2
#define UBX_FRAME_HEADER_SIZE sizeof(struct ubx_header)
#define UBX_FRAME_CHECKSUM_SIZE sizeof(struct ubx_checksum)
#define UBX_FRAME_REQ_SIZE sizeof(struct ubx_frame_req)
#define UBX_FRAME_STATUS_SIZE sizeof(struct ubx_frame_status)
#define UBX_FRAME_SIZE(n) (UBX_FRAME_REQ_SIZE + sizeof(n))

#ifndef UBX_MAX_PAYLOAD_SIZE
#define UBX_MAX_PAYLOAD_SIZE 164 // Maximum payload size, see 32.15.4.2
#endif

#define UBX_MAX_FRAME_SIZE	(UBX_FRAME_REQ_SIZE + UBX_MAX_PAYLOAD_SIZE)

#define UBX_SYNC {ubx_sync_1, ubx_sync_2}

#define UBX_HEADER_DEFINE(cls,clsid)					\
	const struct ubx_header ubx_header_##cls##_##clsid = {		\
		.sync1 = ubx_sync_1,					\
		.sync2 = ubx_sync_2,					\
		.class = ubx_class_##cls,				\
		.id = ubx_##cls##_id_##clsid,				\
		.len = {0, 0},						\
	}

#define UBX_FRAME_PAYLOAD_DEFINE(cls,clsid)				\
	const struct {							\
	}

#define UBX_FRAME_DEFINE(_name, _payload)				\
	static struct ubx_frame ##_name = {				\
		.header = {},						\
		.checksum = {},						\
		.payload = _payload,					\
		.len = 0,						\
		.status = {},						\
		.state = UBX_FRAME_STATE_UNINITIALIZED,			\
	}

#define UBX_HEADER_NAV_DEFINE(n) UBX_HEADER_DEFINE(nav,n);
#define UBX_HEADER_RXM_DEFINE(n) UBX_HEADER_DEFINE(rxm,n);
#define UBX_HEADER_INF_DEFINE(n) UBX_HEADER_DEFINE(inf,n);
#define UBX_HEADER_ACK_DEFINE(n) UBX_HEADER_DEFINE(ack,n);
#define UBX_HEADER_CFG_DEFINE(n) UBX_HEADER_DEFINE(cfg,n);
#define UBX_HEADER_UPD_DEFINE(n) UBX_HEADER_DEFINE(upd,n);
#define UBX_HEADER_MON_DEFINE(n) UBX_HEADER_DEFINE(mon,n);
#define UBX_HEADER_AID_DEFINE(n) UBX_HEADER_DEFINE(aid,n);
#define UBX_HEADER_TIM_DEFINE(n) UBX_HEADER_DEFINE(tim,n);
#define UBX_HEADER_ESF_DEFINE(n) UBX_HEADER_DEFINE(esf,n);
#define UBX_HEADER_MGA_DEFINE(n) UBX_HEADER_DEFINE(mga,n);
#define UBX_HEADER_LOG_DEFINE(n) UBX_HEADER_DEFINE(log,n);
#define UBX_HEADER_SEC_DEFINE(n) UBX_HEADER_DEFINE(sec,n);
#define UBX_HEADER_HNR_DEFINE(n) UBX_HEADER_DEFINE(hnr,n);

/* ubx payload sizes */
#define UBX_PAYLOAD_CFG_CFG_SIZE	sizeof(struct ubx_payload_cfg_cfg)
#define UBX_PAYLOAD_CFG_PM2_SIZE	sizeof(struct ubx_payload_cfg_pm2)
#define UBX_PAYLOAD_CFG_PRT_SIZE	sizeof(union ubx_payload_cfg_prt)
#define UBX_PAYLOAD_CFG_PWR_SIZE	sizeof(struct ubx_payload_cfg_pwr)
#define UBX_PAYLOAD_CFG_RXM_SIZE	sizeof(struct ubx_payload_cfg_rxm)
#define UBX_PAYLOAD_CFG_TP5_SIZE	sizeof(struct ubx_payload_cfg_tp5)
#define UBX_PAYLOAD_NAV_PVT_SIZE	sizeof(struct ubx_payload_nav_pvt)
#define UBX_PAYLOAD_RXM_PMREQ_SIZE	sizeof(struct ubx_payload_rxm_pmreq)
#define UBX_PAYLOAD_SEC_UNIQID_SIZE	sizeof(struct ubx_payload_sec_uniqid)

#endif /* __GNSS_UBLOX_UBX_H__ */
