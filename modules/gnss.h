
#ifndef GNSS_H_
#define GNSS_H_

#include "fsl_dspi.h"
#include "pin_mux.h"
#include "peripherals.h"


#ifndef DEF_NUM_SENS
#define DEF_NUM_SENS 7 // The maximum number of ESF sensors
#endif

// For storing SPI bytes received during sendSpiCommand
#define SFE_UBLOX_SPI_BUFFER_SIZE 128

// UBX-HNR-ATT (0x28 0x01): Attitude solution
const uint16_t UBX_HNR_ATT_LEN = 32;

// UBX-HNR-PVT (0x28 0x00): High rate output of PVT solution
const uint16_t UBX_HNR_PVT_LEN = 72;

// UBX-HNR-INS (0x28 0x02): Vehicle dynamics information
const uint16_t UBX_HNR_INS_LEN = 36;

// UBX-MGA-ACK-DATA0 (0x13 0x60): Multiple GNSS acknowledge message
const uint16_t UBX_MGA_ACK_DATA0_LEN = 8;

const uint16_t UBX_ESF_STATUS_MAX_LEN = 16 + (4 * DEF_NUM_SENS);

// UBX-NAV-STATUS (0x01 0x03): Receiver navigation status
const uint16_t UBX_NAV_STATUS_LEN = 16;

// UBX-NAV-DOP (0x01 0x04): Dilution of precision
const uint16_t UBX_NAV_DOP_LEN = 18;

// UBX-NAV-ATT (0x01 0x05): Attitude solution
const uint16_t UBX_NAV_ATT_LEN = 32;

// UBX-NAV-POSECEF (0x01 0x01): Position solution in ECEF
const uint16_t UBX_NAV_POSECEF_LEN = 20;

// UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
const uint16_t UBX_NAV_PVT_LEN = 92;

const uint16_t UBX_NAV_ODO_LEN = 20;

// UBX-NAV-VELECEF (0x01 0x11): Velocity solution in ECEF
const uint16_t UBX_NAV_VELECEF_LEN = 20;

// UBX-NAV-VELNED (0x01 0x12): Velocity solution in NED frame
const uint16_t UBX_NAV_VELNED_LEN = 36;

const uint16_t UBX_NAV_HPPOSECEF_LEN = 28;

// UBX-NAV-HPPOSLLH (0x01 0x14): High precision geodetic position solution
const uint16_t UBX_NAV_HPPOSLLH_LEN = 36;

// UBX-NAV-PVAT (0x01 0x17): Navigation position velocity attitude time solution
const uint16_t UBX_NAV_PVAT_LEN = 116;

// UBX-NAV-CLOCK (0x01 0x22): Clock solution
const uint16_t UBX_NAV_CLOCK_LEN = 20;

const uint16_t UBX_NAV_TIMELS_LEN = 24;

// UBX-NAV-SVIN (0x01 0x3B): Survey-in data
const uint16_t UBX_NAV_SVIN_LEN = 40;

const uint16_t UBX_NAV_SAT_MAX_BLOCKS = 255; // numSvs is 8-bit
const uint16_t UBX_NAV_SAT_MAX_LEN = 8 + (12 * UBX_NAV_SAT_MAX_BLOCKS);

const uint16_t UBX_NAV_RELPOSNED_LEN_F9 = 64;
const uint16_t UBX_NAV_AOPSTATUS_LEN = 16;

// Global Status Returns
typedef enum
{
  SFE_UBLOX_STATUS_SUCCESS,
  SFE_UBLOX_STATUS_FAIL,
  SFE_UBLOX_STATUS_CRC_FAIL,
  SFE_UBLOX_STATUS_TIMEOUT,
  SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
  SFE_UBLOX_STATUS_OUT_OF_RANGE,
  SFE_UBLOX_STATUS_INVALID_ARG,
  SFE_UBLOX_STATUS_INVALID_OPERATION,
  SFE_UBLOX_STATUS_MEM_ERR,
  SFE_UBLOX_STATUS_HW_ERR,
  SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
  SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
  SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
  SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;


// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
// packetAuto is used to store expected "automatic" messages
typedef enum
{
  SFE_UBLOX_PACKET_PACKETCFG,
  SFE_UBLOX_PACKET_PACKETACK,
  SFE_UBLOX_PACKET_PACKETBUF,
  SFE_UBLOX_PACKET_PACKETAUTO
} sfe_ublox_packet_buffer_e;


// ubxPacket validity
typedef enum
{
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
  SFE_UBLOX_PACKET_VALIDITY_VALID,
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
  SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

typedef struct
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;
  sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
}ubxPacket;

// Additional flags and pointers that need to be stored with each message type
typedef struct
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t automatic : 1;         // Will this message be delivered and parsed "automatically" (without polling)
      uint8_t implicitUpdate : 1;    // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
      uint8_t addToFileBuffer : 1;   // Should the raw UBX data be added to the file buffer?
      uint8_t callbackCopyValid : 1; // Is the copy of the data struct used by the callback valid/fresh?
    } bits;
  } flags;
}ubxAutomaticFlags;


typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t year : 1;
      uint32_t month : 1;
      uint32_t day : 1;
      uint32_t hour : 1;
      uint32_t min : 1;
      uint32_t sec : 1;

      uint32_t validDate : 1;
      uint32_t validTime : 1;
      uint32_t fullyResolved : 1;
      uint32_t validMag : 1;

      uint32_t tAcc : 1;
      uint32_t nano : 1;
      uint32_t fixType : 1;
      uint32_t gnssFixOK : 1;
      uint32_t diffSoln : 1;
      uint32_t psmState : 1;
      uint32_t headVehValid : 1;
      uint32_t carrSoln : 1;

      uint32_t confirmedAvai : 1;
      uint32_t confirmedDate : 1;
      uint32_t confirmedTime : 1;

      uint32_t numSV : 1;
      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
      uint32_t velN : 1;
      uint32_t velE : 1;
    } bits;
  } moduleQueried1;
  union
  {
    uint32_t all;
    struct
    {
      uint32_t velD : 1;
      uint32_t gSpeed : 1;
      uint32_t headMot : 1;
      uint32_t sAcc : 1;
      uint32_t headAcc : 1;
      uint32_t pDOP : 1;

      uint32_t invalidLlh : 1;

      uint32_t headVeh : 1;
      uint32_t magDec : 1;
      uint32_t magAcc : 1;
    } bits;
  } moduleQueried2;
} UBX_NAV_PVT_moduleQueried_t;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day;   // Day of month, range 1..31 (UTC)
  uint8_t hour;  // Hour of day, range 0..23 (UTC)
  uint8_t min;   // Minute of hour, range 0..59 (UTC)
  uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validDate : 1;     // 1 = valid UTC Date
      uint8_t validTime : 1;     // 1 = valid UTC time of day
      uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
      uint8_t validMag : 1;      // 1 = valid magnetic declination
    } bits;
  } valid;
  uint32_t tAcc;   // Time accuracy estimate (UTC): ns
  int32_t nano;    // Fraction of second, range -1e9 .. 1e9 (UTC): ns
  uint8_t fixType; // GNSSfix Type:
                   // 0: no fix
                   // 1: dead reckoning only
                   // 2: 2D-fix
                   // 3: 3D-fix
                   // 4: GNSS + dead reckoning combined
                   // 5: time only fix
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
      uint8_t diffSoln : 1;  // 1 = differential corrections were applied
      uint8_t psmState : 3;
      uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t carrSoln : 2;     // Carrier phase range solution status:
                                // 0: no carrier phase range solution
                                // 1: carrier phase range solution with floating ambiguities
                                // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t reserved : 5;
      uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
      uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
      uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
    } bits;
  } flags2;
  uint8_t numSV;    // Number of satellites used in Nav Solution
  int32_t lon;      // Longitude: deg * 1e-7
  int32_t lat;      // Latitude: deg * 1e-7
  int32_t height;   // Height above ellipsoid: mm
  int32_t hMSL;     // Height above mean sea level: mm
  uint32_t hAcc;    // Horizontal accuracy estimate: mm
  uint32_t vAcc;    // Vertical accuracy estimate: mm
  int32_t velN;     // NED north velocity: mm/s
  int32_t velE;     // NED east velocity: mm/s
  int32_t velD;     // NED down velocity: mm/s
  int32_t gSpeed;   // Ground Speed (2-D): mm/s
  int32_t headMot;  // Heading of motion (2-D): deg * 1e-5
  uint32_t sAcc;    // Speed accuracy estimate: mm/s
  uint32_t headAcc; // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
  uint16_t pDOP;    // Position DOP * 0.01
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height and hMSL
    } bits;
  } flags3;
  uint8_t reserved1[5];
  int32_t headVeh; // Heading of vehicle (2-D): deg * 1e-5
  int16_t magDec;  // Magnetic declination: deg * 1e-2
  uint16_t magAcc; // Magnetic declination accuracy: deg * 1e-2
} UBX_NAV_PVT_data_t;



typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_PVT_data_t data;
  UBX_NAV_PVT_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_PVT_data_t);
  void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *);
  UBX_NAV_PVT_data_t *callbackData;
} UBX_NAV_PVT_t;

UBX_NAV_PVT_t *packetUBXNAVPVT = NULL;

enum commTypes
{
  COMM_TYPE_I2C = 0,
  COMM_TYPE_SERIAL,
  COMM_TYPE_SPI
} commType = COMM_TYPE_I2C; // Controls which port we look to for incoming bytes



int32_t getLatitude(uint16_t maxWait);
bool getPVT(uint16_t maxWait, ubxPacket *packetCfg);
bool initPacketUBXNAVPVT();
bool checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);
bool checkUbloxSpi(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);
sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly);
bool checkAutomatic(uint8_t Class, uint8_t ID);
uint16_t getMaxPayloadSize(uint8_t Class, uint8_t ID);
void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);
void calcChecksum(ubxPacket *msg);
void sendSpiCommand(ubxPacket *outgoingUBX);
//sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait);   // Poll the module until a config packet and an ACK is received, or just an ACK
sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime);   // Poll the module until a config packet and an ACK is received, or just an ACK

//sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait); // Poll the module until a config packet is received
sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime); // Poll the module until a config packet is received


/////////////////// move to main
//// The packet buffers
//  // These are pointed at from within the ubxPacket
//  uint8_t payloadAck[2];           // Holds the requested ACK/NACK
//  uint8_t payloadBuf[2];           // Temporary buffer used to screen incoming packets or dump unrequested packets
//  size_t packetCfgPayloadSize = 0; // Size for the packetCfg payload. .begin will set this to MAX_PAYLOAD_SIZE if necessary. User can change with setPacketCfgPayloadSize
//  uint8_t *payloadCfg = NULL;
//  uint8_t *payloadAuto = NULL;
uint8_t *spiBuffer = NULL;                              // A buffer to store any bytes being recieved back from the device while we are sending via SPI
uint8_t spiBufferIndex = 0;                             // Index into the SPI buffer
uint8_t spiTransactionSize = SFE_UBLOX_SPI_BUFFER_SIZE; // Default size of the SPI buffer
int8_t nmeaByteCounter = 0;
//// Init the packet structures and init them with pointers to the payloadAck, payloadCfg, payloadBuf and payloadAuto arrays
//ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
//ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
//ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
//ubxPacket packetAuto = {0, 0, 0, 0, 0, payloadAuto, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
/////////////////// move to main


// Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

// The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const int UBX_CLASS_NAV = 0x01;  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_RXM = 0x02;  // Receiver Manager Messages: Satellite Status, RTC Status
const uint8_t UBX_CLASS_INF = 0x04;  // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const uint8_t UBX_CLASS_ACK = 0x05;  // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const uint8_t UBX_CLASS_CFG = 0x06;  // Configuration Input Messages: Configure the receiver.
const uint8_t UBX_CLASS_UPD = 0x09;  // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const uint8_t UBX_CLASS_MON = 0x0A;  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_AID = 0x0B;  //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
const uint8_t UBX_CLASS_TIM = 0x0D;  // Timing Messages: Time Pulse Output, Time Mark Results
const uint8_t UBX_CLASS_ESF = 0x10;  //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
const uint8_t UBX_CLASS_MGA = 0x13;  // Multiple GNSS Assistance Messages: Assistance data for various GNSS
const uint8_t UBX_CLASS_LOG = 0x21;  // Logging Messages: Log creation, deletion, info and retrieval
const uint8_t UBX_CLASS_SEC = 0x27;  // Security Feature Messages
const uint8_t UBX_CLASS_HNR = 0x28;  //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
const uint8_t UBX_CLASS_NMEA = 0xF0; // NMEA Strings: standard NMEA strings
const uint8_t UBX_CLASS_PUBX = 0xF1; // Proprietary NMEA-format messages defined by u-blox

// Class: CFG
// The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
const uint8_t UBX_CFG_ANT = 0x13;       // Antenna Control Settings. Used to configure the antenna control settings
const uint8_t UBX_CFG_BATCH = 0x93;     // Get/set data batching configuration.
const uint8_t UBX_CFG_CFG = 0x09;       // Clear, Save, and Load Configurations. Used to save current configuration
const uint8_t UBX_CFG_DAT = 0x06;       // Set User-defined Datum or The currently defined Datum
const uint8_t UBX_CFG_DGNSS = 0x70;     // DGNSS configuration
const uint8_t UBX_CFG_ESFALG = 0x56;    // ESF alignment
const uint8_t UBX_CFG_ESFA = 0x4C;      // ESF accelerometer
const uint8_t UBX_CFG_ESFG = 0x4D;      // ESF gyro
const uint8_t UBX_CFG_GEOFENCE = 0x69;  // Geofencing configuration. Used to configure a geofence
const uint8_t UBX_CFG_GNSS = 0x3E;      // GNSS system configuration
const uint8_t UBX_CFG_HNR = 0x5C;       // High Navigation Rate
const uint8_t UBX_CFG_INF = 0x02;       // Depending on packet length, either: poll configuration for one protocol, or information message configuration
const uint8_t UBX_CFG_ITFM = 0x39;      // Jamming/Interference Monitor configuration
const uint8_t UBX_CFG_LOGFILTER = 0x47; // Data Logger Configuration
const uint8_t UBX_CFG_MSG = 0x01;       // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
const uint8_t UBX_CFG_NAV5 = 0x24;      // Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
const uint8_t UBX_CFG_NAVX5 = 0x23;     // Navigation Engine Expert Settings
const uint8_t UBX_CFG_NMEA = 0x17;      // Extended NMEA protocol configuration V1
const uint8_t UBX_CFG_ODO = 0x1E;       // Odometer, Low-speed COG Engine Settings
const uint8_t UBX_CFG_PM2 = 0x3B;       // Extended power management configuration
const uint8_t UBX_CFG_PMS = 0x86;       // Power mode setup
const uint8_t UBX_CFG_PRT = 0x00;       // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const uint8_t UBX_CFG_PWR = 0x57;       // Put receiver in a defined power state
const uint8_t UBX_CFG_RATE = 0x08;      // Navigation/Measurement Rate Settings. Used to set port baud rates.
const uint8_t UBX_CFG_RINV = 0x34;      // Contents of Remote Inventory
const uint8_t UBX_CFG_RST = 0x04;       // Reset Receiver / Clear Backup Data Structures. Used to reset device.
const uint8_t UBX_CFG_RXM = 0x11;       // RXM configuration
const uint8_t UBX_CFG_SBAS = 0x16;      // SBAS configuration
const uint8_t UBX_CFG_TMODE3 = 0x71;    // Time Mode Settings 3. Used to enable Survey In Mode
const uint8_t UBX_CFG_TP5 = 0x31;       // Time Pulse Parameters
const uint8_t UBX_CFG_USB = 0x1B;       // USB Configuration
const uint8_t UBX_CFG_VALDEL = 0x8C;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const uint8_t UBX_CFG_VALGET = 0x8B;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
const uint8_t UBX_CFG_VALSET = 0x8A;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

// Class: NMEA
// The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
const uint8_t UBX_NMEA_MSB = 0xF0; // All NMEA enable commands have 0xF0 as MSB. Equal to UBX_CLASS_NMEA
const uint8_t UBX_NMEA_DTM = 0x0A; // GxDTM (datum reference)
const uint8_t UBX_NMEA_GAQ = 0x45; // GxGAQ (poll a standard message (if the current talker ID is GA))
const uint8_t UBX_NMEA_GBQ = 0x44; // GxGBQ (poll a standard message (if the current Talker ID is GB))
const uint8_t UBX_NMEA_GBS = 0x09; // GxGBS (GNSS satellite fault detection)
const uint8_t UBX_NMEA_GGA = 0x00; // GxGGA (Global positioning system fix data)
const uint8_t UBX_NMEA_GLL = 0x01; // GxGLL (latitude and long, whith time of position fix and status)
const uint8_t UBX_NMEA_GLQ = 0x43; // GxGLQ (poll a standard message (if the current Talker ID is GL))
const uint8_t UBX_NMEA_GNQ = 0x42; // GxGNQ (poll a standard message (if the current Talker ID is GN))
const uint8_t UBX_NMEA_GNS = 0x0D; // GxGNS (GNSS fix data)
const uint8_t UBX_NMEA_GPQ = 0x40; // GxGPQ (poll a standard message (if the current Talker ID is GP))
const uint8_t UBX_NMEA_GQQ = 0x47; // GxGQQ (poll a standard message (if the current Talker ID is GQ))
const uint8_t UBX_NMEA_GRS = 0x06; // GxGRS (GNSS range residuals)
const uint8_t UBX_NMEA_GSA = 0x02; // GxGSA (GNSS DOP and Active satellites)
const uint8_t UBX_NMEA_GST = 0x07; // GxGST (GNSS Pseudo Range Error Statistics)
const uint8_t UBX_NMEA_GSV = 0x03; // GxGSV (GNSS satellites in view)
const uint8_t UBX_NMEA_RLM = 0x0B; // GxRMC (Return link message (RLM))
const uint8_t UBX_NMEA_RMC = 0x04; // GxRMC (Recommended minimum data)
const uint8_t UBX_NMEA_TXT = 0x41; // GxTXT (text transmission)
const uint8_t UBX_NMEA_VLW = 0x0F; // GxVLW (dual ground/water distance)
const uint8_t UBX_NMEA_VTG = 0x05; // GxVTG (course over ground and Ground speed)
const uint8_t UBX_NMEA_ZDA = 0x08; // GxZDA (Time and Date)

// The following are used to configure the NMEA protocol main talker ID and GSV talker ID
const uint8_t UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00; // main talker ID is system dependent
const uint8_t UBX_NMEA_MAINTALKERID_GP = 0x01;            // main talker ID is GPS
const uint8_t UBX_NMEA_MAINTALKERID_GL = 0x02;            // main talker ID is GLONASS
const uint8_t UBX_NMEA_MAINTALKERID_GN = 0x03;            // main talker ID is combined receiver
const uint8_t UBX_NMEA_MAINTALKERID_GA = 0x04;            // main talker ID is Galileo
const uint8_t UBX_NMEA_MAINTALKERID_GB = 0x05;            // main talker ID is BeiDou
const uint8_t UBX_NMEA_GSVTALKERID_GNSS = 0x00;           // GNSS specific Talker ID (as defined by NMEA)
const uint8_t UBX_NMEA_GSVTALKERID_MAIN = 0x01;           // use the main Talker ID

// Class: PUBX
// The following are used to enable PUBX messages with configureMessage
// See the M8 receiver description & protocol specification for more details
const uint8_t UBX_PUBX_CONFIG = 0x41;   // Set protocols and baud rate
const uint8_t UBX_PUBX_POSITION = 0x00; // Lat/Long position data
const uint8_t UBX_PUBX_RATE = 0x40;     // Set/get NMEA message output rate
const uint8_t UBX_PUBX_SVSTATUS = 0x03; // Satellite status
const uint8_t UBX_PUBX_TIME = 0x04;     // Time of day and clock information

// Class: HNR
// The following are used to configure the HNR message rates
const uint8_t UBX_HNR_ATT = 0x01; // HNR Attitude
const uint8_t UBX_HNR_INS = 0x02; // HNR Vehicle Dynamics
const uint8_t UBX_HNR_PVT = 0x00; // HNR PVT

// Class: INF
// The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_INF_CLASS = 0x04;   // All INF messages have 0x04 as the class
const uint8_t UBX_INF_DEBUG = 0x04;   // ASCII output with debug contents
const uint8_t UBX_INF_ERROR = 0x00;   // ASCII output with error contents
const uint8_t UBX_INF_NOTICE = 0x02;  // ASCII output with informational contents
const uint8_t UBX_INF_TEST = 0x03;    // ASCII output with test contents
const uint8_t UBX_INF_WARNING = 0x01; // ASCII output with warning contents

// Class: LOG
// The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_LOG_CREATE = 0x07;           // Create Log File
const uint8_t UBX_LOG_ERASE = 0x03;            // Erase Logged Data
const uint8_t UBX_LOG_FINDTIME = 0x0E;         // Find index of a log entry based on a given time, or response to FINDTIME requested
const uint8_t UBX_LOG_INFO = 0x08;             // Poll for log information, or Log information
const uint8_t UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; // Odometer log entry
const uint8_t UBX_LOG_RETRIEVEPOS = 0x0B;      // Position fix log entry
const uint8_t UBX_LOG_RETRIEVESTRING = 0x0D;   // Byte string log entry
const uint8_t UBX_LOG_RETRIEVE = 0x09;         // Request log data
const uint8_t UBX_LOG_STRING = 0x04;           // Store arbitrary string on on-board flash

// Class: MGA
// The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_MGA_ACK_DATA0 = 0x60;      // Multiple GNSS Acknowledge message
const uint8_t UBX_MGA_ANO = 0x20;            // Multiple GNSS AssistNow Offline assistance - NOT SUPPORTED BY THE ZED-F9P! "The ZED-F9P supports AssistNow Online only."
const uint8_t UBX_MGA_BDS_EPH = 0x03;        // BDS Ephemeris Assistance
const uint8_t UBX_MGA_BDS_ALM = 0x03;        // BDS Almanac Assistance
const uint8_t UBX_MGA_BDS_HEALTH = 0x03;     // BDS Health Assistance
const uint8_t UBX_MGA_BDS_UTC = 0x03;        // BDS UTC Assistance
const uint8_t UBX_MGA_BDS_IONO = 0x03;       // BDS Ionospheric Assistance
const uint8_t UBX_MGA_DBD = 0x80;            // Either: Poll the Navigation Database, or Navigation Database Dump Entry
const uint8_t UBX_MGA_GAL_EPH = 0x02;        // Galileo Ephemeris Assistance
const uint8_t UBX_MGA_GAL_ALM = 0x02;        // Galileo Almanac Assitance
const uint8_t UBX_MGA_GAL_TIMOFFSET = 0x02;  // Galileo GPS time offset assistance
const uint8_t UBX_MGA_GAL_UTC = 0x02;        // Galileo UTC Assistance
const uint8_t UBX_MGA_GLO_EPH = 0x06;        // GLONASS Ephemeris Assistance
const uint8_t UBX_MGA_GLO_ALM = 0x06;        // GLONASS Almanac Assistance
const uint8_t UBX_MGA_GLO_TIMEOFFSET = 0x06; // GLONASS Auxiliary Time Offset Assistance
const uint8_t UBX_MGA_GPS_EPH = 0x00;        // GPS Ephemeris Assistance
const uint8_t UBX_MGA_GPS_ALM = 0x00;        // GPS Almanac Assistance
const uint8_t UBX_MGA_GPS_HEALTH = 0x00;     // GPS Health Assistance
const uint8_t UBX_MGA_GPS_UTC = 0x00;        // GPS UTC Assistance
const uint8_t UBX_MGA_GPS_IONO = 0x00;       // GPS Ionosphere Assistance
const uint8_t UBX_MGA_INI_POS_XYZ = 0x40;    // Initial Position Assistance
const uint8_t UBX_MGA_INI_POS_LLH = 0x40;    // Initial Position Assitance
const uint8_t UBX_MGA_INI_TIME_UTC = 0x40;   // Initial Time Assistance
const uint8_t UBX_MGA_INI_TIME_GNSS = 0x40;  // Initial Time Assistance
const uint8_t UBX_MGA_INI_CLKD = 0x40;       // Initial Clock Drift Assitance
const uint8_t UBX_MGA_INI_FREQ = 0x40;       // Initial Frequency Assistance
const uint8_t UBX_MGA_INI_EOP = 0x40;        // Earth Orientation Parameters Assistance
const uint8_t UBX_MGA_QZSS_EPH = 0x05;       // QZSS Ephemeris Assistance
const uint8_t UBX_MGA_QZSS_ALM = 0x05;       // QZSS Almanac Assistance
const uint8_t UBX_MGA_QZAA_HEALTH = 0x05;    // QZSS Health Assistance

// Class: MON
// The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
const uint8_t UBX_MON_COMMS = 0x36; // Comm port information
const uint8_t UBX_MON_GNSS = 0x28;  // Information message major GNSS selection
const uint8_t UBX_MON_HW2 = 0x0B;   // Extended Hardware Status
const uint8_t UBX_MON_HW3 = 0x37;   // HW I/O pin information
const uint8_t UBX_MON_HW = 0x09;    // Hardware Status
const uint8_t UBX_MON_IO = 0x02;    // I/O Subsystem Status
const uint8_t UBX_MON_MSGPP = 0x06; // Message Parse and Process Status
const uint8_t UBX_MON_PATCH = 0x27; // Output information about installed patches
const uint8_t UBX_MON_RF = 0x38;    // RF information
const uint8_t UBX_MON_RXBUF = 0x07; // Receiver Buffer Status
const uint8_t UBX_MON_RXR = 0x21;   // Receiver Status Information
const uint8_t UBX_MON_SPAN = 0x31;  // Signal characteristics
const uint8_t UBX_MON_SYS = 0x39;   // Current system performance information
const uint8_t UBX_MON_TXBUF = 0x08; // Transmitter Buffer Status. Used for query tx buffer size/state.
const uint8_t UBX_MON_VER = 0x04;   // Receiver/Software Version. Used for obtaining Protocol Version.

// Class: NAV
// The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
const uint8_t UBX_NAV_ATT = 0x05;       // Vehicle "Attitude" Solution
const uint8_t UBX_NAV_CLOCK = 0x22;     // Clock Solution
const uint8_t UBX_NAV_DOP = 0x04;       // Dilution of precision
const uint8_t UBX_NAV_EOE = 0x61;       // End of Epoch
const uint8_t UBX_NAV_GEOFENCE = 0x39;  // Geofencing status. Used to poll the geofence status
const uint8_t UBX_NAV_HPPOSECEF = 0x13; // High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const uint8_t UBX_NAV_HPPOSLLH = 0x14;  // High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
const uint8_t UBX_NAV_ODO = 0x09;       // Odometer Solution
const uint8_t UBX_NAV_ORB = 0x34;       // GNSS Orbit Database Info
const uint8_t UBX_NAV_PL = 0x62;        // Protection Level Information
const uint8_t UBX_NAV_POSECEF = 0x01;   // Position Solution in ECEF
const uint8_t UBX_NAV_POSLLH = 0x02;    // Geodetic Position Solution
const uint8_t UBX_NAV_PVT = 0x07;       // All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
const uint8_t UBX_NAV_PVAT = 0x17;      // Navigation position velocity attitude time solution (ZED-F9R only)
const uint8_t UBX_NAV_RELPOSNED = 0x3C; // Relative Positioning Information in NED frame
const uint8_t UBX_NAV_RESETODO = 0x10;  // Reset odometer
const uint8_t UBX_NAV_SAT = 0x35;       // Satellite Information
const uint8_t UBX_NAV_SIG = 0x43;       // Signal Information
const uint8_t UBX_NAV_STATUS = 0x03;    // Receiver Navigation Status
const uint8_t UBX_NAV_SVIN = 0x3B;      // Survey-in data. Used for checking Survey In status
const uint8_t UBX_NAV_TIMEBDS = 0x24;   // BDS Time Solution
const uint8_t UBX_NAV_TIMEGAL = 0x25;   // Galileo Time Solution
const uint8_t UBX_NAV_TIMEGLO = 0x23;   // GLO Time Solution
const uint8_t UBX_NAV_TIMEGPS = 0x20;   // GPS Time Solution
const uint8_t UBX_NAV_TIMELS = 0x26;    // Leap second event information
const uint8_t UBX_NAV_TIMEUTC = 0x21;   // UTC Time Solution
const uint8_t UBX_NAV_VELECEF = 0x11;   // Velocity Solution in ECEF
const uint8_t UBX_NAV_VELNED = 0x12;    // Velocity Solution in NED
const uint8_t UBX_NAV_AOPSTATUS = 0x60; // AssistNow Autonomous status

// Class: RXM
// The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_RXM_COR = 0x34;       // Differential correction input status
const uint8_t UBX_RXM_MEASX = 0x14;     // Satellite Measurements for RRLP
const uint8_t UBX_RXM_PMP = 0x72;       // PMP raw data (NEO-D9S) (two different versions) (packet size for version 0x01 is variable)
const uint8_t UBX_RXM_PMREQ = 0x41;     // Requests a Power Management task (two different packet sizes)
const uint8_t UBX_RXM_RAWX = 0x15;      // Multi-GNSS Raw Measurement Data
const uint8_t UBX_RXM_RLM = 0x59;       // Galileo SAR Short-RLM report (two different packet sizes)
const uint8_t UBX_RXM_RTCM = 0x32;      // RTCM input status
const uint8_t UBX_RXM_SFRBX = 0x13;     // Broadcast Navigation Data Subframe
const uint8_t UBX_RXM_SPARTN = 0x33;    // SPARTN input status
const uint8_t UBX_RXM_SPARTNKEY = 0x36; // Poll/transfer dynamic SPARTN keys

// Class: SEC
// The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_SEC_UNIQID = 0x03; // Unique chip ID

// Class: TIM
// The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_TIM_TM2 = 0x03;  // Time mark data
const uint8_t UBX_TIM_TP = 0x01;   // Time Pulse Timedata
const uint8_t UBX_TIM_VRFY = 0x06; // Sourced Time Verification

// Class: UPD
// The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
const uint8_t UBX_UPD_SOS = 0x14; // Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup

// The following are used to enable RTCM messages
const uint8_t UBX_RTCM_MSB = 0xF5;    // All RTCM enable commands have 0xF5 as MSB
const uint8_t UBX_RTCM_1005 = 0x05;   // Stationary RTK reference ARP
const uint8_t UBX_RTCM_1074 = 0x4A;   // GPS MSM4
const uint8_t UBX_RTCM_1077 = 0x4D;   // GPS MSM7
const uint8_t UBX_RTCM_1084 = 0x54;   // GLONASS MSM4
const uint8_t UBX_RTCM_1087 = 0x57;   // GLONASS MSM7
const uint8_t UBX_RTCM_1094 = 0x5E;   // Galileo MSM4
const uint8_t UBX_RTCM_1097 = 0x61;   // Galileo MSM7
const uint8_t UBX_RTCM_1124 = 0x7C;   // BeiDou MSM4
const uint8_t UBX_RTCM_1127 = 0x7F;   // BeiDou MSM7
const uint8_t UBX_RTCM_1230 = 0xE6;   // GLONASS code-phase biases, set to once every 10 seconds
const uint8_t UBX_RTCM_4072_0 = 0xFE; // Reference station PVT (ublox proprietary RTCM message)
const uint8_t UBX_RTCM_4072_1 = 0xFD; // Additional reference station information (ublox proprietary RTCM message)

// Class: ACK
const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;
const uint8_t UBX_ACK_NONE = 0x02; // Not a real value

// Class: ESF
//  The following constants are used to get External Sensor Measurements and Status
//  Information.
const uint8_t UBX_ESF_MEAS = 0x02;
const uint8_t UBX_ESF_RAW = 0x03;
const uint8_t UBX_ESF_STATUS = 0x10;
const uint8_t UBX_ESF_RESETALG = 0x13;
const uint8_t UBX_ESF_ALG = 0x14;
const uint8_t UBX_ESF_INS = 0x15; // 36 bytes

const uint8_t SVIN_MODE_DISABLE = 0x00;
const uint8_t SVIN_MODE_ENABLE = 0x01;

// The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
const uint8_t COM_PORT_I2C = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB = 3;
const uint8_t COM_PORT_SPI = 4;

const uint8_t COM_TYPE_UBX = (1 << 0);
const uint8_t COM_TYPE_NMEA = (1 << 1);
const uint8_t COM_TYPE_RTCM3 = (1 << 5);
const uint8_t COM_TYPE_SPARTN = (1 << 6);

// Configuration Sub-Section mask definitions for saveConfigSelective (UBX-CFG-CFG)
const uint32_t VAL_CFG_SUBSEC_IOPORT = 0x00000001;   // ioPort - communications port settings (causes IO system reset!)
const uint32_t VAL_CFG_SUBSEC_MSGCONF = 0x00000002;  // msgConf - message configuration
const uint32_t VAL_CFG_SUBSEC_INFMSG = 0x00000004;   // infMsg - INF message configuration
const uint32_t VAL_CFG_SUBSEC_NAVCONF = 0x00000008;  // navConf - navigation configuration
const uint32_t VAL_CFG_SUBSEC_RXMCONF = 0x00000010;  // rxmConf - receiver manager configuration
const uint32_t VAL_CFG_SUBSEC_SENCONF = 0x00000100;  // senConf - sensor interface configuration (requires protocol 19+)
const uint32_t VAL_CFG_SUBSEC_RINVCONF = 0x00000200; // rinvConf - remove inventory configuration
const uint32_t VAL_CFG_SUBSEC_ANTCONF = 0x00000400;  // antConf - antenna configuration
const uint32_t VAL_CFG_SUBSEC_LOGCONF = 0x00000800;  // logConf - logging configuration
const uint32_t VAL_CFG_SUBSEC_FTSCONF = 0x00001000;  // ftsConf - FTS configuration (FTS products only)

// Bitfield wakeupSources for UBX_RXM_PMREQ
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX = 0x00000008;  // uartrx
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0 = 0x00000020; // extint0
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1 = 0x00000040; // extint1
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS = 0x00000080;   // spics

#endif /* GNSS_H_ */
