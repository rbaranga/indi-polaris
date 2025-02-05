#include "indi_benropolaris.h"

#include "iostream"
#include "map"
#include "string"
#include "thread"
#include "mutex"
#include "fstream"
#include "chrono"
#include "regex"
#include "sstream"
#include "termios.h"
#include "connectionplugins/connectiontcp.h"
#include "indicom.h"

// using namespace INDI::AlignmentSubsystem;

const int CMD_284_MODE       = 284; // 1&284&2&-1# => 284@284, halfSpeed:0;interval:;mode:8;pause:;photoNum:;remNum:;repeNum:;runTime:;speed:0;state:0;track:0;#
// const int CMD_296_?       = 296; // 1&296&2&-1# => ???
// const int CMD_303_?       = 303; // 1&303&2&-1# => ???
// const int CMD_305_?       = 305; // 1&305&2&step:2;# => ??
const int CMD_518_AHRS       = 518;
const int CMD_519_GOTO       = 519;
const int CMD_520_POSITION   = 520; // 1&520&2&state:1;# => 527@ret:1;#
const int CMD_523_RESET_AXIS = 523; // 1&523&3&axis:1;# => ??
// const int CMD_524_?    = 524; // 1&524&3&-1# => ??
const int CMD_525_UNKNOWN    = 525;
// Fast move commands '513', '514', '521'
// Slow move commands '532', '533', '534'
// const int CMD_527_COMPASS    = 527; // 1&527&3&compass:{compass};lat:{lat};lng:{lon};# => ???
const int CMD_531_TRACK      = 531; // 1&531&3&state:1;speed:0;# => 
// const int CMD_771_FILES      = 771;
const int CMD_775_STORAGE    = 775; // 1&775&2&-1# => 775@status:1;totalspace:30417;freespace:30373;usespace:43;#
const int CMD_778_BATTERY    = 778; // 1&778&2&-1# => 778@capacity:99;charge:0;#
const int CMD_780_VERSION    = 780; // 1&780&2&-1# => 780@hw:1.2.1.2;sw:6.0.0.48;exAxis:;sv:1;ov: ;
// const int CMD_790_SECURITY   = 790;
// const int CMD_799_?   = 799; // #1&799&2&-1# => 799@ret:-1;#
// const int CMD_802_WIFI       = 802; // #1&802&2&-1# => 802@band:0;#
const int CMD_808_CONNECTION = 808; // 1&808&2&type:0;# => 808@ret:0;#

const int POSITION_UPDATE_MAX_AGE = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(5)).count();
const int POSITION_UPDATE_REFRESH_AGE = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(2)).count();
const int MODE_UPDATE_REFRESH_AGE = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(15)).count();
const int POLLING_PERIOD = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(1)).count();
const int KEEPALIVE_PERIOD = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(500)).count();

static std::unique_ptr<BenroPolaris> polaris(new BenroPolaris());

BenroPolaris::BenroPolaris() {
    setVersion(0, 1);
    setTelescopeConnection(CONNECTION_TCP);

    // We add an additional debug level so we can log verbose scope status
    // DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
    SetTelescopeCapability(
        TELESCOPE_CAN_GOTO                  | /** Can the telescope go to to specific coordinates? */
        TELESCOPE_CAN_SYNC                  | /** Can the telescope sync to specific coordinates? */
        TELESCOPE_CAN_PARK                  | /** Can the telescope park? */
        TELESCOPE_CAN_ABORT                 | /** Can the telescope abort motion? */
        // ? TELESCOPE_HAS_TIME                  | /** Does the telescope have configurable date and time settings? */
        // ? TELESCOPE_HAS_LOCATION              | /** Does the telescope have configuration location settings? */
        // TELESCOPE_HAS_PIER_SIDE             | /** Does the telescope have pier side property? */
        // TELESCOPE_HAS_PEC                   | /** Does the telescope have PEC playback? */
        TELESCOPE_HAS_TRACK_MODE            | /** Does the telescope have track modes (sidereal, lunar, solar..etc)? */
        TELESCOPE_CAN_CONTROL_TRACK          /** Can the telescope engage and disengage tracking? */
        // TELESCOPE_HAS_TRACK_RATE            | /** Does the telescope have custom track rates? */
        // TELESCOPE_HAS_PIER_SIDE_SIMULATION  | /** Does the telescope simulate the pier side property? */
        // TELESCOPE_CAN_TRACK_SATELLITE       | /** Can the telescope track satellites? */
        // TELESCOPE_CAN_FLIP                  | /** Does the telescope have a command for flipping? */
        // TELESCOPE_CAN_HOME_FIND             | /** Can the telescope find home position? */
        // ? TELESCOPE_CAN_HOME_SET              | /** Can the telescope set the current position as the new home position? */
        // ? TELESCOPE_CAN_HOME_GO               /** Can the telescope slew to home position? */
    , 4);
}

/**************************************************************************************
 ** INDI is asking us for our default device name
 ***************************************************************************************/
const char *BenroPolaris::getDefaultName() {
    return "Benro Polaris";
}

/**************************************************************************************
 ** We init our properties here.
 ***************************************************************************************/
bool BenroPolaris::initProperties() {
    /* Make sure to init parent properties first */
    const bool parentInitialised = INDI::Telescope::initProperties();

    addDebugControl();

    AltAzNP[AZM].fill("AZM", "Azm (dd:mm:ss)", "%010.6m", 0, 360, 0, 0);
    AltAzNP[ALT].fill("ALT", "Alt (hh:mm:ss)", "%010.6m", 0, 90, 0, 0);
    AltAzNP.fill(getDeviceName(), "ALTAZ_COORD", "Coordinates", MAIN_CONTROL_TAB,
              IP_RO, 60, IPS_IDLE);
    defineProperty(AltAzNP);
    AltAzNP.load();

    DeviceInfoTP[ASTRO_MODULE_VERSION].fill("HARDWARE_VERSION", "Head hadware version", "-");
    DeviceInfoTP[SOFTWARE_VERSION].fill("SOFTWARE_VERSION", "Head software version", "-");
    DeviceInfoTP[ASTRO_MODULE_VERSION].fill("ASTRO_MODULE_VERSION", "Astro module version", "-");
    DeviceInfoTP[SV].fill("???", "???", "-");
    DeviceInfoTP[OV].fill( "???", "???", "-");
    DeviceInfoTP.fill(getDeviceName(), "DEVICE_INFO", "Device Info", INFO_TAB, IP_RO, 0, IPS_IDLE);
    
    StorageNP[TOTAL].fill("TOTAL_STORAGE", "Total storage", "%.0f", 0., 99999., 0., 0.);
    StorageNP[FREE].fill("FREE_STORAGE", "Free storage", "%.0f", 0., 99999., 0., 0.);
    StorageNP[USED].fill("USED_STORAGE", "Used storage", "%.0f", 0., 99999., 0., 0.);
    StorageNP.fill(getDeviceName(), "STORAGE_INFO", "Storage Info", INFO_TAB, IP_RO, 0, IPS_IDLE);
    
    BatteryNP[CAPACITY].fill("BATTERY", "Battery (%)", "%3.0f", 0., 100., 0., 0.);
    BatteryNP.fill(getDeviceName(), "BATTERY", "Battery", INFO_TAB, IP_RO, 0, IPS_IDLE);

    CommandTP[REQUEST].fill("REQUEST", "Request", "");
    CommandTP[RESPONSE].fill("RESPONSE", "Response", "");
    CommandTP.fill(getDeviceName(), "COMMAND", "Command", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);
    
    setCurrentPollingPeriod(POLLING_PERIOD);
    return parentInitialised;
}

/**************************************************************************************
 ** We update our properties here.
 ***************************************************************************************/
bool BenroPolaris::updateProperties() {
    const bool parentUpdated = INDI::Telescope::updateProperties();

    LOGF_INFO("Current mount status: %s", isConnected() ? "Connected" : "Disconnected");
    if (isConnected()) {
        WriteRequest(EncodeRequest(CMD_780_VERSION, 2));
        WriteRequest(EncodeRequest(CMD_775_STORAGE, 2));
        WriteRequest(EncodeRequest(CMD_778_BATTERY, 2));

        defineProperty(DeviceInfoTP);
        DeviceInfoTP.load();
        defineProperty(StorageNP);
        StorageNP.load();
        defineProperty(BatteryNP);
        BatteryNP.load();
        defineProperty(CommandTP);
        CommandTP.load();
    } else {
        deleteProperty(DeviceInfoTP);
        deleteProperty(StorageNP);
        deleteProperty(BatteryNP);
        deleteProperty(CommandTP);
    }
    
    return parentUpdated;
}

/////////////////////////////////////////////////////////////////////////////////////
/// Prop changes
////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************
 ** INDI is asking us to set custom properties
 ***************************************************************************************/
void BenroPolaris::ISGetProperties(const char *dev) {
    INDI::Telescope::ISGetProperties(dev);
}

/**************************************************************************************
 ** INDI is notifying us to that a new property was set
 ***************************************************************************************/
bool BenroPolaris::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
    LOGF_INFO("ISNewBLOB: %s", name);

    // Pass it up the chain
    return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

/**************************************************************************************
 ** INDI is notifying us to that a new property was set
 ***************************************************************************************/
bool BenroPolaris::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    LOGF_INFO("ISNewNumber: %s", name);
    
    // Pass it up the chain
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
 ** INDI is notifying us to that a new property was set
 ***************************************************************************************/
bool BenroPolaris::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    LOGF_INFO("ISNewSwitch: %s", name);

    // Pass it up the chain
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
 ** INDI is notifying us to that a new property was set
 ***************************************************************************************/
bool BenroPolaris::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
    LOGF_INFO("ISNewText: %s", name);

    if (std::strcmp(name, CommandTP.getName()) == 0) {
        if (std::strlen(texts[REQUEST]) > 0 && strcasecmp(texts[REQUEST], CommandTP[REQUEST].getText()) != 0) {
            WriteRequest(texts[REQUEST]);
        }
        if (std::strlen(texts[RESPONSE]) > 0 && strcasecmp(texts[RESPONSE], CommandTP[RESPONSE].getText()) != 0) {
            StoreResponseAndUpdateState(DecodeResponse(texts[RESPONSE]));
        }
    }

    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

/////////////////////////////////////////////////////////////////////////////////////
/// Communication
////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************************
 ** INDI wants us to connect to the telescope
 ***************************************************************************************/
bool BenroPolaris::Connect() {
    const bool connected = INDI::Telescope::Connect();
    if (connected) {
        readResponseCallback = IEAddCallback(PortFD, [](int fileRef, void* instance) {
            static_cast<BenroPolaris*>(instance)->ReadResponses(fileRef);
        }, this);

        SetTimer(KEEPALIVE_PERIOD);
    }
    return connected;
}

/**************************************************************************************
 ** INDI wants us to disconnect from the telescope
 ***************************************************************************************/
bool BenroPolaris::Disconnect() {
    const bool disconnected = INDI::Telescope::Disconnect();
    if (disconnected) {
        // IERmTimer(keepaliveTimer);
        IERmCallback(readResponseCallback);
    }
    return disconnected;
}

/**************************************************************************************
 ** Client is asking us to complete handshake with the telescope
 ***************************************************************************************/
bool BenroPolaris::Handshake() {
    LOG_INFO("Handshake");

    bool fakeDevice = true;
    if (!isSimulation()) {
        WriteRequest(EncodeRequest(CMD_284_MODE, 2));

        if (responses.find(CMD_284_MODE) != responses.end()) {
            std::string mode = responses[CMD_284_MODE].first["mode"];
            std::string track = responses[CMD_284_MODE].first["track"];
            
            if (mode == "8") {
                if (track == "3") {
                    WriteRequest(EncodeRequest(CMD_808_CONNECTION, 2, {{"type", "0"}}));
                    WriteRequest(EncodeRequest(CMD_520_POSITION, 2, {{"state", "1"}}));
                    
                    // // TODO: later
                    // # if we want to run Aim test or Drift test over a set of targets in the sky
                    // if Config.log_performance_data_test == 1 or Config.log_performance_data_test == 2:
                    //     asyncio.create_task(self.goto_tracking_test())
                    // # if we want to run Speed test to ramp moveaxis rate over its full range
                    // if Config.log_performance_data == 3 and Config.log_performance_data_test == 3:
                    //     asyncio.create_task(self.moveaxis_ramp_speed_test())

                } else {
                    LOGF_INFO("Invalid track %s, expected 3", track.c_str());
                    LOG_ERROR("Polaris is not aligned and tracking, please use app to do a basic alignment and reconnect driver");
                    return fakeDevice;
                }
            } else {
                LOGF_INFO("Invalid mode %s, expected 8", mode.c_str());
                LOG_ERROR("Polaris is not in astro mode, please use app to switch to astro mode and reconnect driver");
                return fakeDevice;
            }
        } else {
            LOG_ERROR("Failed to get proper reponse from polaris, try to reconnect wifi and driver");
            return fakeDevice;
        }   
    }

    return true;
}

/**************************************************************************************
 ** Client is asking us to report telescope status
 ***************************************************************************************/
bool BenroPolaris::ReadScopeStatus() {
    if (!isConnected()) {
        LOG_ERROR("Not connected to the telescope");        
    }

    LOG_INFO("Scope status:");
    for (const auto& pair : responses) {
        std::string dataString = "";
        for (const auto& data : pair.second.first) {
            dataString += data.first + ":" + data.second + ";";
        }
        
        LOGF_INFO(" - %d => %s (%d)", pair.first, dataString.c_str(), pair.second.second);
    }

    return true;
}

/**************************************************************************************
 ** Write a request to the telescope
 ***************************************************************************************/
void BenroPolaris::WriteRequest(std::string request, bool readResponse, int retries) {
    int errorCode = 0;
    int bytesWritten = 0;
    if ((errorCode = tty_write_string(PortFD, request.c_str(), &bytesWritten)) != TTY_OK) {
        if (retries > 0) {
            WriteRequest(request, readResponse,  - 1);
        } else {
            LOGF_ERROR("Failed to send request '%s' with error %d", request.c_str(), errorCode);
            setConnected(false);
        }
    }
    
    LOGF_INFO("Sent request: %s", request.c_str());
    tcflush(PortFD, TCIFLUSH);
}

/**************************************************************************************
 ** Read response from the telescope
 ***************************************************************************************/
void BenroPolaris::ReadResponses(int fileRef) {
    if (fileRef != PortFD) {
        LOGF_WARN("Different file reference %d vs %d", fileRef, PortFD);
    }

    int errorCode = 0;
    do {
        int bytesRead = 0;
        char response[256];

        if ((errorCode = tty_read_section(PortFD, response, '#', 1, &bytesRead)) == TTY_OK) {
            response[bytesRead] = '\0';
            // LOGF_INFO("Response: %s", response);
            StoreResponseAndUpdateState(DecodeResponse(response));
        }
    } while (errorCode == TTY_OK);
}

void BenroPolaris::StoreResponseAndUpdateState(std::pair<int, std::map<std::string, std::string>> decodedResponse) {
    const int code = decodedResponse.first;
    std::string dataString = "";
    responses[code] = std::make_pair(decodedResponse.second, std::chrono::system_clock::now().time_since_epoch().count());
    for (const auto& data : decodedResponse.second) {
        dataString += data.first + ":" + data.second + ";";
    }

    // LOGF_INFO("Response: %d, %s", code, dataString.c_str());
    switch (code) {
        case CMD_284_MODE:
            // 284@mode:8;state:0;track:3;speed:0;halfSpeed:0;remNum:;runTime:;photoNum:;pause:;interval:;repeNum:;#

            break;

        case CMD_518_AHRS:
            // 518@w:0.4402258;x:-0.5703645;y:-0.5810719;z:-0.3784723;w:-0.3784722;x:-0.5703645;y:-0.5810719;z:-0.4402257;compass:175.1536255;alt:-19.0213356;#
            INDI::IHorizontalCoordinates AltAz { std::stof(decodedResponse.second["alt"]), std::stof(decodedResponse.second["compass"]) };
            if (std::abs(AltAzNP[ALT].getValue() - AltAz.altitude) > 0.001 ||
                std::abs(AltAzNP[AZM].getValue() - AltAz.azimuth) > 0.001) {
                INDI::IEquatorialCoordinates Eq { 0, 0 };
                INDI::HorizontalToEquatorial(&AltAz, &m_Location, ln_get_julian_from_sys(), &Eq);
                
                AltAzNP[AZM].setValue(AltAz.azimuth);
                AltAzNP[ALT].setValue(AltAz.altitude);
                AltAzNP.apply();

                NewRaDec(Eq.rightascension, Eq.declination);
            }
            break;
        case CMD_519_GOTO:
            // 519@ret:1;track:0;#
            // 519@ret:0;track:0;#
            if (decodedResponse.second["ret"] == "1") {
                
            //     TrackState = SCOPE_SLEWING;
            } else if (decodedResponse.second["track"] == "1") {
            //     TrackState = SCOPE_TRACKING;
            } else {
            //     TrackState = SCOPE_IDLE;
            }
            break;
        case CMD_531_TRACK:
            // 531@ret:3;#
            if (decodedResponse.second["ret"] == "0") {
                TrackState = SCOPE_IDLE;
            } else {
                TrackState = SCOPE_TRACKING;
            }
            break;

        case CMD_780_VERSION:
            // 780@hw:1.2.1.2;sw:6.0.0.48;exAxis:1.0.2.14;sv:1;ov: ;#
            DeviceInfoTP[HARDWARE_VERSION].setText(decodedResponse.second["hw"].c_str());
            DeviceInfoTP[SOFTWARE_VERSION].setText(decodedResponse.second["sw"].c_str());
            DeviceInfoTP[ASTRO_MODULE_VERSION].setText(decodedResponse.second["exAxis"].c_str());
            DeviceInfoTP[SV].setText(decodedResponse.second["sv"].c_str());
            DeviceInfoTP[OV].setText(decodedResponse.second["ov"].c_str());
            DeviceInfoTP.apply();
            break;
        
        case CMD_775_STORAGE:
            // 775@status:1;totalspace:30417;freespace:30408;usespace:8;#
            StorageNP[TOTAL].setValue(std::stof(decodedResponse.second["totalspace"]));
            StorageNP[FREE].setValue(std::stof(decodedResponse.second["freespace"]));
            StorageNP[USED].setValue(std::stof(decodedResponse.second["usespace"]));
            StorageNP.setState(decodedResponse.second["status"] == "1" ? IPS_OK : IPS_ALERT);
            StorageNP.apply();
            break;
        
        case CMD_778_BATTERY:
            // 778@capacity:99;charge:0;#
            BatteryNP[CAPACITY].setValue(std::stof(decodedResponse.second["capacity"]));
            BatteryNP.setState(decodedResponse.second["charge"] == "1" ? IPS_OK : IPS_IDLE);
            BatteryNP.apply();
            break;
        
        case CMD_525_UNKNOWN:
            break;

        default:
            LOGF_INFO("Response: %d, %s", code, dataString.c_str());
            break;
    }
}


/**************************************************************************************
 ** Encode Request
 ***************************************************************************************/
std::string BenroPolaris::EncodeRequest(int command, int type, std::map<std::string, std::string> data) {
    if (data.empty()) {
        return EncodeRequest(command, type);
    }

    std::string dataAsString = "";
    for (const auto& pair : data) {
        dataAsString += pair.first + ":" + pair.second + ";";
    }

    return EncodeRequest(command, type, dataAsString);
}

/**************************************************************************************
 ** Encode Request
 ***************************************************************************************/
std::string BenroPolaris::EncodeRequest(int command, int type) {
    return EncodeRequest(command, type, "-1");
}

/**************************************************************************************
 ** Encode Request
 ***************************************************************************************/
std::string BenroPolaris::EncodeRequest(int command, int type, std::string data) {
    if (data.empty()) {
        return EncodeRequest(command, type);
    }

    std::string request = "1&" + std::to_string(command) + "&" + std::to_string(type) + "&" + data + "#";
    return request;
}

/**************************************************************************************
 ** Decode Response
 ***************************************************************************************/
std::pair<int, std::map<std::string, std::string>> BenroPolaris::DecodeResponse(std::string message) {
    std::regex pattern(R"(^(\d{3})@([^#]*)#)");
    std::cmatch matches;
    if (std::regex_match(message.c_str(), matches, pattern)) {
        int command = std::stoi(matches[1]);
        std::istringstream ss(matches[2]);
        std::string pair;

        std::map<std::string, std::string> data;
        while (std::getline(ss, pair, ';')) {
            size_t pos = pair.find(':');
            if (pos != std::string::npos) {
                std::string key = pair.substr(0, pos);
                std::string value = pair.substr(pos + 1);
                data[key] = value;
            }
        }
        return std::make_pair(command, data);
    }

    return std::make_pair(-1, std::map<std::string, std::string>());
}

/////////////////////////////////////////////////////////////////////////////////////
/// Motion
/////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************************
 ** Client is asking us to move in NS axis
 ***************************************************************************************/
bool BenroPolaris::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) {
    LOGF_INFO("MoveNS: %d : %s", dir, command == MOTION_START ? "start" : "stop");
    return true;
}

/**************************************************************************************
 ** Client is asking us to move in WE axis
 ***************************************************************************************/
bool BenroPolaris::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) {
    LOGF_INFO("MoveWE: %d : %s", dir, command == MOTION_START ? "start" : "stop");
    return true;
}

/**************************************************************************************
 ** Client is asking us to go to specific coordinates
 ***************************************************************************************/
bool BenroPolaris::Goto(double ra, double dec) {
    if (TrackState != SCOPE_IDLE) {
        Abort();
    }

    LOGF_INFO("GOTO: RA %lf DEC %lf", ra, dec);

    INDI::IEquatorialCoordinates Eq { ra, dec };
    INDI::IHorizontalCoordinates AltAz { 0, 0 };

    INDI::EquatorialToHorizontal(&Eq, &m_Location, ln_get_julian_from_sys(), &AltAz);

    WriteRequest(EncodeRequest(CMD_519_GOTO, 3, {
        {"state", "1"},
        {"yaw", std::to_string(std::round(AltAz.azimuth * 10000) / 10000)},
        {"pitch", std::to_string(std::round(AltAz.altitude * 10000) / 10000)},
        {"lat", std::to_string(std::round(m_Location.latitude * 10000) / 10000)},
        {"track", TrackState == SCOPE_TRACKING ? "1" : "0" },
        {"speed", "0"},
        {"lng", std::to_string(std::round(m_Location.longitude * 10000) / 10000)},
    }));
    
    LOGF_INFO("NEW GOTO TARGET: Ra %lf Dec %lf - Alt %lf Az %lf", ra, dec, AltAz.altitude, AltAz.azimuth);
    return true;
}

/**************************************************************************************
 ** Client is asking us to sync to specific coordinates
 ***************************************************************************************/
bool BenroPolaris::Sync(double ra, double dec) {
    LOGF_INFO("Sync: %f, %f", ra, dec);
    return true;
}

/**************************************************************************************
 ** Client is asking us to abort motion
 ***************************************************************************************/
bool BenroPolaris::Abort() {
    LOG_INFO("Abort");

    // cmd = '519'
    // msg = f"1&{cmd}&3&state:0;yaw:0.0;pitch:0.0;lat:{self._sitelatitude:.5f};track:0;speed:0;lng:{self._sitelongitude:.5f};#"
    WriteRequest(EncodeRequest(CMD_519_GOTO, 3, {
        {"state", "0"},
        {"yaw", "0.0"},
        {"pitch", "0.0"},
        {"lat", std::to_string(std::round(LocationNP[LOCATION_LATITUDE].getValue() * 10000) / 10000)},
        {"track", "0"},
        {"speed", "0"},
        {"lng", std::to_string(std::round(LocationNP[LOCATION_LONGITUDE].getValue() * 10000) / 10000)},
    }));
    TrackState = SCOPE_IDLE;
    return true;
}

/**************************************************************************************
 ** Client is asking us to set tracking enabled
 ***************************************************************************************/
bool BenroPolaris::SetTrackEnabled(bool enabled) {
    LOGF_INFO("SetTrackEnabled: %s", enabled ? "true" : "false");
    // cmd = '531'
    // msg = f"1&{cmd}&3&state:{state};speed:0;#"
    WriteRequest(EncodeRequest(CMD_531_TRACK, 3, {
        { "state", enabled ? "1" : "0" }, { "speed", "0" },
    }));
    return true;
}

/**************************************************************************************
 ** Client is asking us to set tracking mode
 ***************************************************************************************/
bool BenroPolaris::SetTrackMode(uint8_t mode) {
    LOGF_INFO("SetTrackMode: %d", mode);
    return true;
}

/**************************************************************************************
 ** Client is asking us to set tracking rate
 ***************************************************************************************/
bool BenroPolaris::SetTrackRate(double raRate, double deRate) {
    LOGF_INFO("SetTrackRate: %f, %f", raRate, deRate);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////
/// Parking
/////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************************
 ** Client is asking us to park the telescope
 ***************************************************************************************/
bool BenroPolaris::Park() {
    if (TrackState == SCOPE_TRACKING) {
        SetTrackEnabled(false);
    }
    WriteRequest(EncodeRequest(CMD_523_RESET_AXIS, 3, {{"axis", "1"}}));
    WriteRequest(EncodeRequest(CMD_523_RESET_AXIS, 3, {{"axis", "2"}}));
    WriteRequest(EncodeRequest(CMD_523_RESET_AXIS, 3, {{"axis", "3"}}));

    TrackState = SCOPE_PARKED;
    return true;
}

/**************************************************************************************
 ** Client is asking us to unpark the telescope
 ***************************************************************************************/
bool BenroPolaris::UnPark() {
    LOG_INFO("UnPark");
    return true;
}

/**************************************************************************************
 ** Client is asking us to set current park
 ***************************************************************************************/
bool BenroPolaris::SetCurrentPark() {
    LOG_INFO("SetCurrentPark");
    return true;
}

/**************************************************************************************
 ** Client is asking us to set default park
 ***************************************************************************************/
bool BenroPolaris::SetDefaultPark() {
    LOG_INFO("SetDefaultPark");
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////
/// Misc.
/////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************************
 ** Client is notifiying us that the timer has hit, so it's time to do some work
 ***************************************************************************************/
void BenroPolaris::TimerHit() {
    if (!isConnected()) {
        return;
    }

    const int64_t now = std::chrono::system_clock::now().time_since_epoch().count();
    const int16_t positionUpdateAge = responses.find(CMD_518_AHRS) != responses.end()
        ? responses[CMD_518_AHRS].second
        : POSITION_UPDATE_MAX_AGE;
    if (positionUpdateAge >= POSITION_UPDATE_MAX_AGE) {
        LOG_WARN("Last position update more than 5 seconds ago, tracking?");
    }
    if (positionUpdateAge >= POSITION_UPDATE_REFRESH_AGE) {
        LOG_INFO("Last position update more than 2 seconds ago, requesting new update");
        WriteRequest(EncodeRequest(CMD_520_POSITION, 2, {{"state", "1"}}));
    }
   
    if (responses.find(CMD_284_MODE) == responses.end() 
        || now - responses[CMD_284_MODE].second > MODE_UPDATE_REFRESH_AGE) {
        LOG_INFO("Last position update more than 2 seconds ago, requesting new update");
        WriteRequest(EncodeRequest(CMD_284_MODE, 2));
    }

    SetTimer(KEEPALIVE_PERIOD);
}

/**************************************************************************************
 ** Client is giving a new location
 ***************************************************************************************/
bool BenroPolaris::updateLocation(double latitude, double longitude, double elevation) {
    LOGF_INFO("updateLocation: %f, %f, %f", latitude, longitude, elevation);

    return true;
}

/**************************************************************************************
 ** Perform regular commands in specific intervals
 ***************************************************************************************/
// void BenroPolaris::CheckDrift() {
//     while (isConnected()) {
//         {
//             LOG_INFO("Checking drift...");
//             // async def drift_error_test(self, ra, dec, duration=120):
//             //     a0_ra = self._rightascension
//             //     a0_dec = self._declination
//             //     a0_track = self.tracking
//             //     t0 = datetime.datetime.now()
//             //     await asyncio.sleep(duration)
//             //     a1_ra = self._rightascension
//             //     a1_dec = self._declination
//             //     a1_track = self.tracking
//             //     t1 = datetime.datetime.now()
//             //     d_t = (t1 - t0).total_seconds()
//             //     d_ra = clamparcsec((a0_ra - a1_ra)*3600/24*360)/d_t*60
//             //     d_dec = clamparcsec((a0_dec - a1_dec)*3600)/d_t*60
//             //     a_ra = ra if ra else self._targetrightascension if self._targetrightascension else self._rightascension
//             //     a_dec = dec if dec else self._targetdeclination if self._targetdeclination else self._declination
//             //     time = self.get_performance_data_time()
//             //     self.logger.info(f",DATA2,{time:.3f},{a0_track},{a1_track},{a_ra},{a_dec},{d_ra:.3f},{d_dec:.3f}")
//             //     return        
//         }

//         // Wait for some time before reading again
//         std::this_thread::sleep_for(std::chrono::minutes(2));
//     }
// }