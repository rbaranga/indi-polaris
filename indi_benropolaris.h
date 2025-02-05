#pragma once

#include "inditelescope.h"
#include "indiguiderinterface.h"
#include "indipropertyswitch.h"
#include "alignment/AlignmentSubsystemForDrivers.h"

// typedef enum { PARK_COUNTERCLOCKWISE = 0, PARK_CLOCKWISE } ParkDirection_t;
// typedef enum { PARK_NORTH = 0, PARK_EAST, PARK_SOUTH, PARK_WEST } ParkPosition_t;

class BenroPolaris : public INDI::Telescope, public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers {
    public:
        BenroPolaris();

        virtual bool initProperties() override;
        virtual void ISGetProperties(const char *dev) override;
        virtual bool updateProperties() override;
        virtual bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                               char *formats[], char *names[], int n) override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

    protected:
        /////////////////////////////////////////////////////////////////////////////////////
        /// Communication
        /////////////////////////////////////////////////////////////////////////////////////
        virtual bool Handshake() override;
        virtual bool Connect() override;
        virtual bool Disconnect() override;
        virtual bool ReadScopeStatus() override;
        
        /////////////////////////////////////////////////////////////////////////////////////
        /// Motion
        /////////////////////////////////////////////////////////////////////////////////////
        virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
        virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
        virtual bool Goto(double ra, double dec) override;
        virtual bool Sync(double ra, double dec) override;
        virtual bool Abort() override;
        virtual bool SetTrackEnabled(bool enabled) override;
        virtual bool SetTrackMode(uint8_t mode) override;
        virtual bool SetTrackRate(double raRate, double deRate) override;

        /////////////////////////////////////////////////////////////////////////////////////
        /// Misc.
        /////////////////////////////////////////////////////////////////////////////////////
        virtual const char *getDefaultName() override;
        virtual void TimerHit() override;
        virtual bool updateLocation(double latitude, double longitude, double elevation) override;
        // virtual bool saveConfigItems(FILE *fp) override;
        // double GetSlewRate();
        // double GetParkDeltaAz(ParkDirection_t target_direction, ParkPosition_t target_position);

        /////////////////////////////////////////////////////////////////////////////////////
        /// Guiding
        /////////////////////////////////////////////////////////////////////////////////////
        // virtual IPState GuideNorth(uint32_t ms) override;
        // virtual IPState GuideSouth(uint32_t ms) override;
        // virtual IPState GuideEast(uint32_t ms) override;
        // virtual IPState GuideWest(uint32_t ms) override;

        /////////////////////////////////////////////////////////////////////////////////////
        /// Parking
        /////////////////////////////////////////////////////////////////////////////////////
        virtual bool Park() override;
        virtual bool UnPark() override;
        virtual bool SetCurrentPark() override;
        virtual bool SetDefaultPark() override;

    private:
        /////////////////////////////////////////////////////////////////////////////////////
        /// Comunication
        /////////////////////////////////////////////////////////////////////////////////////
        void WriteRequest(std::string request, bool readResponse = true, int retries = 3);
        void ReadResponses(int portRef);
        int readResponseCallback;

        // void Keepalive();
        // int keepaliveTimer;
        
        std::map<int, std::pair<std::map<std::string, std::string>, int64_t>> responses;
        void StoreResponseAndUpdateState(std::pair<int, std::map<std::string, std::string>> decodedResponse);

        /////////////////////////////////////////////////////////////////////////////////////
        /// Message Encoding/Decoding
        /////////////////////////////////////////////////////////////////////////////////////
        std::pair<int, std::map<std::string, std::string>> DecodeResponse(std::string message);
        std::string EncodeRequest(int command, int type, std::map<std::string, std::string> data);
        std::string EncodeRequest(int command, int type, std::string data);
        std::string EncodeRequest(int command, int type);

        /////////////////////////////////////////////////////////////////////////////////////
        /// Properties
        /////////////////////////////////////////////////////////////////////////////////////
        INDI::PropertyNumber AltAzNP {2};
        enum
        {
            ALT,
            AZM,
        };

        INDI::PropertyText DeviceInfoTP {5};
        enum
        {
            HARDWARE_VERSION,
            SOFTWARE_VERSION,
            ASTRO_MODULE_VERSION,
            SV,
            OV,
        };

        INDI::PropertyNumber StorageNP {3};
        enum
        {
            TOTAL,
            FREE,
            USED,
        };

        INDI::PropertyNumber BatteryNP {1};
        enum
        {   
            CAPACITY,
        };

        INDI::PropertyText CommandTP {2};
        enum
        {   
            REQUEST,
            RESPONSE,
        };
};