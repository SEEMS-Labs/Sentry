// Include gaurd.
#ifndef STATEMANAGER_H
#define STATEMANAGER_H

#include "sentryConfigInfo.h"

/**
 * Class designed to monitor and manage all the states of the sentry.
 * Follows a singleton design pattern.
 */
class StateManager {

    private:
        PowerState sps;                 // Sentry Power State.
        WakeState sws;                  // Sentry Wake State.
        DataTransmissionState sdts;     // Sentry Data Transmission State.
        MovementState sms;              // Sentry Movement State.
        StartupState sss;               // Sentry Startup State.
        ConnectionState ncs;            // Sentry Network Connectivity State.

        inline static StateManager* manager_instance = NULL;  // Pointer to StateManager instance.
        StateManager() : 
            sps(PowerState::ps_CHARGED),            // Default to charged state.
            sws(WakeState::ws_STANDBY),             // Default to standby state.
            sdts(DataTransmissionState::ds_IDLE),   // Default to idle transmission state.
            sms(MovementState::ms_IDLE),            // Default to idle movement state.
            sss(StartupState::ss_INVALID),          // Default to invalid startup state.
            ncs(ConnectionState::ns_NONE) {}        // Default to no networks connected state.

    public:
        PowerState getSentryPowerState()                            { return sps; } 
        WakeState getSentryWakeState()                              { return sws; }
        DataTransmissionState getSentryDataTransmissionState()      { return sdts; }
        MovementState getSentryMovementState()                      { return sms; }
        StartupState getSentryStartupState()                        { return sss; }
        ConnectionState getSentryConnectionState()                  { return ncs; }

        void setSentryPowerState(PowerState state)                          { this->sps = state; }
        void setSentryWakeState(WakeState state)                            { this->sws = state; }
        void setSentryDataTransmissionState(DataTransmissionState state)    { this->sdts = state; }
        void setSentryMovementState(MovementState state)                    { this->sms = state; }
        void setSentryStartupState(StartupState state)                      { this->sss = state; }
        void setSentryConnectionState(ConnectionState state)                { this->ncs = state; } 

        StateManager(const StateManager *obj) = delete;     // Delete the copy ctor.
        
        // Return the singleton instance of the state manager.
        static StateManager *getManager() {
            if(manager_instance == NULL) manager_instance = new StateManager();
            return manager_instance;
        }

};

// End include gaurd.
#endif /* SentryState.h */