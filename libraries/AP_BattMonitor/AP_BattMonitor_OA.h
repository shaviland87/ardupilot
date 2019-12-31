#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_OA_EXT/optimAero.h>

class AP_BattMonitor_OA : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_OA(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params, uint8_t instance);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return false; }

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    void init(void) override {}
    void init_oa_batt(void);

private:
    uint8_t _instance;
    bool _has_cell_voltages;       // flag this as true once they have recieved a valid cell voltage report
    bool _init_oa_batt;
    uint8_t _instance_of_OA;        // which instance of OA this is
};
