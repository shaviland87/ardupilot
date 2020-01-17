#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_OA.h"

//#include <stdio.h>
/*
reads data from external source
 */
extern const AP_HAL::HAL& hal;


/// Constructor
AP_BattMonitor_OA::AP_BattMonitor_OA(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       uint8_t instance) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _instance(instance)
{
    _init_oa_batt       = false;
    _has_cell_voltages  = true; // tbd temporary
}

void AP_BattMonitor_OA::init_oa_batt(void){

    unsigned char number_of_oa_instances    = 0;
    _instance_of_OA                         = 0;

    for(int j=0; j<_mon.num_instances(); j++)
    {
        // loop thru 1-n instances -- this particular instance is _instance(k) -- we want to know which paritcular oa instance this is
        if(_mon.get_type(j) == AP_BattMonitor_Params::BattMonitor_Type_OA )
        {
            // we have confirmed the jth instance is an oa type
            if( j == _instance ){
                //this means the current loop we are on is same instance
                _instance_of_OA = number_of_oa_instances;    
            }
            number_of_oa_instances++;  // add to the number of oa instances that exist
        }
    }
}

// read the voltage 
void
AP_BattMonitor_OA::read()
{
    const optimAero *oa_ = AP::oa();
    AP_Int8 typeOA;

    if(!_init_oa_batt){
        init_oa_batt(); // figure out how many oa modules are configured 
    }

    for(uint8_t i = 0; i< oa_->num_oa_connections(); i++){
        typeOA = oa_->get_type(i);
        if(typeOA == optimAero::optimAero_TYPE_ARDUINO || typeOA == optimAero::optimAero_TYPE_MULTI)
        {
            //we could potentially have batter data
            optimAero::ardu_cells curr_cells    = oa_->get_batt_cells(i);

            //this voltage is total amongst all the cells
            //_state.voltage                      = oa_->get_batt_voltage(i);

            // we need to fill this battery type
            switch(_instance_of_OA){
                case 0:{
                    /*need to contain zero - five here for cells 1-6 of batt 1 */
                _state.cell_voltages.cells[0] = curr_cells.cells[0];
                _state.cell_voltages.cells[1] = curr_cells.cells[1];
                _state.cell_voltages.cells[2] = curr_cells.cells[2];
                _state.cell_voltages.cells[3] = curr_cells.cells[3];
                _state.cell_voltages.cells[4] = curr_cells.cells[4];
                _state.cell_voltages.cells[5] = curr_cells.cells[5];
                _state.cell_voltages.cells[6] = curr_cells.cells[6];
                _state.cell_voltages.cells[7] = curr_cells.cells[7];
                _state.cell_voltages.cells[8] = 0;
                _state.cell_voltages.cells[9] = 0;
                
                _state.voltage                = oa_->get_batt_voltage(i,0);
                _state.last_time_micros       = AP_HAL::micros();
                _state.current_amps           = 0;
                _state.healthy                = 1;
    
                }break;

                case 1:{
                /*need to contain 8-13 here for cells 1-6 of batt 2 */
                _state.cell_voltages.cells[0] = curr_cells.cells[8];
                _state.cell_voltages.cells[1] = curr_cells.cells[9];
                _state.cell_voltages.cells[2] = curr_cells.cells[10];
                _state.cell_voltages.cells[3] = curr_cells.cells[11];
                _state.cell_voltages.cells[4] = curr_cells.cells[12];
                _state.cell_voltages.cells[5] = curr_cells.cells[13];
                _state.cell_voltages.cells[6] = curr_cells.cells[14];
                _state.cell_voltages.cells[7] = curr_cells.cells[15];
                _state.cell_voltages.cells[8] = 0;
                _state.cell_voltages.cells[9] = 0;

                _state.voltage                = oa_->get_batt_voltage(i,1);
                _state.last_time_micros       = AP_HAL::micros();
                _state.current_amps           = 0;
                _state.healthy                = 1;                  
                }break;
                
                case 2:{
                /*need to contain 16-21 here for cells 1-6 of batt 3 */
                _state.cell_voltages.cells[0] = curr_cells.cells[16];
                _state.cell_voltages.cells[1] = curr_cells.cells[17];
                _state.cell_voltages.cells[2] = curr_cells.cells[18];
                _state.cell_voltages.cells[3] = curr_cells.cells[19];
                _state.cell_voltages.cells[4] = curr_cells.cells[20];
                _state.cell_voltages.cells[5] = curr_cells.cells[21];
                _state.cell_voltages.cells[6] = curr_cells.cells[22];
                _state.cell_voltages.cells[7] = curr_cells.cells[23];
                _state.cell_voltages.cells[8] = 0;
                _state.cell_voltages.cells[9] = 0;

                _state.voltage                = oa_->get_batt_voltage(i,2);
                _state.last_time_micros       = AP_HAL::micros();    
                _state.current_amps           = 0;
                _state.healthy                = 1;                               
                }
                break;
                case 3:{
                /*need to contain 24-29 here for cells 1-6 of batt 4 */
                _state.cell_voltages.cells[0] = curr_cells.cells[24];
                _state.cell_voltages.cells[1] = curr_cells.cells[25];
                _state.cell_voltages.cells[2] = curr_cells.cells[26];
                _state.cell_voltages.cells[3] = curr_cells.cells[27];
                _state.cell_voltages.cells[4] = curr_cells.cells[28];
                _state.cell_voltages.cells[5] = curr_cells.cells[29];
                _state.cell_voltages.cells[6] = curr_cells.cells[30];
                _state.cell_voltages.cells[7] = curr_cells.cells[31];
                _state.cell_voltages.cells[8] = 0;
                _state.cell_voltages.cells[9] = 0;

                _state.voltage                = oa_->get_batt_voltage(i,3);
                _state.last_time_micros       = AP_HAL::micros();    
                _state.current_amps           = 0;
                _state.healthy                = 1;                        
                }break;
                
                case 4:{
                 /* won't be used unless we have two arduinos need to contain zero - five here for cells 1-6 of batt 1 */
                _state.cell_voltages.cells[0] = curr_cells.cells[32];
                _state.cell_voltages.cells[1] = curr_cells.cells[33];
                _state.cell_voltages.cells[2] = curr_cells.cells[34];
                _state.cell_voltages.cells[3] = curr_cells.cells[35];
                _state.cell_voltages.cells[4] = curr_cells.cells[36];
                _state.cell_voltages.cells[5] = curr_cells.cells[37];
                _state.cell_voltages.cells[6] = curr_cells.cells[38];
                _state.cell_voltages.cells[7] = curr_cells.cells[39];
                _state.cell_voltages.cells[8] = curr_cells.cells[40]; //only going up to 48 cells == NUMCELL*NUMBATT

                _state.voltage                = oa_->get_batt_voltage(i,4);
                _state.last_time_micros       = AP_HAL::micros();    
                _state.current_amps           = 0;
                _state.healthy                = 1;                        
                }break;
                
                default:
                    _state.healthy = 0;
                    _state.voltage = 69;
                    _state.current_amps = 0;
                    _state.last_time_micros = AP_HAL::micros();
                break;
            }
            
        }
    }

    /* this will have to be somewhat smart
    -- this needs to loop thru for multiple monitor instances and fill cells potentially
    -- if one of the oa connections is an arduino 
        (1) look to see if arduino structure battery populated - if so lets grab some cells
         - depending on what monitor instance we are grab (1-10)*instance
         - confirm those cells are healthy, sum the voltage - no current - could grab temperatures too?
    */
}


/*
  msgBatt.cell[0] = convertCellData(flvss1.getCell1());
  msgBatt.cell[1] = convertCellData(flvss1.getCell2());
  msgBatt.cell[2] = convertCellData(flvss1.getCell3());
  msgBatt.cell[3] = convertCellData(flvss1.getCell4());
  msgBatt.cell[4] = convertCellData(flvss1.getCell5());
  msgBatt.cell[5] = convertCellData(flvss1.getCell6());
  //skip 6,7
  msgBatt.cell[8] = convertCellData(flvss2.getCell1());
  msgBatt.cell[9] = convertCellData(flvss2.getCell2());
  msgBatt.cell[10] = convertCellData(flvss2.getCell3());
  msgBatt.cell[11] = convertCellData(flvss2.getCell4());
  msgBatt.cell[12] = convertCellData(flvss2.getCell5());
  msgBatt.cell[13] = convertCellData(flvss2.getCell6());
  //skip 14,15
  msgBatt.cell[16] = convertCellData(flvss3.getCell1());
  msgBatt.cell[17] = convertCellData(flvss3.getCell2());
  msgBatt.cell[18] = convertCellData(flvss3.getCell3());
  msgBatt.cell[19] = convertCellData(flvss3.getCell4());
  msgBatt.cell[20] = convertCellData(flvss3.getCell5());
  msgBatt.cell[21] = convertCellData(flvss3.getCell6());
  //skip 22,23
  msgBatt.cell[24] = convertCellData(flvss4.getCell1());
  msgBatt.cell[25] = convertCellData(flvss4.getCell2());
  msgBatt.cell[26] = convertCellData(flvss4.getCell3());
  msgBatt.cell[27] = convertCellData(flvss4.getCell4());
  msgBatt.cell[28] = convertCellData(flvss4.getCell5());
  msgBatt.cell[29] = convertCellData(flvss4.getCell6());
*/