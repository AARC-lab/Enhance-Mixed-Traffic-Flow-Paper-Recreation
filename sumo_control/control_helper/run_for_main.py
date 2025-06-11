import traci
import sys
from .control import *
from .data_coll import *


interrupt_timestamp = 500

def run(base_setting_para):  # Function to execute the defined operations
    step = 0
    MPR = base_setting_para[2]
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # left_lane_ids = ["1_2", "3_3", "4_2"]  # leftmost main road lanes
        # for lane_id in left_lane_ids:
        #     try:
        #         traci.lane.setAllowed(lane_id, ["passenger"])
        #         # print(f"[Hybrid] Restricted {lane_id} to CAV_ori")
        #     except Exception as e:
        #         print(f"[Hybrid] Error restricting {lane_id}: {e}")
        # Basic information in this simulation step
        vehicle_ids = traci.vehicle.getIDList()
        time_stamp = int(step * 10) / 10
        # Control each vehicle in sequence
        for vehicle_id in vehicle_ids:
            control_func_hybrid(vehicle_id, MPR)
            if time_stamp % 1 == 0:
                data_coll_vi_ti(vehicle_id, time_stamp)
        if time_stamp % 1 == 0:
            data_coll_ti(vehicle_ids)
        # Periodic summary of data
        ### - Accompanied by data output and saving -
        if time_stamp == interrupt_timestamp:
            data_coll_t_check(base_setting_para, interrupt_timestamp, time_stamp)
            break
        
        
        step += 0.1
    traci.close()
    sys.stdout.flush()