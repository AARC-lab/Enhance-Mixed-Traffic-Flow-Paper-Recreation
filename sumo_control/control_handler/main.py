import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import optparse
import xml.etree.ElementTree as ET
import traci  # noqa
import pandas as pd
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
from sumolib import checkBinary  # noqa
from control_helper._generation_files import *
from control_helper.run_for_main import *


multi_index_0 = 12 * 3                          # Main road traffic    unit: 100 vehicles
multi_index_1 = 8                               # Ramp traffic         unit: 100 vehicles
initial_speed_0 = 110 / 3.6                     # Initial speed on main road, 110 km/h
initial_speed_1 = 80 / 3.6                      # Initial speed on ramp, 80 km/h
MPR = 100                                         # MPR of CAV
without_ramp = False

base_setting_para = [multi_index_0, multi_index_1, MPR, without_ramp]
speed_setting = [round(initial_speed_0, 2), round(initial_speed_1, 2)]


# This section is standard and does not need modification
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of SUMO")
    optParser.add_option("--gui", action="store_true", default=False, help="run SUMO with GUI")
    options, args = optParser.parse_args()
    return options

# This part is mostly standard
if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    sumoBinary = checkBinary('sumo-gui')

    if not without_ramp:
        generate_routefile(multi_index_0, multi_index_1, MPR)                     # Generate traffic
        generate_roadnet_file_func(speed_setting)
        traci.start([sumoBinary, "-c", "/home/ruby/Nazmus Shakib/AARC Lab/Paper Recreation: Enhance Mix Traffic Flow/deployment/environment/main_config.sumocfg"])
        # if MPR >= 60:
        #     print("High MPR detected. Applying CAV-only restriction on left lanes.")

        #     left_lane_ids = ["1_2", "3_3", "4_2", "0_1_2", "1_0_3"]
        #     for lane_id in left_lane_ids:
        #         try:
        #             traci.lane.setAllowed(lane_id, ["CAV_ori"])
        #             print(f"[Applied] {lane_id} now restricted to CAV_ori only.")
        #         except Exception as e:
        #             print(f"[Error] Could not restrict {lane_id}: {e}")

    if without_ramp:
        generate_routefile_without_ramp(multi_index_0, multi_index_1, MPR)        # Generate traffic
        generate_roadnet_without_ramp_file_func(speed_setting)
        traci.start([sumoBinary, "-c", "/home/ruby/Nazmus Shakib/AARC Lab/Paper Recreation: Enhance Mix Traffic Flow/deployment/environment/main_config_without_ramp.sumocfg"])

    traci.setLegacyGetLeader(True)

    #################################################
    run(base_setting_para)                                    # Execute function
