import traci
from ._config import *

# Dictionary to track platoon memberships
platoon_data = {}  # Key: leader_id, Value: list of vehicles in the platoon

# Dictionary to track each vehicle's current mode
vehicle_modes = {}  # Key: vehicle_id, Value: mode

dedicated_left_lanes = ["1_2", "3_3", "4_2"]

# IDM Parameters (for HDVs)
IDM_DESIRED_SPEED = 30.56  # m/s
IDM_TIME_HEADWAY = 1.5  # s
IDM_MAX_ACCEL = 1.4  # m/s²
IDM_DECEL = 2.0 # m/s²
IDM_MIN_GAP = 2.0  # m

# CACC Parameters (for CAVs)
CACC_FOLLOWING_DISTANCE = 20  # m
CACC_CATCH_UP_DISTANCE = 120  # m
CACC_KP = 0.5  # Proportional gain for speed error
CACC_KG = 0.1 # gap control coefficient
CACC_KD = 0.3  # Derivative gain for acceleration difference
PLATOON_SIZE_THRESHOLD = 9  # Max platoon size

# CACC Parameters (for CAVs)
CACC_PARAMS = {
    "leading_mode": {"time_headway": 1.1, "speed_factor": 1.0},
    "following_mode": {"time_headway": 0.6, "speed_factor": 1.1},
    "lead-catching_mode": {"time_headway": 0.7, "speed_factor": 1.2},
    "catch-up_following_mode": {"time_headway": 0.6, "speed_factor": 1.3},
    "catching_mode": {"time_headway": 0.7, "speed_factor": 1.2},
}

def get_platoon_size(leader_id):
    """Returns the size of the platoon."""
    return len(platoon_data.get(leader_id, [leader_id]))  # Include leader


platoon_history = {}  # key = leader_id, value = { members, start_time, end_time }

def join_platoon(leader_id, vehicle_id):
    sim_time = traci.simulation.getTime()

    # Initialize platoon_data entry if not present
    if leader_id not in platoon_data:
        platoon_data[leader_id] = [leader_id, vehicle_id]
        platoon_history[leader_id] = {
            "members": set([leader_id, vehicle_id]),
            "start_time": sim_time,
            "end_time": sim_time,
        }
        return True

    # Check if platoon is not full
    if len(platoon_data[leader_id]) < PLATOON_SIZE_THRESHOLD:
        platoon_data[leader_id].append(vehicle_id)

        # Initialize history entry if missing
        if leader_id not in platoon_history:
            platoon_history[leader_id] = {
                "members": set(platoon_data[leader_id]),
                "start_time": sim_time,
                "end_time": sim_time,
            }

        # Update history
        platoon_history[leader_id]["members"].add(vehicle_id)
        platoon_history[leader_id]["end_time"] = sim_time
        return True

    return False



def IDM_control(vehicle_id, leader_id, gap, current_speed):
    """Implements IDM acceleration control for non-CAV vehicles."""
    if leader_id and gap is not None:
        leader_speed = traci.vehicle.getSpeed(leader_id)
        delta_speed = current_speed - leader_speed  # Speed difference

        # Compute desired gap
        s_star = IDM_MIN_GAP + max(0, current_speed * IDM_TIME_HEADWAY + (current_speed * delta_speed) / (2 * (IDM_MAX_ACCEL * IDM_DECEL) ** 0.5))

        # Compute acceleration using IDM formula
        if gap < 0.5:
            # print(f"[IDM Warning] {vehicle_id} has very small gap ({gap:.3f} m) to {leader_id}")
            return -IDM_DECEL
        else:
            acceleration = IDM_MAX_ACCEL * (1 - (current_speed / IDM_DESIRED_SPEED) ** 4 - (s_star / gap) ** 2)

        # Apply acceleration limits
        acceleration = max(-IDM_DECEL, min(acceleration, IDM_MAX_ACCEL))
        # print("Applied IDM on ", vehicle_id)
        return acceleration
    else:
        return IDM_MAX_ACCEL  # Free-flow acceleration

def CACC_control(vehicle_id, leader_id, gap, current_speed, mode, min_gap=2.0):
    if leader_id and gap is not None:
        leader_speed = traci.vehicle.getSpeed(leader_id)
        leader_accel = traci.vehicle.getAcceleration(leader_id)

        speed_error = leader_speed - current_speed

        desired_gap = current_speed * CACC_PARAMS[mode]["time_headway"] + min_gap
        gap_error = gap - desired_gap
        acceleration_command = CACC_KP * speed_error + CACC_KG * gap_error + CACC_KD * leader_accel

        if mode in CACC_PARAMS:
            acceleration_command *= CACC_PARAMS[mode]["speed_factor"]  #Applying Speed Factor
            # print("Applied CACC on ", vehicle_id)

        acceleration_command = max(-3.0, min(acceleration_command, 2.0))  # Clip to safe range

        # print(f"Vehicle {vehicle_id} | Gap Error: {gap_error:.2f}, Speed Error: {speed_error:.2f}, Acc Cmd: {acceleration_command:.2f}")

        return acceleration_command
    return 0


def control_func_hybrid(vehicle_id, MPR):
    """Applies IDM or CACC acceleration control to vehicles based on their type."""
    v_type = traci.vehicle.getTypeID(vehicle_id)
    is_cav = "CAV_ori" in v_type
    current_speed = traci.vehicle.getSpeed(vehicle_id)

    # edge_id = traci.vehicle.getRoadID(vehicle_id)
    # lane_count = traci.edge.getLaneNumber(edge_id)
    # position = traci.vehicle.getPosition(vehicle_id)[0]  # x-position on road

    # if is_cav and MPR >= 40:
    #     try:
    #         current_lane = traci.vehicle.getLaneIndex(vehicle_id)
    #         edge_id = traci.vehicle.getRoadID(vehicle_id)
    #         lane_count = traci.edge.getLaneNumber(edge_id)

    #         if current_lane < lane_count - 1:  # not in leftmost
    #             target_lane = current_lane + 1

    #             # Assign duration
    #             if current_lane == 0 and position>=1200:  # right to mid
    #                 duration = 100
    #                 traci.vehicle.changeLane(vehicle_id, target_lane, duration)
    #             # elif current_lane == 1 and position>=1200:  # mid to left
    #             #     duration = 50
    #             #     traci.vehicle.changeLane(vehicle_id, target_lane, duration)

    #     except traci.TraCIException:
    #         pass

    # Get leader information
    leader_info = traci.vehicle.getLeader(vehicle_id)
    leader_id, gap = leader_info if leader_info else (None, None)

    # IDM for Non-CAVs
    if not is_cav:
        current_lane_id = traci.vehicle.getLaneID(vehicle_id)
        # # Enforce hybrid control: kick HDVs out of left lane
        # if current_lane_id in dedicated_left_lanes:
        #     try:
        #         print(f"[Enforce] Moving HDV {vehicle_id} out of {current_lane_id}")
        #         traci.vehicle.changeLane(vehicle_id, 1, 10)  # push to middle lane
        #     except traci.TraCIException:
        #         pass  # vehicle might be mid-transition
        acceleration = IDM_control(vehicle_id, leader_id, gap, current_speed)

        # Apply acceleration change
        traci.vehicle.setAcceleration(vehicle_id, acceleration, 1)
        return

    # CACC for CAVs
    current_mode = vehicle_modes.get(vehicle_id, "default")

    if leader_id and gap is not None:
        leader_type = traci.vehicle.getTypeID(leader_id)

        if "CAV_ori" in leader_type and gap < CACC_CATCH_UP_DISTANCE:
            platoon_size = get_platoon_size(leader_id)

            if gap < CACC_FOLLOWING_DISTANCE:
                current_mode = "following_mode"
                # traci.vehicle.setTau(vehicle_id, 0.6)
                # traci.vehicle.setSpeedFactor(vehicle_id, 1.1)

            elif gap < CACC_CATCH_UP_DISTANCE:
                if platoon_size < PLATOON_SIZE_THRESHOLD:
                    current_mode = "catch-up_following_mode"
                    # traci.vehicle.setTau(vehicle_id, 0.6)
                    # traci.vehicle.setSpeedFactor(vehicle_id, 1.3)  # catch-up_following_mode
                else:
                    current_mode = "catching_mode"
                    # traci.vehicle.setTau(vehicle_id, 0.7)
                    # traci.vehicle.setSpeedFactor(vehicle_id, 1.2)  # catching_mode

            else:
                current_mode = "lead-catching_mode"
                # traci.vehicle.setTau(vehicle_id, 0.7)
                # traci.vehicle.setSpeedFactor(vehicle_id, 1.2) # for lead-catching sf = 1.2

            success = join_platoon(leader_id, vehicle_id) if platoon_size < PLATOON_SIZE_THRESHOLD else False
            if not success:
                current_mode = "leading_mode"
                platoon_data[vehicle_id] = [vehicle_id]
                # traci.vehicle.setTau(vehicle_id, 1.1)
                # traci.vehicle.setSpeedFactor(vehicle_id, 1)

            acceleration = CACC_control(vehicle_id, leader_id, gap, current_speed, current_mode)
            

        else:
            # No front CAV detected within 120m
            current_mode = "leading_mode"
            platoon_data[vehicle_id] = [vehicle_id]
            acceleration = CACC_control(vehicle_id, leader_id, gap, current_speed, current_mode)
            # acceleration = 1  # Maintain speed in leading mode
            # traci.vehicle.setTau(vehicle_id, 1.1)
            # traci.vehicle.setSpeedFactor(vehicle_id, 1)

    else:
        # No leader detected
        current_mode = "leading_mode"
        acceleration = CACC_control(vehicle_id, leader_id, gap, current_speed, current_mode)
        # acceleration = 1  # Maintain speed
        # traci.vehicle.setTau(vehicle_id, 1.1)
        # traci.vehicle.setSpeedFactor(vehicle_id, 1)

    # Store vehicle mode
    vehicle_modes[vehicle_id] = current_mode

    # Apply acceleration
    traci.vehicle.setAcceleration(vehicle_id, acceleration, 1)

    # print(f"Vehicle {vehicle_id} | Mode: {current_mode} | Platoon Size: {get_platoon_size(vehicle_id)} | Gap: {gap} | Accel: {acceleration:.2f} m/s²")

def control_func_baseline(vehicle_id):
    traci.vehicle.setAcceleration(vehicle_id, 1.2, 1)
