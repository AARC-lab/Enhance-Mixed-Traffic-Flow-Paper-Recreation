Network file description:
edge_id = 1: This edge starts from 0m to 2000m where there are 3 lanes
This edge contains 3 lane with IDs as follows:
[right, mid, left] = [1_0, 1_1, 1_2]

edge_id = 3: This edge starts from 2000m to 2250m where there are 4 lanes with the merging ramp lane
This edge contains 4 lane with IDs as follows:
[merged_lane, right, mid, left] = [3_0, 3_1, 3_2, 3_3]

edge_id = 4: This edge starts from 2250m to 3250m where there are 3 lanes  
This edge contains 3 lane with IDs as follows:
[right, mid, left] = [4_0, 4_1, 4_2]

edge_id = 2: This edge contains ramp with id 2_0  

There are two connecting edge with IDs 0_1 and 1_0.
Edge ID 0_1: [right, mid, left] = [0_1_0, 0_1_1, 0_1_2]
Edge ID 1_0: [merged_lane, right, mid, left] = [1_0_0, 1_0_1, 1_0_2, 1_0_3]

ramp_lane id = :0_0_0
left_lane_id = [1_2, 3_3, 4_2, 0_1_2, 1_0_3]
mid_lane_id = [1_1, 3_2, 4_1, 0_1_1, 1_0_2]
right_lane_id = [1_0, 3_1, 4_0, 0_1_0, 1_0_1]

This work is the modified version of the original work that can be found at: https://github.com/CASUUU/SUMO-BeginnerControl.git
Paper title: Enhancing Mixed Traffic Flow with Platoon Control and Lane Management for Connected and Autonomous Vehicles
