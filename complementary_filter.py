POSITION_FILTER_RATIO = 0.1 
HEADING_FILTER_RATIO = 0.01 

def merge(odo_pose, localised_pose):
    odo_x = odo_pose[0]
    odo_y = odo_pose[1]
    localised_x = localised_pose[0]
    localised_y = localised_pose[1]
    merged_x = odo_x * (1 - POSITION_FILTER_RATIO) + localised_x * POSITION_FILTER_RATIO
    merged_y = odo_y * (1 - POSITION_FILTER_RATIO) + localised_y * POSITION_FILTER_RATIO

    odo_z = odo_pose[2]
    localised_z = localised_pose[2]
    merged_angle = odo_z * (1 - HEADING_FILTER_RATIO) + localised_z * HEADING_FILTER_RATIO
    return [merged_x, merged_y, merged_angle]
    