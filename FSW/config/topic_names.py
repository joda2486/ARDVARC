# Topic names
ANNOTATED_CAMERA_FRAMES = "camera/annotated_frames"
CAMERA_FRAMES = "camera/frames"
UAS_POSES = "mavros/local_position/pose"
RECENT_RGV_SIGHTINGS = "camera/recent_rgv_sightings"
RAW_BLUETOOTH = "bluetooth/az_els"
UAS_TO_RGV_DIRECTION_VECTORS = "estimation/direction_vectors_uas"
ESTIMATED_RGV_STATES = "estimation/estimated_rgv_states"
MISSION_STATES = "state_machine/mission_states"
STATE_MACHINE_CRITERIA = "state_machine/state_machine_criteria"
#TODO(LF): what is the SETPOINTS for?
SETPOINTS = "pixhawk/setpoints"
REGIONS_OF_INTEREST = "pixhawk/regions_of_interest"

BATTERY = "mavros/battery"

UAS_ARMING_STATE = "mavros/state"
UAS_SETPOINT_LOCAL = "mavros/setpoint_position/local"

# Lyon used this for testing
MAVROS_GPS_POS_FORTESTING = "mavros/global_position/global"

# clients/services
CLIENT_ARMING = "/mavros/cmd/arming"
CLIENT_SET_MODE = "/mavros/set_mode"
