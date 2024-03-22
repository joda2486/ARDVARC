"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    main loop for all fsw

Notes:
    - Some things we NEVER want to do:
     + quit the program
     + do noting unintentionally
     + error out and quit the program
     + get stuck in a loop
"""

# Do rospy first so that the node is ready when other files are imported
import rospy

# Make node for everything else
rospy.init_node("everything_else")

# local imports
from .functional import determine_mission_state, guidance, estimate_rgv_state, generate_state_machine_criteria, process_bluetooth

# third part imports
import numpy as np

# stl imports
import time

# Call all the setups
determine_mission_state.setup()
guidance.setup()
estimate_rgv_state.setup()
generate_state_machine_criteria.setup()
process_bluetooth.setup()

# Run the estimation loop until we die
rate = rospy.Rate(3)
now = rospy.Time.now()
while not rospy.is_shutdown():
    estimate_rgv_state.publish_estimated_rgv_state()
    rate.sleep()
