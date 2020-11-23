#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#dt-exec echo "This is an empty launch script. Update it to launch your application."
#dt-exec roslaunch encoder_localization encoder_localization_node.launch veh:="$VEHICLE_NAME"
#dt-exec roslaunch at_localization at_localization_node.launch veh:="$VEHICLE_NAME"
dt-exec roslaunch fused_localization fused_localization_node.launch veh:="$VEHICLE_NAME"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
