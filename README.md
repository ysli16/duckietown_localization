## Overview
This repo contains 3 packages for duckiebot localization.

**encoder_localization** package utilizes wheel encoders to estimate current pose of the duckiebot. The initial pose is assumed to be (0.15,0) with rotation angle pi in the map frame. It subscribes to both wheel encoders' tick and publish the estimated transformation between baselink frame and map frame to topic `encoder_localization_node/transform`. A service called `encoder_localization_node/frame_calibration` is provided that uses the given transform to calibrate the estimated encoder frame transform. This service is called by the `fused_localization_node`.

**at_localization** package utilizes Apriltag to estimate current pose of the duckiebot. The position of the Apriltag is assumed to be at the origin of map frame. It subscribes to camera images and publish the estimated transformation between baselink frame and map frame to topic `at_localization_node/transform`.

**fused_localization** package provides a fused estimation of duckiebot pose. It subscribes to both encoder_localization and at_localization transformations mentioned above and publish the estimated transformation between baselink frame and map frame to topic `fused_localization_node/transform`. The first time the Apriltag becomes visible, it calls service `encoder_localization_node/frame_calibration` to calledcalibrate the encoder_baselink frame to match exactly the at_localization transformation. When Apriltag estimation is available, fused_localization completely relays on at_localization and projects at_localization results on the ground plane. When Apriltag is not visible, encoder estimation is used, starting from the last Apriltag pose received.

## Usage

### encoder_localization package
Edit `default.sh` inside `launchers` folder. Commented every lines start with `dt-exec` except

`dt-exec roslaunch encoder_localization encoder_localization_node.launch veh:="$VEHICLE_NAME"`

Then run command

`dts devel build -H <duckiebot_name>.local -f`

Replace `<duckiebot_name>` with your duckiebot name.

To run the image, use command

`dts devel run -H <duckiebot_name>.local`

To visualize frame transformations, run command in a new terminal

`dts start_gui_tools <duckiebot_name>`

Replace `<duckiebot_name>` with your duckiebot name.

Then run command `rviz` and select `TF` and `image` in the opening window.

### at_localization package
Edit `default.sh` inside `launchers` folder. Commented every lines start with `dt-exec` except

`dt-exec roslaunch at_localization at_localization_node.launch veh:="$VEHICLE_NAME"`

Then run command

`dts devel build -H <duckiebot_name>.local -f`

Replace `<duckiebot_name>` with your duckiebot name.

To run the image, use command

`dts devel run -H <duckiebot_name>.local`

To visualize frame transformations, run command in a new terminal

`dts start_gui_tools <duckiebot_name>`

Replace `<duckiebot_name>` with your duckiebot name.

Then run command `rviz` and select `TF` and `image` in the opening window.

### fused_localization package
Edit `default.sh` inside `launchers` folder. Commented every lines start with `dt-exec` except

`dt-exec roslaunch fused_localization fused_localization_node.launch veh:="$VEHICLE_NAME"`

Then run command

`dts devel build -H <duckiebot_name>.local -f`

Replace `<duckiebot_name>` with your duckiebot name.

To run the image, use command

`dts devel run -H <duckiebot_name>.local`

To visualize frame transformations, run command in a new terminal

`dts start_gui_tools <duckiebot_name>`

Replace `<duckiebot_name>` with your duckiebot name.

Then run command `rviz` and select `TF` and `image` in the opening window.
