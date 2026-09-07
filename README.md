# P-AgBot Control Workspace

Purdue P-AgBot: vision-based row-following and multi-row mission navigation for a
Clearpath Jackal UGV, plus a Gazebo simulation bringup.

Packages: `agbot_bringup` (simulation launch), `agbot_vision_nav` (row-centering
controller, mission FSM, operator panel), `agbot_gps_nav` (GPS waypoint
navigation for open-sky transit from the trailer to a row entrance).

### Note:

Make sure to always source before running ROS commands:

```jsx
cd agbot_control_ws/ && source devel/setup.bash
```

Also, this repo assumes that the robot being used has a front and a back camera. The front camera is the fisheye camera, while the back camera is the Logitech Brio. If the device being used is not the same, the device path needs to be changed inside `agbot_vision_nav/launch/cameras.launch` or passsed as a parameter when starting the camera node.

- Find camera path:

    ```
    ls -l /dev/v4l/by-id/          # find the -video-index0 link
    ```

    Always pick the `...-video-index0` link (each camera exposes several `/dev/video*` nodes; only index0 is the video stream).

- Pass as a parameter when launching the camera:

    ```jsx
    roslaunch agbot_vision_nav cameras.launch front_device:=/dev/v4l/by-id/<new-brio-serial>-video-index0
    ```

### Real Robot Testing:

- Start camera:

    ```jsx
    roslaunch agbot_vision_nav cameras.launch
    ```

- Start navigation (specify number of rows to navigate):

    ```jsx
    roslaunch agbot_vision_nav vision_nav.launch mission_enabled:=true num_rows:=3
    ```

- Enable rear camera:

    ```jsx
    roslaunch agbot_vision_nav vision_nav.launch mission_enabled:=true num_rows:=3 rear_camera_enabled:=true
    ```

- Visualize the segmentation:

    ```jsx
    rqt_image_view /vision_nav_node/debug/image
    ```

- Record ros bags of the camera (rear optional):

    ```jsx
    rosbag record /usb_cam/image_raw/compressed /brio_rear/image_raw/compressed
    ```

- Record rosbags of other topics:

    ```jsx
    rosbag record -O robot_info.bag /cmd_vel /odometry/filtered /vision_nav_node/debug/image
    ```

#### Run in simulation:

- Launch the robot inside Gazebo and Rviz

    ```jsx
    roslaunch agbot_bringup agbot_gazebo.launch
    ```

- Testing with real cmd_vel:

    ```jsx
    roslaunch agbot_vision_nav vision_nav.launch sim:=true
    ```

- View segmentation:

    ```jsx
    rqt_image_view /vision_nav_node/debug/image
    ```

- Run the mission mode:

    ```jsx
    roslaunch agbot_vision_nav vision_nav.launch sim:=true mission_enabled:=true num_rows:=3
    ```

- Enable rear camera:

    ```jsx
    roslaunch agbot_vision_nav vision_nav.launch sim:=true mission_enabled:=true rear_camera_enabled:=true num_rows:=2
    ```

### Commands

- Access the robot (-X is for GUI, not required for all laptops):

    ```jsx
    ssh -X administrator@192.168.0.179
    ```

- Copy things into/out of the robot:
    - sftp:

        ```jsx
        sftp administrator@192.168.0.179
        ```

        ```jsx
        get 2026-07-07-09-03-49.bag /home/weichien241/ag_bot/src/videos
        ```

    - scp:
        - From laptop to Jackal:

            ```jsx
            scp frame_000450.jpg administrator@192.168.0.179:~/agbot_control_ws/src/
            ```

        - From Jackal to laptop:

            ```jsx
            ! scp administrator@192.168.0.179:agbot_control_ws/src/tmp/segmentation_overlay.jpg /home/chien21/agbot_control_ws/src/tmp/
            ```

- Others:
    - Verify the cameras and odometry

        ```jsx
        rostopic hz /usb_cam/image_raw/compressed /brio_rear/image_raw/compressed
        rostopic hz /odometry/filtered
        ```

    - Testing if torch Exists

        ```jsx
        source ~/agbot_venv/bin/activate
        source ~/agbot_control_ws/devel/setup.bash
        python3 -c "import torch, rospy, cv_bridge; print('all imports OK', torch.__version__)"
        ```

#### GPS waypoint navigation in simulation:

A blank Gazebo world with a simulated GNSS receiver — give the robot a point and
it drives there and stops. No maize, no camera, so it loads in seconds.

- Start the world (add `rviz:=true` to click goals):

    ```jsx
    roslaunch agbot_bringup agbot_gps_sim.launch rviz:=true
    ```

- Start GPS navigation:

    ```jsx
    roslaunch agbot_gps_nav gps_nav.launch sim:=true
    ```

- Send a goal, either by clicking **2D Nav Goal** in RViz (its Fixed Frame must
  be `map`), or as a latitude/longitude — note `x` is the LONGITUDE:

    ```jsx
    rostopic pub -1 /gps_nav_node/goal_wgs84 geometry_msgs/PointStamped \
      '{header: {frame_id: wgs84}, point: {x: -86.9910, y: 40.4695}}'
    ```

- Watch what it is doing:

    ```jsx
    rostopic echo /gps_nav_node/status
    ```

The field origin lives in `agbot_gps_nav/config/gps_datum.yaml` and is currently
a placeholder — replace it with surveyed coordinates before saving any real
waypoint.
