# reach_ros_node — vendored into P-AgBot (2026-09-22)

Vendored from a colleague's fork (github.com/kimkt0408/reach_ros_node, itself
from rpng/reach_ros_node), already run on this lab's Reach M2 over USB-Ethernet
TCP. Changes made here, all found before first use on this robot:

- `scripts/nmea_tcp_driver` was checked in as `nmea_tcp_driver.txt` and not
  executable, so `rosrun reach_ros_node nmea_tcp_driver` could not start.
- **RTK float (GGA 5) was reported as RTK fixed (GGA 4)**, both as
  `STATUS_GBAS_FIX`, so a `status >= 2` gate (agbot_gps_nav `min_fix_status:
  2`) accepted a float solution. Now 4 -> 2, 5 and DGPS -> 1, single -> 0.
- A refused connection (Reach still booting) made the node `exit()`; it now
  retries.
- One launch, `launch/reach_gps_fix.launch` (args `host`, `port`, `frame_gps`,
  `fix_topic`), publishing `/gps/fix` in frame `navsat_link` so
  navsat_transform applies the antenna offset. Every NMEA line is now
  `logdebug`, not printed.

⚠ The driver publishes a fix only once it has BOTH a GGA and a GST sentence.
Enable GST in the Reach's NMEA output, or `/gps/fix` never appears.

```bash
sudo ifconfig enxXXXXXXXXXXXX 192.168.2.2 up     # Reach USB-Ethernet interface
roslaunch reach_ros_node reach_gps_fix.launch     # host 192.168.2.15, port 7777
rostopic echo /gps/fix                            # status.status 2 == RTK FIXED
```

The original README follows.

---

# Reach RTK ROS Node

This is a very simple ROS node that allows for publishing of NMEA messages onto the ROS framework.
This package aims to support the [Reach RTK GNSS](https://emlid.com/shop/reach-rtk-kit/) module by Emlid.
Right now this supports all NMEA messages from the package, while some are not used to publish anything onto ROS.

## Quickstart Guide

1. Clone this package into your ROS workspace and build it
2. Turn on your Reach RTK module
3. Ensure that you are connected to the same network as it
4. Edit the "Position output"
   - Be in TCP mode
   - Role: Server
   - Address: localhost
   - Port: any free port
   - Format: NMEA
5. Using the IP of the Reach RTK and the specified port launch this package
6. `rosrun reach_ros_node nmea_tcp_driver _host:=128.4.89.123 _port:=2234`

## Driver Details

- Publishes GPS fix, velocity, and time reference
  - `/gps/fix (/tcpfix)` - NavSatFixed
  - `/tcpvel` - TwistedStamped
  - `/tcptime` - TimeReference
- Can specify the following launch parameters
  - `~frame_timeref` - Frame of the time reference
  - `~frame_gps` - Frame of the fix and velocity
  - `~use_rostime` - If set to true, ROS time is used instead of the GPS time
  - `~use_rmc` - Use compressed RMC message (note: this does not have the covariance for the fix)

## Credit

Original starting point of the driver was the ROS driver [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) which was then expanded by [CearLab](https://github.com/CearLab/nmea_tcp_driver) to work with the Reach RTK.
This package is more complete, and aims to allow for use of the Reach RTK in actual robotic systems, please open a issue if you run into any issues.
Be sure to checkout this other driver by [enwaytech](https://github.com/enwaytech/reach_rs_ros_driver) for the Reach RS.

---

# RTK GPS Utilization with [Emlid Reach M2](https://emlid.com/reach/)

This guide explains how to receive **RTK-corrected GPS data** from the **Emlid Reach M2** and stream it into ROS using a modified version of the [`reach_ros_node`](https://github.com/rpng/reach_ros_node) driver.

> 📚 Reference: [NMEA Format Overview](https://anavs.com/knowledgebase/nmea-format/)

---

## 🔌 Step 0: Verify GPS Is Connected and Streaming

### 🧠 Overview

Ensure your host PC can communicate with the Reach M2 GPS over USB Ethernet (CDC ECM mode) and verify that it's streaming NMEA messages over TCP.

---

### ✅ 1. Identify the GPS USB Ethernet Interface

Use `nmcli` to list available network interfaces:

```bash
nmcli
```

Look for a device labeled something like:

```
enxdeb48762021a: unmanaged
        "Emlid ReachM2"
        ethernet (cdc_ether), DE:B4:87:62:02:1A, hw, mtu 1500
```

This indicates the GPS is detected as a USB Ethernet device (`cdc_ether` driver).

---

### 🛠️ 2. Manually Assign an IP to the Interface

Assign your host PC an IP address in the same subnet (e.g., `192.168.2.2`) using:

```bash
sudo ifconfig enxdeb48762021a 192.168.2.2 up
```

You can confirm it with:

```bash
ifconfig enxdeb48762021a
```

---

### 🧪 3. Ping the Assigned IP (Optional Sanity Check)

To confirm the interface is active:

```bash
ping 192.168.2.2
```

Expected output:

```
64 bytes from 192.168.2.2: icmp_seq=1 ttl=64 time=0.030 ms
```

---

### 🌐 4. Verify GPS NMEA Stream via Telnet

Once connected, check that the GPS is streaming NMEA data via TCP:

```bash
telnet 192.168.2.15 7777
```

- `192.168.2.15`: Default IP of Reach M2 in USB CDC mode
- `7777`: Port number for Position Output (set in **ReachView → Position Output → TCP Server**)

You should see continuous NMEA sentences like:

```
$GNGGA,...
$GNRMC,...
```

---

Let me know if you'd like to automate this setup or make the IP assignment persistent.

---

## 🧩 Step 1: Clone and Build the ROS Driver

We use a slightly modified fork of the original `reach_ros_node`:

```bash
git clone https://github.com/rpng/reach_ros_node.git
```

> ✏️ I applied minor fixes to support USB-over-TCP usage. You can find my working version here:  
> [kimkt0408/pagbot/source/reach_ros_node](https://github.com/kimkt0408/reach_ros_node.git)

Then build the package:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 Step 2: Run the Driver in ROS

1. **Connect the Reach M2 to your PC (Clearpath Jackal) via USB**
2. **Assign a static IP on your USB network interface**:

```bash
sudo ifconfig enxXXXX up 192.168.2.2
```

3. **Launch the driver**:

```bash
rosrun reach_ros_node nmea_tcp_driver _host:=192.168.2.15 _port:=7777
```

---

## 📡 Step 3: View the GPS Data in ROS

After starting the driver, check that the topic is publishing:

```bash
rostopic list
```

You should see:

```
/tcpfix (/gps/fix) # With roslaunch, remapped /tcpfix to /gps/fix for further usages
```

To inspect the GPS data:

```bash
rostopic echo /tcpfix # or

rostopic echo /gps/fix
```

This topic is of type `sensor_msgs/NavSatFix`.

> 🟡 Note: If the NMEA messages appear empty, the Reach M2 may not yet have a GNSS fix. Make sure it has a clear sky view and a valid RTK correction source.

---

## 📂 Optional: Create a Launch File

To avoid retyping parameters, create a launch file like this:

```xml
<!-- File: launch/reach_gps_fix.launch -->
<launch>
  <node pkg="reach_ros_node" type="nmea_tcp_driver" name="reach_ros_node" output="screen">
    <param name="host" value="192.168.2.15" />
    <param name="port" value="7777" />
    <remap from="/tcpfix" to="/gps/fix" />
  </node>
</launch>
```

Run it with:

```bash
roslaunch reach_ros_node reach_gps_fix.launch
```

---

Let me know if you'd like to add:

- Integration with `robot_localization`
- Republish to a `tf` frame (`base_link`, `gps_link`)
- Logging tools or GPS fix monitoring
