"""lidar_tf.launch (real robot) must match agbot_lidar.urdf.xacro (sim/RViz).

The two are separate copies of one number -- where the Velodyne sits above
base_link -- because the real robot does not load our xacro. This builds the
URDF exactly as agbot_gazebo.launch does (lidar:=true) and compares.
Skipped where xacro / the Jackal description are not available.
"""
import math
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
LOADER = os.path.join(SRC, "agbot_bringup", "scripts", "load_robot_description.sh")
EXTRAS = os.path.join(SRC, "agbot_bringup", "urdf", "agbot_camera.urdf.xacro")
JACKAL = os.path.join(SRC, "jackal", "jackal_description", "urdf", "jackal.urdf.xacro")
TF_LAUNCH = os.path.join(SRC, "agbot_nav", "launch", "lidar_tf.launch")


def _launch_offset():
    root = ET.parse(TF_LAUNCH).getroot()
    node = root.find("node[@name='lidar_tf']")
    args = node.get("args").split()
    xyz, ypr, parent = args[0:3], args[3:6], args[6]
    return [float(v) for v in xyz], [float(v) for v in ypr], parent


def _xacro_offset():
    if shutil.which("xacro") is None or not os.path.exists(JACKAL):
        pytest.skip("xacro or jackal_description not available")
    result = subprocess.run(
        ["bash", LOADER, EXTRAS, JACKAL, "none", "lidar:=true"],
        capture_output=True, text=True, timeout=60)
    if result.returncode != 0:
        # Skip ONLY when ROS's xacro is not importable; anything else is a
        # broken URDF and must fail. (A '--' inside an XML comment in the
        # lidar xacro was once reported here as a quiet skip.)
        if "No module named 'xacro'" in result.stderr:
            pytest.skip("ROS python path not sourced (no xacro module)")
        pytest.fail("xacro failed on our URDF:\n" + result.stderr[-1500:])
    urdf = result.stdout
    joints = {}
    for joint in ET.fromstring(urdf).findall("joint"):
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None
                                  else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None
                                  else "0 0 0").split()]
        joints[joint.find("child").get("link")] = (joint.find("parent").get("link"), xyz, rpy)
    # Compose from base_link down. Only yaw is allowed on this chain (the
    # unit faces backwards); roll/pitch would need full 3D composition.
    chain, link = [], "velodyne1"
    while link != "base_link":
        parent, xyz, rpy = joints[link]
        assert abs(rpy[0]) < 1e-9 and abs(rpy[1]) < 1e-9, "roll/pitch on the lidar chain at %s" % link
        chain.append((xyz, rpy[2]))
        link = parent
    x = y = z = yaw = 0.0
    for (dx, dy, dz), dyaw in reversed(chain):
        x += dx * math.cos(yaw) - dy * math.sin(yaw)
        y += dx * math.sin(yaw) + dy * math.cos(yaw)
        z += dz
        yaw += dyaw
    return [x, y, z], math.atan2(math.sin(yaw), math.cos(yaw))


def test_static_tf_matches_the_urdf():
    launch_xyz, launch_ypr, parent = _launch_offset()
    assert parent == "base_link"
    assert launch_ypr[1:] == [0.0, 0.0]          # static_transform_publisher: yaw pitch roll
    urdf_xyz, urdf_yaw = _xacro_offset()
    for got, want in zip(launch_xyz, urdf_xyz):
        assert got == pytest.approx(want, abs=1e-3), (launch_xyz, urdf_xyz)
    d = launch_ypr[0] - urdf_yaw
    assert math.atan2(math.sin(d), math.cos(d)) == pytest.approx(0.0, abs=1e-3)


def test_launch_args_are_numbers_then_frames():
    root = ET.parse(TF_LAUNCH).getroot()
    args = root.find("node[@name='lidar_tf']").get("args")
    # x y z yaw pitch roll parent child; the child is a $(arg ...) substitution
    head, child = args.split("base_link")
    assert all(re.match(r"^-?[0-9.]+$", a) for a in head.split()) and len(head.split()) == 6
    assert child.strip() == "$(arg lidar_frame)"
