#!/usr/bin/env python3
"""PyQt operator panel for field testing: launch, pause, resume, stop.

Why this exists. Field testing meant remembering two roslaunch command lines
and their arguments, and the only way to stop the robot was Ctrl-C on the
node -- which does not pause a mission, it destroys it. rows_driven, the
boustrophedon turn direction, the exit detector's arming distance and the
row-entry pose all live in the node, so a mid-run stop meant restarting the
whole mission from row 1. This panel drives the node's ~pause service
instead, which freezes the state machine and leaves all of that intact.

On the real robot it starts everything the run needs: the USB cameras and the
vision-nav node. In SIMULATION it starts only the vision-nav node -- Gazebo is
launched from its own terminal (`roslaunch agbot_bringup agbot_gazebo.launch`)
and the frame-source button is disabled, because Gazebo's output would
otherwise bury the vision-nav startup config block in this terminal, and that
block is the part worth reading. roslaunch's own output still goes to the
terminal this was started from -- that is where the startup config block and
the detector lines appear.

What it is NOT: a monitoring tool. The debug image
(rqt_image_view /vision_nav_node/debug/image) and that console are still where
you read what the robot is thinking. This panel starts things and stops them.

Design notes:
  - roslaunch runs as a CHILD PROCESS, not as an API call. roslaunch's Python
    API wants to own the process and does not survive being embedded in a Qt
    event loop; a subprocess is also what lets Stop actually kill the whole
    process group, node and camera drivers included.
  - Empty fields are NOT passed. That is the repo's config contract: params
    that are not overridden come from config/params.yaml. Passing an empty or
    defaulted value for everything would silently re-pin every knob at panel
    defaults, which is the exact failure this repo spent a session unpicking
    in 2026-07-30. Type a value only when you mean to override it.
  - rospy callbacks do not touch Qt widgets. The status subscriber writes to a
    plain attribute and a QTimer paints it, because Qt widgets may only be
    touched from the thread that owns them.

Run it standalone (it does not need to be in the same launch file):

    rosrun agbot_vision_nav operator_panel.py
"""

import os
import signal
import subprocess
import sys
import time

import rosgraph
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from agbot_vision_nav.launch_args import cameras_launch_args, mission_launch_args

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

NODE_NAMESPACE = "/vision_nav_node"


class LaunchProcess(object):
    """One roslaunch child process, killed as a group.

    roslaunch spawns the nodes as children of itself, so killing only the
    roslaunch PID orphans them -- the camera stays open, the next launch fails
    with a device-busy error that reads like a USB problem, and the old node
    keeps publishing cmd_vel. start_new_session puts the whole tree in its own
    process group so one signal reaches all of it.
    """

    def __init__(self, name):
        self.name = name
        self._proc = None

    def running(self):
        return self._proc is not None and self._proc.poll() is None

    def start(self, args):
        if self.running():
            return False, "%s is already running" % self.name
        cmd = ["roslaunch"] + args
        try:
            self._proc = subprocess.Popen(cmd, start_new_session=True)
        except OSError as exc:
            return False, "could not start %s: %s" % (self.name, exc)
        return True, " ".join(cmd)

    def stop(self, grace=10.0, straggler_grace=3.0):
        if not self.running():
            return False
        try:
            pgid = os.getpgid(self._proc.pid)
        except OSError:
            self._proc = None
            return True
        # SIGINT, not SIGKILL: roslaunch shuts its nodes down cleanly on
        # SIGINT, which is what makes the node write its metrics CSV summary
        # and publish a final zero Twist. SIGKILL would skip both.
        try:
            os.killpg(pgid, signal.SIGINT)
            self._proc.wait(timeout=grace)
        except (subprocess.TimeoutExpired, OSError):
            pass
        # ⚠ roslaunch exiting does NOT mean its children are gone, and waiting
        # on it is therefore not enough. Gazebo classic routinely outlives its
        # roslaunch: gzserver/gzclient block on a Fuel model download during
        # teardown (`libcurl: (28) Failed to connect to
        # fuel.ignitionrobotics.org`, ~2 min of TCP timeout) long after every
        # node has logged "killing on exit". Observed 2026-08-06: Stop looked
        # like it did nothing because the escalation below only ran on a
        # roslaunch TIMEOUT, and roslaunch had exited perfectly cleanly.
        # A leftover gzserver holds the world, the ports and /use_sim_time, so
        # the next Start comes up against a half-dead simulator.
        self._reap_group(pgid, straggler_grace)
        self._proc = None
        return True

    @staticmethod
    def _reap_group(pgid, grace):
        """Escalate TERM -> KILL until the process group is empty."""
        for sig, wait_for in ((signal.SIGTERM, grace), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(pgid, sig)
            except OSError:
                return  # group already empty; nothing outlived roslaunch
            deadline = time.time() + wait_for
            while time.time() < deadline:
                try:
                    os.killpg(pgid, 0)  # signal 0 = existence probe
                except OSError:
                    return
                time.sleep(0.2)


class OperatorPanel(QWidget):
    def __init__(self):
        super(OperatorPanel, self).__init__()
        self.setWindowTitle("AgBot vision-nav operator panel")

        self._cameras = LaunchProcess("cameras")
        self._mission = LaunchProcess("vision_nav")
        self._status_text = "(no status yet)"
        self._paused = False

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_cameras_box())
        layout.addWidget(self._build_mission_box())
        layout.addWidget(self._build_control_box())

        self._status_label = QLabel(self._status_text)
        self._status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._status_label.setWordWrap(True)
        layout.addWidget(self._status_label)

        self._log_label = QLabel("")
        self._log_label.setWordWrap(True)
        layout.addWidget(self._log_label)

        rospy.Subscriber(
            NODE_NAMESPACE + "/status", String, self._status_cb, queue_size=1
        )
        timer = QTimer(self)
        timer.timeout.connect(self._refresh)
        timer.start(500)
        self._timer = timer

    # ------------------------------------------------------------- layout --
    def _build_cameras_box(self):
        box = QGroupBox("1. Frame source")
        row = QHBoxLayout(box)
        self._cameras_btn = QPushButton("Start cameras")
        self._cameras_btn.clicked.connect(self._toggle_cameras)
        row.addWidget(self._cameras_btn)
        self._cameras_label = QLabel("")
        self._cameras_label.setWordWrap(True)
        row.addWidget(self._cameras_label)
        return box

    def _build_mission_box(self):
        box = QGroupBox("2. Mission")
        form = QFormLayout(box)

        self._model_path = QLineEdit()
        self._model_path.setPlaceholderText(
            "blank = config/exported_best.pt in the package"
        )
        form.addRow("model_path", self._model_path)

        self._sim = QCheckBox("simulation (Gazebo camera topics)")
        form.addRow("", self._sim)

        self._mission_enabled = QCheckBox("multi-row mission (headland turns)")
        self._mission_enabled.setChecked(True)
        form.addRow("", self._mission_enabled)

        self._rear_enabled = QCheckBox("rear camera (back-out + rear-steered exit)")
        self._rear_enabled.setChecked(True)
        form.addRow("", self._rear_enabled)

        self._num_rows = QLineEdit()
        self._num_rows.setPlaceholderText("3   (0 = until no rows left)")
        form.addRow("num_rows", self._num_rows)

        self._first_turn = QComboBox()
        # "" first so the default is "don't pass it" -- params.yaml decides.
        self._first_turn.addItems(["", "left", "right"])
        form.addRow("first_turn_direction", self._first_turn)

        self._linear_x = QLineEdit()
        self._linear_x.setPlaceholderText("blank = params.yaml (0.15 m/s)")
        form.addRow("linear_x_cruise", self._linear_x)

        self._angular_z = QLineEdit()
        self._angular_z.setPlaceholderText("blank = params.yaml (0.175 rad/s)")
        form.addRow("angular_z_max", self._angular_z)

        hint = QLabel(
            "Blank fields are not passed, so config/params.yaml decides them.\n"
            "Fill one in only to override it for this run."
        )
        hint.setWordWrap(True)
        form.addRow("", hint)

        self._mission_btn = QPushButton("Start mission")
        self._mission_btn.clicked.connect(self._toggle_mission)
        form.addRow("", self._mission_btn)
        return box

    def _build_control_box(self):
        box = QGroupBox("3. Control")
        row = QHBoxLayout(box)
        self._pause_btn = QPushButton("Pause")
        self._pause_btn.clicked.connect(self._toggle_pause)
        row.addWidget(self._pause_btn)
        note = QLabel(
            "Pause stops the robot but KEEPS the mission state "
            "(row count, turn direction, position in the row)."
        )
        note.setWordWrap(True)
        row.addWidget(note)
        return box

    # ------------------------------------------------------------ actions --
    def _toggle_cameras(self):
        if self._cameras.running():
            self._stop_with_feedback(self._cameras, "frame source")
            return
        # Real-robot USB bringup only. In simulation this button is disabled
        # (see _refresh) because Gazebo belongs in its own terminal.
        ok, message = self._cameras.start(cameras_launch_args())
        self._log(message)

    def _mission_args(self):
        # The blank-means-do-not-pass rule lives in launch_args.py so it can
        # be unit-tested without Qt or ROS. See test_launch_args.py.
        return mission_launch_args(
            self._model_path.text(),
            sim=self._sim.isChecked(),
            mission_enabled=self._mission_enabled.isChecked(),
            rear_camera_enabled=self._rear_enabled.isChecked(),
            num_rows=self._num_rows.text(),
            first_turn_direction=self._first_turn.currentText(),
            linear_x_cruise=self._linear_x.text(),
            angular_z_max=self._angular_z.text(),
        )

    def _toggle_mission(self):
        if self._mission.running():
            self._stop_with_feedback(self._mission, "mission")
            return
        try:
            args = self._mission_args()
        except ValueError as exc:
            self._log(str(exc))
            return
        ok, message = self._mission.start(args)
        self._log(message)

    def _stop_with_feedback(self, launch, label):
        """Shut a launch down, keeping the window painted while it happens.

        Stopping blocks for as long as the teardown takes, and with Gazebo
        that is routinely several seconds (see LaunchProcess.stop). Without
        this the window greys out and looks hung at exactly the moment the
        operator is trying to stop a moving robot -- which invites a second
        click, or a panic Ctrl-C that leaves the stragglers behind.
        """
        self._log("stopping %s ..." % label)
        self._cameras_btn.setEnabled(False)
        self._mission_btn.setEnabled(False)
        QApplication.processEvents()
        try:
            launch.stop()
        finally:
            # _refresh, not setEnabled(True): the frame-source button is also
            # disabled by the simulation checkbox, and re-enabling it blindly
            # here would hand back a button that must stay greyed out.
            self._mission_btn.setEnabled(True)
            self._refresh()
        self._log("%s stopped" % label)

    def _toggle_pause(self):
        target = not self._paused
        try:
            rospy.wait_for_service(NODE_NAMESPACE + "/pause", timeout=2.0)
            proxy = rospy.ServiceProxy(NODE_NAMESPACE + "/pause", SetBool)
            response = proxy(target)
            self._log("pause -> %s" % response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self._log("pause failed (is the node running?): %s" % exc)

    # -------------------------------------------------------------- status --
    def _status_cb(self, msg):
        # rospy thread: touch only plain attributes, never widgets.
        self._status_text = msg.data
        self._paused = msg.data.startswith("paused")

    def _log(self, text):
        rospy.loginfo("operator_panel: %s", text)
        self._log_label.setText(text)

    def _refresh(self):
        if rospy.is_shutdown():
            self.close()
            return
        # In simulation there is nothing for this button to start: Gazebo is
        # run from its own terminal on purpose (its output would otherwise
        # bury the vision-nav startup config block), and the URDF cameras come
        # up with it. Disabled rather than hidden, so the reason is visible.
        # (Still enabled if a real-camera launch is somehow running, so ticking
        # the box mid-session can never strand a process with no Stop button.)
        sim = self._sim.isChecked()
        self._cameras_btn.setEnabled(not sim or self._cameras.running())
        self._cameras_btn.setText(
            "Stop cameras" if self._cameras.running() else "Start cameras"
        )
        self._cameras_label.setText(
            "simulation: start Gazebo in its own terminal --\n"
            "roslaunch agbot_bringup agbot_gazebo.launch" if sim else
            "real robot: front WDR + rear Brio (cameras.launch)"
        )
        self._mission_btn.setText(
            "Stop mission" if self._mission.running() else "Start mission"
        )
        self._pause_btn.setText("Resume" if self._paused else "Pause")
        self._status_label.setText("status: %s" % self._status_text)

    def closeEvent(self, event):
        # Closing the window must not leave a robot driving itself.
        self._mission.stop()
        self._cameras.stop()
        event.accept()


def ensure_master(timeout=10.0):
    """Start a roscore if none is running, and return the child (or None).

    The panel is the FIRST thing started in a session, before any roslaunch,
    so there is usually no master yet -- and rospy.init_node() below blocks
    indefinitely without one, which looks exactly like the GUI failing to
    open. On the robot a master is normally already up (the Jackal base
    bringup runs as a service), in which case this does nothing.
    """
    if rosgraph.is_master_online():
        return None
    proc = subprocess.Popen(
        ["roscore"],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        if rosgraph.is_master_online():
            return proc
        time.sleep(0.2)
    raise RuntimeError("roscore did not come up within %.0f s" % timeout)


def main():
    own_roscore = ensure_master()
    # disable_signals so Ctrl-C reaches Qt rather than rospy's handler, and
    # closeEvent still gets to stop the child launches.
    rospy.init_node("operator_panel", anonymous=True, disable_signals=True)
    app = QApplication(sys.argv)
    panel = OperatorPanel()
    panel.show()
    try:
        code = app.exec_()
    finally:
        # Only tear down a roscore this panel started; never one that was
        # already serving the robot when the panel opened.
        if own_roscore is not None:
            os.killpg(os.getpgid(own_roscore.pid), signal.SIGINT)
    sys.exit(code)


if __name__ == "__main__":
    main()
