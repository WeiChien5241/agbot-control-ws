import pytest

from agbot_vision_nav.launch_args import (
    cameras_launch_args,
    mission_launch_args,
)

MODEL = "/home/user/exported_best.pt"


def args_of(argv):
    """The name:=value pairs only, as a dict."""
    return dict(a.split(":=", 1) for a in argv if ":=" in a)


def test_cameras_launch_args():
    assert cameras_launch_args() == ["agbot_vision_nav", "cameras.launch"]


def test_minimal_run_passes_only_model_path_and_the_checkboxes():
    """⚠ THE contract: a blank field must NOT be passed, so config/params.yaml
    stays the source of truth. A panel that helpfully sent its own default for
    every knob would re-pin all of them on every run -- which is exactly the
    bug that made editing params.yaml a no-op before 2026-07-30."""
    argv = mission_launch_args(MODEL)
    assert argv[:2] == ["agbot_vision_nav", "vision_nav.launch"]
    assert args_of(argv) == {"model_path": MODEL}


def test_blank_model_path_falls_back_to_the_launch_default():
    """vision_nav.launch defaults model_path to the package's
    config/exported_best.pt, which is the normal case on the robot. Requiring
    it here would make the panel harder to use than the command line."""
    argv = mission_launch_args("")
    assert args_of(argv) == {}
    assert argv == ["agbot_vision_nav", "vision_nav.launch"]


def test_blank_and_whitespace_fields_are_dropped():
    argv = mission_launch_args(
        MODEL, num_rows="", linear_x_cruise="   ", angular_z_max=None,
        first_turn_direction="",
    )
    assert args_of(argv) == {"model_path": MODEL}


def test_filled_fields_are_passed():
    argv = mission_launch_args(
        MODEL, num_rows="4", first_turn_direction="right",
        linear_x_cruise="0.2", angular_z_max="0.25",
    )
    assert args_of(argv) == {
        "model_path": MODEL,
        "num_rows": "4",
        "first_turn_direction": "right",
        "linear_x_cruise": "0.2",
        "angular_z_max": "0.25",
    }


def test_checkboxes_always_carry_an_opinion():
    """A checkbox has no blank state, so it is always passed -- including
    when false. rear_camera_enabled:=false must be able to turn the rear
    camera OFF even though params.yaml may say true."""
    argv = mission_launch_args(
        MODEL, mission_enabled=True, rear_camera_enabled=False
    )
    assert args_of(argv)["mission_enabled"] == "true"
    assert args_of(argv)["rear_camera_enabled"] == "false"


def test_sim_flag():
    assert "sim:=true" in mission_launch_args(MODEL, sim=True)
    assert "sim:=true" not in mission_launch_args(MODEL, sim=False)


def test_blank_model_path_variants_are_all_dropped():
    for blank in ("", "   ", None):
        assert "model_path" not in args_of(mission_launch_args(blank))


def test_unknown_field_raises_rather_than_being_dropped():
    """A typo'd field name must not be silently discarded: the run would
    quietly use the yaml value the operator was trying to override."""
    with pytest.raises(ValueError):
        mission_launch_args(MODEL, linear_x_cruse="0.2")


def test_cameras_launch_args_wdr_front_passes_nothing():
    """"wdr" is cameras.launch's own default (front:=true brio_front:=false),
    so the default call and an explicit wdr call must be identical -- that is
    what keeps every existing caller (and the test above) working unchanged."""
    assert cameras_launch_args(front_source="wdr") == cameras_launch_args()


def test_cameras_launch_args_brio_front_overrides_both_toggles():
    argv = cameras_launch_args(front_source="brio_front")
    assert args_of(argv) == {"front": "false", "brio_front": "true"}


def test_cameras_launch_args_rejects_unknown_front_source():
    with pytest.raises(ValueError):
        cameras_launch_args(front_source="gopro")


def test_mission_launch_args_brio_front_sets_camera_topic():
    """The Brio swap changes which topic vision_nav_node must subscribe to,
    not just which node cameras.launch starts -- the two launch files share
    no state, so mission_launch_args has to carry this too."""
    argv = mission_launch_args(MODEL, front_source="brio_front")
    assert args_of(argv)["camera_topic"] == "/brio_front/image_raw/compressed"
    assert args_of(argv)["camera_topic_is_compressed"] == "true"


def test_mission_launch_args_wdr_front_passes_no_camera_topic():
    """wdr is vision_nav.launch's own default (/usb_cam/.../compressed), so
    it must not be passed explicitly -- passing it would silently stop a
    camera_topic:= override on the roslaunch command line (if anyone ever
    added one) from winning, same as any other blank-field rule in this repo.
    """
    argv = mission_launch_args(MODEL, front_source="wdr")
    assert "camera_topic" not in args_of(argv)


def test_mission_launch_args_sim_ignores_front_source():
    """There is no Brio in Gazebo -- sim always wins the camera_topic."""
    argv = mission_launch_args(MODEL, sim=True, front_source="brio_front")
    assert "camera_topic" not in args_of(argv)
    assert "sim:=true" in argv


def test_mission_launch_args_rejects_unknown_front_source():
    with pytest.raises(ValueError):
        mission_launch_args(MODEL, front_source="gopro")


def test_frame_source_is_always_the_real_cameras():
    """The panel never starts Gazebo.

    It used to, keyed off the simulation checkbox. The settled sim workflow is
    the simulator in its OWN terminal (its output otherwise buries the
    vision-nav startup config block), so the panel's frame-source button is
    the real-robot USB bringup and nothing else -- and it takes no sim
    argument at all, so the old behaviour cannot come back by accident.
    """
    assert cameras_launch_args() == ["agbot_vision_nav", "cameras.launch"]
    with pytest.raises(TypeError):
        cameras_launch_args(sim=True)
