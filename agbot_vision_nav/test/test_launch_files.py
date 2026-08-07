"""Static checks on the launch files.

These cost nothing and catch a class of bug that is otherwise invisible until
someone is standing in a field. roslaunch parses XML at launch time, so a
malformed comment or a typo'd arg name is not a warning -- it is an
`RLException: Invalid roslaunch XML syntax` and the run does not start. The
dev sandbox has no ROS1, so pytest is the only place this can be caught before
the ROS1 machine sees it.

Real failure, 2026-08-06: `--` written as an em-dash inside an `<arg>` comment
in vision_nav.launch. A double hyphen is ILLEGAL inside an XML comment (it is
reserved for the `-->` delimiter), so the file would not parse at all. Every
attempt to start a mission from the operator panel died with
`not well-formed (invalid token): line 117, column 104` and nothing else in
the workspace was wrong. This repo writes `--` as an em-dash in prose
everywhere, which is fine in Python and YAML and a syntax error here.
"""

import os
import re
import xml.etree.ElementTree as ET

import pytest

# test/ -> agbot_vision_nav/ -> src/  (the catkin workspace's source root, so
# sibling packages such as agbot_bringup are covered too: a broken
# agbot_gazebo.launch fails the same way and is launched by the same panel.)
WORKSPACE_SRC = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

XML_SUFFIXES = (".launch", ".launch.xml", ".xacro", ".urdf")
# Third-party checkouts live alongside the custom packages and are not ours.
SKIP_DIRS = {"jackal", "virtual_maize_field", "build", "devel", ".git", "tmp"}


def _xml_files():
    found = []
    for root, dirs, files in os.walk(WORKSPACE_SRC):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            if name.endswith(XML_SUFFIXES):
                found.append(os.path.join(root, name))
    return sorted(found)


XML_FILES = _xml_files()


def test_there_are_xml_files_to_check():
    """Guards the guard: a broken discovery path would make every test below
    vacuously pass and silently stop protecting anything."""
    assert XML_FILES, "no launch/xacro files found under %s" % WORKSPACE_SRC


@pytest.mark.parametrize("path", XML_FILES, ids=lambda p: os.path.basename(p))
def test_xml_is_well_formed(path):
    try:
        ET.parse(path)
    except ET.ParseError as exc:
        pytest.fail("%s is not well-formed XML: %s" % (path, exc))


@pytest.mark.parametrize("path", XML_FILES, ids=lambda p: os.path.basename(p))
def test_no_double_hyphen_inside_a_comment(path):
    """The specific 2026-08-06 failure, reported clearly.

    test_xml_is_well_formed already fails on this, but with a byte offset that
    says nothing about the cause. This one names the line and the text.
    """
    source = open(path).read()
    for match in re.finditer(r"<!--(.*?)(?:-->|\Z)", source, re.S):
        body = match.group(1)
        if "--" not in body:
            continue
        line = source[: match.start()].count("\n") + 1
        offenders = [seg.strip() for seg in body.split("\n") if "--" in seg]
        pytest.fail(
            "%s: comment starting at line %d contains '--', which is illegal "
            "inside an XML comment (reserved for the '-->' delimiter). "
            "Use ':' or an en-dash instead.\n  %s"
            % (path, line, "\n  ".join(offenders))
        )


@pytest.mark.parametrize(
    "path",
    [p for p in XML_FILES if p.endswith((".launch", ".launch.xml"))],
    ids=lambda p: os.path.basename(p),
)
def test_every_referenced_arg_is_declared(path):
    """`$(arg foo)` with no `<arg name="foo">` is a launch-time RLException.

    Worth pinning because of how this repo's config contract is written: every
    tunable is `<param ... value="$(arg x)" if="$(eval arg('x') != '')" />`, so
    adding a knob means touching two places and a typo in either is silent
    until launch.
    """
    source = open(path).read()
    declared = set(re.findall(r'<arg\s+name="([^"]+)"', source))
    referenced = set(re.findall(r"\$\(arg\s+([^)]+?)\s*\)", source))
    referenced |= set(re.findall(r"arg\(\s*'([^']+)'\s*\)", source))
    missing = sorted(referenced - declared)
    assert not missing, "%s references undeclared arg(s): %s" % (
        os.path.basename(path),
        ", ".join(missing),
    )
