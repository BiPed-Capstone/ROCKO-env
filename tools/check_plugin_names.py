#!/usr/bin/env python3
"""Check that every hardware plugin a robot description names can actually be found by ROS.

    python3 -I tools/check_plugin_names.py <expanded.urdf> [<expanded.urdf> ...]

verify_urdf.sh runs it on every expansion it makes (Tier 0). Why it exists: off the Pi the real
motor and IMU plugins are never loaded (the mock or the simulator stands in for them), so a name
that does not match would first show up on the robot, as "class not found" when the launch starts.
For each <plugin> inside a <ros2_control> block it checks the three places that must agree:

  1. the URDF:       <plugin>rocko_env/ICM20948</plugin>
  2. the plugin XML: rocko_env/rocko_env_hardware.xml has <class name="rocko_env/ICM20948"
                     type="rocko_env::ICM20948" base_class_type="hardware_interface::SensorInterface">
  3. the C++ source: some .cpp under rocko_env/ says
                     PLUGINLIB_EXPORT_CLASS(rocko_env::ICM20948, hardware_interface::SensorInterface)

and three build facts pluginlib also needs: that .cpp is listed in rocko_env/CMakeLists.txt; the XML's
<library path> is the CMake library target (add_library); and CMake exports the XML with
pluginlib_export_plugin_description_file(hardware_interface rocko_env_hardware.xml).
A plugin from another package (only mock_components/GenericSystem is used) is accepted by name.
It cannot show that the class compiles, links or loads: only a build and a launch on the Pi can. It reads
the C++ as text with comments removed, so an export inside `#if 0`, or in a file CMake does not build, still
counts.
Standard library only. Exit 0 = all found, 1 = a mismatch (each printed), 2 = could not run.
"""
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

REPO = pathlib.Path(__file__).resolve().parent.parent
HARDWARE_XML = REPO / "rocko_env" / "rocko_env_hardware.xml"
CMAKELISTS = REPO / "rocko_env" / "CMakeLists.txt"
SOURCES = REPO / "rocko_env" / "rocko_env"
EXTERNAL_PLUGINS = {"mock_components/GenericSystem"}   # ros2_control's own mock; not ours to check
# PLUGINLIB_EXPORT_CLASS(<type>, <base class>), across line breaks
EXPORT = re.compile(r"PLUGINLIB_EXPORT_CLASS\(\s*([\w:]+)\s*,\s*([\w:]+)\s*\)")
# C++ comments, removed first so a commented-out export does not count
COMMENTS = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
COMMENTS_CMAKE = re.compile(r"#[^\n]*")   # CMake comments, so a commented-out source line does not count


def declared_classes():
    """name -> (type, base_class_type) from the plugin XML."""
    root = ET.parse(HARDWARE_XML).getroot()
    return {c.get("name"): (c.get("type"), c.get("base_class_type")) for c in root.iter("class")}


def exported_classes():
    """type -> list of (base class, source path relative to rocko_env/), from every PLUGINLIB_EXPORT_CLASS
    in the package's C++ sources. A list, so a class exported twice is seen."""
    found = {}
    for path in SOURCES.rglob("*.cpp"):
        code = COMMENTS.sub("", path.read_text(errors="replace"))
        for cls, base in EXPORT.findall(code):
            found.setdefault(cls, []).append((base, path.relative_to(CMAKELISTS.parent).as_posix()))
    return found


def build_problems(cmake_text, library_path):
    """The two facts about the whole plugin XML that CMake must agree with."""
    problems = []
    target = re.search(r"add_library\(\s*(\w+)", cmake_text)
    if not target or target.group(1) != library_path:
        problems.append(f"{HARDWARE_XML.name} says <library path=\"{library_path}\">, but the CMake library target is "
                        f"{target.group(1) if target else 'not found'}")
    if not re.search(r"pluginlib_export_plugin_description_file\(\s*hardware_interface\s+" + re.escape(HARDWARE_XML.name),
                     cmake_text):
        problems.append(f"CMakeLists.txt does not export {HARDWARE_XML.name} for hardware_interface "
                        f"(pluginlib_export_plugin_description_file)")
    return problems


def plugins_in(urdf_path):
    """Every <plugin> text inside a <ros2_control> block (XML comments are dropped by the parser)."""
    root = ET.parse(urdf_path).getroot()
    return [p.text.strip() for block in root.iter("ros2_control") for p in block.iter("plugin") if p.text]


def main(paths):
    if not paths:
        print("usage: python3 -I tools/check_plugin_names.py <expanded.urdf> [<expanded.urdf> ...]")
        return 2
    try:
        declared, exported = declared_classes(), exported_classes()
        library_path = ET.parse(HARDWARE_XML).getroot().get("path")
        cmake_text = COMMENTS_CMAKE.sub("", CMAKELISTS.read_text())
        names = sorted({name for path in paths for name in plugins_in(path)})
    except (OSError, ET.ParseError) as error:
        print(f"could not run: {error}")
        return 2
    problems = build_problems(cmake_text, library_path)
    for name in names:
        if name in EXTERNAL_PLUGINS:
            continue
        if name not in declared:
            problems.append(f"{name}: no <class name=\"{name}\"> in {HARDWARE_XML.name}")
            continue
        cls, base = declared[name]
        if cls not in exported:
            problems.append(f"{name}: {HARDWARE_XML.name} says type {cls}, but no PLUGINLIB_EXPORT_CLASS({cls}, ...) "
                            f"in any .cpp under rocko_env/rocko_env")
            continue
        if len(exported[cls]) > 1:
            problems.append(f"{name}: {cls} is exported more than once: {exported[cls]}")
        source_base, source = exported[cls][0]
        if source_base != base:
            problems.append(f"{name}: {HARDWARE_XML.name} says base class {base}, the source exports {source_base}")
        if source not in cmake_text:
            problems.append(f"{name}: {source} exports it, but CMakeLists.txt does not build that file")
    for line in problems:
        print(f"FAIL  {line}")
    if not problems:
        ours = [n for n in names if n not in EXTERNAL_PLUGINS]
        print(f"ok    plugin names: {', '.join(ours)} each match the plugin XML and a C++ class exported from a built source;"
              f" the XML's library and export match CMakeLists.txt"
              f" (not checked, from ros2_control itself: {', '.join(sorted(set(names) & EXTERNAL_PLUGINS)) or 'none'})")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
