"""Build the Gazebo twin world from the cell's own robot description.

The twin is a VIEW, not a simulation: it shows where the real robot's links are,
it does not work out where they should be. So the robot is not spawned as an
articulated model with joints and controllers. Every visual link becomes its own
`<static>` model, and gz_twin.py sets each model's pose from that link's TF
transform. Nothing here has joints, inertia, controllers or gravity, and there is
no second robot description to keep in sync: geometry comes from
description/ur5e_robotiq.urdf.xacro, the same file that drives ros2_control and
MoveIt.

Each generated model's origin IS the link frame: the URDF's visual origin is
baked into the SDF `<visual><pose>`, so at runtime the model pose can be set
straight from TF with no per-frame composition.

    world = build_world(scene_sdf, urdf_xml, "/tmp/twin.sdf")
"""
import xml.etree.ElementTree as ET

# Model names are derived from link names so the mirror can map TF -> model.
MODEL_PREFIX = "twin__"


def model_name(link):
    return MODEL_PREFIX + link


def _pose(origin):
    """URDF <origin> -> SDF pose string 'x y z r p y'."""
    xyz = "0 0 0"
    rpy = "0 0 0"
    if origin is not None:
        xyz = origin.get("xyz") or xyz
        rpy = origin.get("rpy") or rpy
    return f"{xyz} {rpy}"


def _geometry(geom):
    """URDF <geometry> -> SDF geometry XML, or None if unsupported."""
    mesh = geom.find("mesh")
    if mesh is not None:
        uri = mesh.get("filename")
        scale = mesh.get("scale")
        scale_xml = f"<scale>{scale}</scale>" if scale else ""
        return f"<mesh><uri>{uri}</uri>{scale_xml}</mesh>"
    box = geom.find("box")
    if box is not None:
        return f"<box><size>{box.get('size')}</size></box>"
    cyl = geom.find("cylinder")
    if cyl is not None:
        return (f"<cylinder><radius>{cyl.get('radius')}</radius>"
                f"<length>{cyl.get('length')}</length></cylinder>")
    sphere = geom.find("sphere")
    if sphere is not None:
        return f"<sphere><radius>{sphere.get('radius')}</radius></sphere>"
    return None


def _material(visual, urdf_root):
    """URDF <material> -> SDF material XML. Meshes usually carry their own."""
    mat = visual.find("material")
    if mat is None:
        return ""
    colour = mat.find("color")
    if colour is None and mat.get("name"):
        # A material referenced by name is defined at robot scope.
        for top in urdf_root.findall("material"):
            if top.get("name") == mat.get("name"):
                colour = top.find("color")
                break
    if colour is None:
        return ""
    rgba = colour.get("rgba", "0.7 0.7 0.7 1")
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def link_models(urdf_xml):
    """Yield (link_name, model_xml) for every link that has a visual.

    Links with several visuals get them all in one model, so the model still
    maps one-to-one onto a TF frame.
    """
    root = ET.fromstring(urdf_xml)
    for link in root.findall("link"):
        name = link.get("name")
        visuals = link.findall("visual")
        if not visuals:
            continue
        parts = []
        for i, v in enumerate(visuals):
            geom = _geometry(v.find("geometry"))
            if geom is None:
                continue
            parts.append(
                f'      <visual name="v{i}">\n'
                f"        <pose>{_pose(v.find('origin'))}</pose>\n"
                f"        <geometry>{geom}</geometry>\n"
                f"        {_material(v, root)}\n"
                f"      </visual>"
            )
        if not parts:
            continue
        body = "\n".join(parts)
        # Parked below the floor until the mirror places it, so a link never
        # flashes at the origin before the first TF lookup succeeds.
        yield name, (
            f'    <model name="{model_name(name)}">\n'
            f"      <static>true</static>\n"
            f"      <pose>0 0 -5 0 0 0</pose>\n"
            f'      <link name="link">\n{body}\n      </link>\n'
            f"    </model>"
        )


def build_world(scene_sdf_path, urdf_xml, out_path):
    """Write scene_sdf_path with the robot's link models injected before </world>.

    Returns (out_path, [link names]).
    """
    with open(scene_sdf_path) as fh:
        scene = fh.read()
    if "</world>" not in scene:
        raise ValueError(f"{scene_sdf_path} has no </world> to inject into")

    names, blocks = [], []
    for name, xml in link_models(urdf_xml):
        names.append(name)
        blocks.append(xml)

    injected = (
        "\n    <!-- Robot link visuals, generated from the cell's URDF by\n"
        "         twin_world.py. Static: gz_twin.py drives each pose from TF. -->\n"
        + "\n".join(blocks)
        + "\n\n  </world>"
    )
    with open(out_path, "w") as fh:
        fh.write(scene.replace("</world>", injected))
    return out_path, names
