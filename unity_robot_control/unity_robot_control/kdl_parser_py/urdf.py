import PyKDL


def _to_kdl_pose(origin):
    if origin is None:
        return PyKDL.Frame.Identity()
    xyz = origin.xyz or [0.0, 0.0, 0.0]
    rpy = origin.rpy or [0.0, 0.0, 0.0]
    rotation = PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2])
    translation = PyKDL.Vector(xyz[0], xyz[1], xyz[2])
    return PyKDL.Frame(rotation, translation)


def _to_kdl_joint(joint):
    axis = joint.axis or [0.0, 0.0, 0.0]
    axis_vec = PyKDL.Vector(axis[0], axis[1], axis[2])
    if joint.type in ("revolute", "continuous"):
        return PyKDL.Joint(joint.name, PyKDL.Vector(0.0, 0.0, 0.0), axis_vec, PyKDL.Joint.RotAxis)
    if joint.type == "prismatic":
        return PyKDL.Joint(joint.name, PyKDL.Vector(0.0, 0.0, 0.0), axis_vec, PyKDL.Joint.TransAxis)
    if joint.type == "fixed":
        return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)
    return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)


def treeFromUrdfModel(robot_model):
    root = robot_model.get_root()
    if not root:
        return False, PyKDL.Tree()

    tree = PyKDL.Tree(root)
    segments = []
    for joint in robot_model.joints:
        parent = joint.parent
        child = joint.child
        if not parent or not child:
            continue
        pose = _to_kdl_pose(joint.origin)
        kdl_joint = _to_kdl_joint(joint)
        segment = PyKDL.Segment(child, kdl_joint, pose, PyKDL.RigidBodyInertia())
        segments.append((parent, child, segment))

    added_links = {root}
    progress = True
    while segments and progress:
        progress = False
        remaining = []
        for parent, child, segment in segments:
            if parent in added_links:
                if tree.addSegment(segment, parent):
                    added_links.add(child)
                    progress = True
                else:
                    remaining.append((parent, child, segment))
            else:
                remaining.append((parent, child, segment))
        segments = remaining

    if segments:
        return False, tree
    return True, tree
