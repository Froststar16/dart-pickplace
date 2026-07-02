import os
import numpy as np
import dartpy as dart

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_URDF = os.path.join(_PROJECT_ROOT, "data", "urdf", "KR5", "KR5 sixx R650.urdf")
GROUND_URDF = os.path.join(_PROJECT_ROOT, "data", "urdf", "KR5", "ground.urdf")


def build_world():
    world = dart.simulation.World()
    world.setGravity([0, -9.81, 0])
    world.setTimeStep(0.001)

    loader = dart.utils.DartLoader()

    urdf_dir = os.path.dirname(ARM_URDF)
    original_dir = os.getcwd()
    os.chdir(urdf_dir)

    ground = loader.parseSkeleton(GROUND_URDF)
    if ground is None:
        os.chdir(original_dir)
        raise RuntimeError(f"Failed to load ground URDF from {GROUND_URDF}")
    world.addSkeleton(ground)

    robot = loader.parseSkeleton(ARM_URDF)
    os.chdir(original_dir)
    if robot is None:
        raise RuntimeError(f"Failed to load arm URDF from {ARM_URDF}")
    robot.setName("arm")
    world.addSkeleton(robot)

    box = _make_box("pick_box", [0.326, 0.057, -0.100])
    world.addSkeleton(box)
    return world, robot, box


def _make_box(name, xyz, size=0.05, color=(0.8, 0.2, 0.2)):
    skel = dart.dynamics.Skeleton(name)
    joint, body = skel.createFreeJointAndBodyNodePair()
    shape = dart.dynamics.BoxShape(np.array([size, size, size]))
    shape_node = body.createShapeNode(shape)
    shape_node.createVisualAspect().setColor(list(color))
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    positions = skel.getPositions()
    positions[3:6] = xyz
    skel.setPositions(positions)
    return skel
