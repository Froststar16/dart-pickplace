import numpy as np
import dartpy as dart
from .controller import StablePDController

POSE_HOME        = np.array([0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
POSE_ABOVE_PICK  = np.array([0.3, -0.6,  0.9,  0.0,  0.4,  0.0])
POSE_AT_PICK     = np.array([0.3, -0.3,  0.6,  0.0,  0.4,  0.0])
POSE_ABOVE_PLACE = np.array([-0.4, -0.6,  0.9, 0.0,  0.4,  0.0])
POSE_AT_PLACE    = np.array([-0.4, -0.3,  0.6, 0.0,  0.4,  0.0])

STATES = [
    ("HOME_TO_ABOVE_PICK", POSE_ABOVE_PICK, None),
    ("DESCEND_TO_PICK",    POSE_AT_PICK,    None),
    ("GRASP",              POSE_AT_PICK,    "grasp"),
    ("LIFT",               POSE_ABOVE_PICK, None),
    ("TRANSPORT",          POSE_ABOVE_PLACE, None),
    ("DESCEND_TO_PLACE",   POSE_AT_PLACE,   None),
    ("RELEASE",            POSE_AT_PLACE,   "release"),
    ("RETREAT",            POSE_HOME,       None),
]

POSITION_TOLERANCE = 0.06   # rad, per-joint -- loose on purpose, see note below
SETTLE_STEPS = 50           # extra steps to hold once a grasp/release triggers
STATE_TIMEOUT = 4000        # steps -- hard ceiling so the FSM can never stall forever
#
# Why a timeout fallback at all: once the arm grasps the box, the controller has to
# fight the box's weight through the wrist as an added, off-center load. With a real
# torque-limited arm and finite PD gains this produces a small steady-state tracking
# error that may never tick below POSITION_TOLERANCE. Rather than tune gains to chase
# a perfect zero (which risks oscillation/instability, the same class of issue you'll
# recognize from tuning your MPC controller), the task advances once it's "close
# enough" OR the timeout fires -- a standard pattern in real task supervisors.


class PickAndPlaceTask:
    def __init__(self, world, robot, box, kp=600.0, kd=120.0):
        self.world = world
        self.robot = robot
        self.box = box
        self.controller = StablePDController(robot, kp, kd)
        self.ee = robot.getBodyNode(robot.getNumBodyNodes() - 1)
        self.state_idx = 0
        self.state_steps = 0
        self.settle_counter = 0
        self.action_done = False
        self.weld = None
        self.done = False

    def _reached(self, q_target):
        q = self.robot.getPositions()
        return np.max(np.abs(q - q_target)) < POSITION_TOLERANCE

    def step(self):
        if self.done:
            return
        name, q_target, action = STATES[self.state_idx]
        dt = self.world.getTimeStep()
        forces = self.controller.compute_forces(q_target, dt)
        self.robot.setForces(forces)
        self.world.step()
        self.state_steps += 1

        close_enough = self._reached(q_target) or self.state_steps >= STATE_TIMEOUT

        if action in ("grasp", "release") and close_enough and not self.action_done:
            if action == "grasp" and self.weld is None:
                self.weld = dart.constraint.WeldJointConstraint(self.ee, self.box.getBodyNode(0))
                self.world.getConstraintSolver().addConstraint(self.weld)
            if action == "release" and self.weld is not None:
                self.world.getConstraintSolver().removeConstraint(self.weld)
                self.weld = None
            self.action_done = True

        ready_to_advance = close_enough and (action is None or self.action_done)
        if action in ("grasp", "release") and self.action_done:
            self.settle_counter += 1
            ready_to_advance = self.settle_counter >= SETTLE_STEPS

        if ready_to_advance:
            self.state_idx += 1
            self.state_steps = 0
            self.settle_counter = 0
            self.action_done = False
            if self.state_idx >= len(STATES):
                self.done = True

    @property
    def state_name(self):
        if self.done:
            return "DONE"
        return STATES[self.state_idx][0]
