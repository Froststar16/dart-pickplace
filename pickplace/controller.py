"""Stable-PD joint-space torque controller for a DART Skeleton.

Standard discrete PD control gets numerically unstable fast at small
timesteps with non-trivial gains. "Stable PD" (Tan et al.) fixes this by
evaluating the position error against where the system *would* be after one
more timestep at the current velocity, and by adding the mass-matrix and
Coriolis/gravity feedforward terms so the feedback gains only have to
correct the residual error rather than fight gravity from scratch every
step.
"""
import numpy as np


class StablePDController:
    def __init__(self, skeleton, kp, kd):
        n = skeleton.getNumDofs()
        self.skeleton = skeleton
        self.kp = np.eye(n) * kp
        self.kd = np.eye(n) * kd

    def compute_forces(self, q_target, dt):
        skel = self.skeleton
        q = skel.getPositions()
        dq = skel.getVelocities()
        M = skel.getMassMatrix()
        Cg = skel.getCoriolisAndGravityForces()
        q_err = q_target - (q + dq * dt)
        dq_err = -dq
        ddq_des = self.kp.dot(q_err) + self.kd.dot(dq_err)
        return M.dot(ddq_des) + Cg
