"""
visualize.py — Runs the pick-and-place simulation and plays back a 3D
matplotlib animation showing the arm as a stick figure, the end-effector
trail, and the box clearly changing colour when grasped/released.

  GREY  box = waiting to be picked
  YELLOW box = grasped and being carried by the arm
  RED   box = released, resting on the ground
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from pickplace.world_setup import build_world
from pickplace.task import PickAndPlaceTask

# -------------------------------------------------------------------------
# 1. Run the simulation and record frames
# -------------------------------------------------------------------------
RECORD_EVERY = 30
MAX_STEPS    = 20000

# States where the box is actively grasped
GRASPED_STATES  = {"LIFT", "TRANSPORT", "DESCEND_TO_PLACE"}
RELEASED_STATES = {"RELEASE", "RETREAT", "DONE"}

print("Running simulation (~15 s) ...")
world, robot, box = build_world()
task = PickAndPlaceTask(world, robot, box)

frames = []
step   = 0

while not task.done and step < MAX_STEPS:
    task.step()
    step += 1

    if step % RECORD_EVERY == 0:
        body_pos = [
            robot.getBodyNode(i).getWorldTransform().translation().copy()
            for i in range(robot.getNumBodyNodes())
        ]
        state = task.state_name
        if state in GRASPED_STATES:
            box_color = "yellow"
        elif state in RELEASED_STATES:
            box_color = "#ff4444"
        else:
            box_color = "#aaaaaa"      # grey = waiting

        frames.append({
            "body_pos":  body_pos,
            "box_pos":   box.getPositions()[3:6].copy(),
            "box_color": box_color,
            "grasped":   state in GRASPED_STATES,
            "state":     state,
            "step":      step,
        })

print(f"Done — {step} sim steps → {len(frames)} animation frames")

# -------------------------------------------------------------------------
# 2. Axis limits
# -------------------------------------------------------------------------
all_pts = np.array(
    [p for f in frames for p in f["body_pos"]] +
    [f["box_pos"] for f in frames]
)
pad = 0.15
x_lim = (all_pts[:, 0].min() - pad, all_pts[:, 0].max() + pad)
y_lim = (all_pts[:, 1].min() - pad, all_pts[:, 1].max() + pad)
z_lim = (all_pts[:, 2].min() - pad, all_pts[:, 2].max() + pad)

# -------------------------------------------------------------------------
# 3. Figure layout  —  3-D view  +  box-height subplot
# -------------------------------------------------------------------------
fig = plt.figure(figsize=(13, 7))
fig.patch.set_facecolor("#0f0f0f")

ax3d = fig.add_subplot(121, projection="3d")
ax3d.set_facecolor("#0f0f0f")
for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
    axis.label.set_color("white")
    axis.set_tick_params(colors="white")
ax3d.set_xlabel("X"); ax3d.set_ylabel("Y"); ax3d.set_zlabel("Z")
ax3d.set_xlim(*x_lim); ax3d.set_ylim(*y_lim); ax3d.set_zlim(*z_lim)
ax3d.view_init(elev=20, azim=-55)
title3d = ax3d.set_title("", color="white", fontsize=12, pad=8)

# Legend text
fig.text(0.05, 0.15, "⬜  waiting   🟡  grasped   🔴  released",
         color="white", fontsize=9)

# 3-D artists
arm_line,   = ax3d.plot([], [], [], color="#00cfff", linewidth=2.5,
                        marker="o", markersize=5, markerfacecolor="white")
ee_trail,   = ax3d.plot([], [], [], color="#00ff99", linewidth=1,
                        alpha=0.55, linestyle="--")
grasp_line, = ax3d.plot([], [], [], color="yellow", linewidth=1.5,
                        linestyle=":", alpha=0.8)
box_dot,    = ax3d.plot([], [], [], marker="s", markersize=11,
                        linestyle="None", color="#aaaaaa")

# Box-height over time subplot
ax2 = fig.add_subplot(122)
ax2.set_facecolor("#1a1a1a")
ax2.tick_params(colors="white"); ax2.xaxis.label.set_color("white")
ax2.yaxis.label.set_color("white")
for sp in ax2.spines.values(): sp.set_edgecolor("#444")
ax2.set_xlabel("Sim step"); ax2.set_ylabel("Box Y (height)")
ax2.set_title("Box height over time", color="white", fontsize=11)

all_steps = [f["step"]    for f in frames]
all_ys    = [f["box_pos"][1] for f in frames]
ax2.plot(all_steps, all_ys, color="#555", linewidth=1)           # full path dim
height_line, = ax2.plot([], [], color="yellow", linewidth=2)     # animated portion
ax2.set_xlim(min(all_steps), max(all_steps))
ax2.set_ylim(min(all_ys) - 0.05, max(all_ys) + 0.05)
ax2.axhline(y=0, color="#444", linestyle="--", linewidth=0.8)
ax2.text(all_steps[0], 0.01, " ground", color="#777", fontsize=8)

fig.tight_layout(pad=2)

# -------------------------------------------------------------------------
# 4. Animation
# -------------------------------------------------------------------------
ee_xs, ee_ys, ee_zs   = [], [], []
hist_steps, hist_ys   = [], []

def update(i):
    f = frames[i]
    bps = f["body_pos"]

    # --- Arm stick figure ---
    xs = [p[0] for p in bps];  ys = [p[1] for p in bps];  zs = [p[2] for p in bps]
    arm_line.set_data(xs, ys);  arm_line.set_3d_properties(zs)

    # --- End-effector trail ---
    ee = bps[-1]
    ee_xs.append(ee[0]);  ee_ys.append(ee[1]);  ee_zs.append(ee[2])
    ee_trail.set_data(ee_xs, ee_ys);  ee_trail.set_3d_properties(ee_zs)

    # --- Box ---
    bp = f["box_pos"]
    box_dot.set_data([bp[0]], [bp[1]])
    box_dot.set_3d_properties([bp[2]])
    box_dot.set_color(f["box_color"])
    box_dot.set_markersize(13 if f["grasped"] else 10)

    # --- Grasp connector line (EE → box) when grasped ---
    if f["grasped"]:
        grasp_line.set_data([ee[0], bp[0]], [ee[1], bp[1]])
        grasp_line.set_3d_properties([ee[2], bp[2]])
    else:
        grasp_line.set_data([], []);  grasp_line.set_3d_properties([])

    # --- Title ---
    glyph = "🟡 GRASPED" if f["grasped"] else f["state"]
    title3d.set_text(f"Step {f['step']:>6}  |  {glyph}")

    # --- Height chart ---
    hist_steps.append(f["step"]);  hist_ys.append(bp[1])
    height_line.set_data(hist_steps, hist_ys)

    return arm_line, ee_trail, box_dot, grasp_line, title3d, height_line


ani = animation.FuncAnimation(
    fig, update,
    frames=len(frames),
    interval=50,
    blit=False,
    repeat=True,
)

plt.show()