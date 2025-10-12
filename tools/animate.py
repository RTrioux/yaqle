import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from quaternion import quaternion as Quat
from pathlib import Path
import argparse
import itertools

parser = argparse.ArgumentParser()
parser.add_argument("file")
parser.add_argument("--save-gif", action="store_true", help="Save the animation as a GIF")
parser.add_argument("--no-repeat", action="store_true", help="Do not repeat the animation")
args = parser.parse_args()
filePath = Path(args.file).absolute()
repeat = not args.no_repeat
save_gif = args.save_gif

# --------------------------------------------
# Utility functions
# --------------------------------------------


def quaternion_to_rotation_matrix(q):
    q0, q1, q2, q3 = q
    return np.array(
        [
            [1 - 2 * q2**2 - 2 * q3**2, 2 * q1 * q2 - 2 * q0 * q3, 2 * q1 * q3 + 2 * q0 * q2],
            [2 * q1 * q2 + 2 * q0 * q3, 1 - 2 * q1**2 - 2 * q3**2, 2 * q2 * q3 - 2 * q0 * q1],
            [2 * q1 * q3 - 2 * q0 * q2, 2 * q2 * q3 + 2 * q0 * q1, 1 - 2 * q1**2 - 2 * q2**2],
        ]
    )


def transformation_matrix_from_quaternion(q, t=(0, 0, 0)):
    R = quaternion_to_rotation_matrix(q)
    tx, ty, tz = t
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def transform_vertices(vertices, T):
    homog = np.column_stack((vertices, np.ones(vertices.shape[0])))
    return (homog @ T.T)[:, :3]


def dual_quat_to_transform(dq):
    dq_r = Quat(*dq[:4])
    dq_d = Quat(*dq[4:])
    t = 2 * dq_d * dq_r.conj()
    return dq_r, np.array([t.x, t.y, t.z])


# --------------------------------------------
# Parse input file
# --------------------------------------------

objects = {}
with open(filePath, "r") as file:
    for line in file:
        if not line.strip():
            continue
        name, rest = line.split(":", 1)
        type_tag, data_str = rest.split(":", 1)
        data = np.array([float(x) for x in data_str.split(",")])
        if name not in objects:
            objects[name] = {"type": type_tag, "frames": []}
        objects[name]["frames"].append(data)

# --------------------------------------------
# Visualization setup
# --------------------------------------------

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_box_aspect([1, 1, 1])
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])

# Base cube (for visualization)
base_cube = np.array(
    [
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
    ]
)

# Local trihedron (length = 1)
base_trihedron = np.array(
    [
        [0, 0, 0],
        [1, 0, 0],  # X
        [0, 0, 0],
        [0, 1, 0],  # Y
        [0, 0, 0],
        [0, 0, 1],  # Z
    ]
)
axis_colors = ["r", "g", "b"]

colors = itertools.cycle(["red", "green", "blue", "orange", "purple", "cyan"])
obj_colors = {name: next(colors) for name in objects}

trails = {name: [] for name in objects}

# --------------------------------------------
# Update function
# --------------------------------------------


def update(frame_idx):
    ax.clear()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_box_aspect([1, 1, 1])
    ax.set_title(f"Frame {frame_idx}")

    for name, obj in objects.items():
        data = obj["frames"][min(frame_idx, len(obj["frames"]) - 1)]
        if obj["type"] == "q":
            q = data
            T = transformation_matrix_from_quaternion(q)
            pos = np.array([0, 0, 0])
        elif obj["type"] == "dq":
            dq_r, t = dual_quat_to_transform(data)
            T = transformation_matrix_from_quaternion([dq_r.w, dq_r.x, dq_r.y, dq_r.z], t)
            pos = t
        else:
            continue  # unknown type

        # --- Transformed cube ---
        transformed = transform_vertices(base_cube, T)
        color = obj_colors[name]
        ax.scatter(*transformed.T, color=color, label=name, alpha=0.6)

        # --- Local trihedron ---
        tri = transform_vertices(base_trihedron, T)
        for i, color in enumerate(axis_colors):
            ax.plot(
                [tri[2 * i][0], tri[2 * i + 1][0]], [tri[2 * i][1], tri[2 * i + 1][1]], [tri[2 * i][2], tri[2 * i + 1][2]], color=color, linewidth=2
            )

        # trail
        trails[name].append(pos)
        trail = np.array(trails[name])
        ax.plot(trail[:, 0], trail[:, 1], trail[:, 2], color=color, linestyle="--")

    ax.legend()


# --------------------------------------------
# Animation
# --------------------------------------------

num_frames = max(len(obj["frames"]) for obj in objects.values())
ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=50, repeat=repeat)

if save_gif:
    ani.save(f"{filePath.stem}.gif", writer="imagemagick", fps=30)
plt.show()
