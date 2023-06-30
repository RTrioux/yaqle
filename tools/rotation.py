import csv
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from quaternion import quaternion as Quat
import argparse
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("file")
args = parser.parse_args()
filePath = Path(args.file).absolute()
print(filePath)

# Create a figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


# Define the initial shape (cube)
vertices = np.array(
    [
        [-1, -1, -1],
        [1, -1, -1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, 1],
        [1, -1, 1],
        [1, 1, 1],
        [-1, 1, 1],
    ]
)

# Define the fixed trihedra for reference frames
trihedra = [
    (np.array([[0, 0, 0], [1, 0, 0]]), "red"),  # X-axis
    (np.array([[0, 0, 0], [0, 1, 0]]), "green"),  # Y-axis
    (np.array([[0, 0, 0], [0, 0, 1]]), "blue"),  # Z-axis
]


# Read quaternions and translations from CSV file
quaternions = []
translations = []
with open(filePath, "r") as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        dual_quaternion = [float(x) for x in row[:8]]
        qr_arr = [float(x) for x in dual_quaternion[:4]]
        qd_arr = [float(x) for x in dual_quaternion[4:]]
        qr = Quat(*qr_arr)
        qd = Quat(*qd_arr)
        t = 2 * qd * qr.conj()

        translation = [t.x, t.y, t.z]
        quaternions.append(qr_arr)
        translations.append(translation)


# Update function for rotation and translation
def update_transform(frame):
    quaternion = quaternions[frame]
    translation = translations[frame]
    transformation_matrix = transformation_matrix_from_quaternion(
        quaternion, translation
    )
    transformed_vertices = transform_vertices(vertices, transformation_matrix)
    ax.clear()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_box_aspect([1, 1, 1])
    ax.scatter(
        transformed_vertices[:, 0],
        transformed_vertices[:, 1],
        transformed_vertices[:, 2],
    )

    x, y, z = translations[frame]
    textCoord = "({:10.2f} {:10.2f} {:10.2f})".format(x, y, z)
    ax.annotate(
        textCoord,
        xy=(0, 1),
        xycoords="axes fraction",
        fontsize=12,
        xytext=(10, -10),
        textcoords="offset points",
        ha="left",
        va="top",
    )

    # Plot fixed trihedra
    for trihedron, color in trihedra:
        trihedron_transformed = transform_vertices(trihedron, transformation_matrix)
        ax.plot(
            trihedron_transformed[:, 0],
            trihedron_transformed[:, 1],
            trihedron_transformed[:, 2],
            color=color,
        )


# Helper function to convert quaternion and translation to a 4x4 transformation matrix
def transformation_matrix_from_quaternion(quaternion, translation):
    q0, q1, q2, q3 = quaternion
    tx, ty, tz = translation
    rotation_matrix = np.array(
        [
            [
                1 - 2 * q2**2 - 2 * q3**2,
                2 * q1 * q2 - 2 * q0 * q3,
                2 * q1 * q3 + 2 * q0 * q2,
            ],
            [
                2 * q1 * q2 + 2 * q0 * q3,
                1 - 2 * q1**2 - 2 * q3**2,
                2 * q2 * q3 - 2 * q0 * q1,
            ],
            [
                2 * q1 * q3 - 2 * q0 * q2,
                2 * q2 * q3 + 2 * q0 * q1,
                1 - 2 * q1**2 - 2 * q2**2,
            ],
        ]
    )
    transformation_matrix = np.array(
        [
            [rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], tx],
            [rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2], ty],
            [rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2], tz],
            [0, 0, 0, 1],
        ]
    )
    return transformation_matrix


# Helper function to transform vertices using a 4x4 transformation matrix
def transform_vertices(vertices, transformation_matrix):
    homogeneous_vertices = np.column_stack((vertices, np.ones(vertices.shape[0])))
    transformed_vertices = np.dot(homogeneous_vertices, transformation_matrix.T)[:, :3]
    return transformed_vertices


# Create the animation
animation = animation.FuncAnimation(
    fig, update_transform, frames=len(quaternions), interval=30
)

# animation.save("animation.gif", writer="imagemagick", fps=60)

# Show the animation
plt.show()
