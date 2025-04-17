import argparse
import json

import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("json_path")
args = parser.parse_args()
with open(args.json_path, "r") as f:
    data = json.load(f)

poses = data["pose"]
targets = data["control"]

x = [p[0] for p in poses]
y = [p[1] for p in poses]
z = [p[2] for p in poses]
xt = [p[0] for p in targets]
yt = [p[1] for p in targets]
zt = [p[2] for p in targets]
points = np.array([x, y, z]).T
colors = np.linspace(0, 1, len(points))
cmap = cm.viridis
norm = mcolors.Normalize(vmin=0, vmax=len(points))

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(xt, yt, zt, c='red')

for i in range(len(points) - 1):
    ax.plot(
        points[i : i + 2, 0],
        points[i : i + 2, 1],
        points[i : i + 2, 2],
        color=cmap(norm(i)),
    )

mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
mappable.set_array([])
cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
cbar.set_label("Time Step")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Pose Trajectory of Flapper")

plt.savefig("trajectory.png")
plt.show()