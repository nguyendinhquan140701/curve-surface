# %%
import trimesh
import plotly.graph_objects as go
import numpy as np
import plotly.offline as pyo

# %%
# mesh = trimesh.load_mesh("data/obj2k.obj")
mesh = trimesh.load_mesh("data/tess.obj")
vertices = mesh.vertices


# %%
x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]
print(f"length:{len(x)}, {len(y)}, {len(z)}")


points_array = np.stack((x, y, z), axis=-1)
centroid = np.mean(points_array, axis=0)
print(f"Original Centroid: {centroid}")

# Translate the point cloud to center around the origin
centered_points = points_array - centroid

x_centered, z_centered, y_centered = (centered_points[:, 0], centered_points[:, 1], centered_points[:, 2])
print("len(y_centered):", len(y_centered))

# %%
fig = go.Figure(
    data=[
        go.Scatter3d(
            x=x_centered,
            y=y_centered,
            z=z_centered,
            mode="markers",
            marker=dict(size=4, color="blue", opacity=0.5),
        )
    ]
)

# Update the layout
fig.update_layout(
    title=f"Point Cloud", scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z")
)

# Show the plot
fig.show()
# pyo.iplot(fig)

# %%
points_array = np.stack((x_centered, y_centered, z_centered), axis=-1)

# Define the target y-coordinate value
target_y = 2.9

# Check if any y-coordinates are approximately equal to target_y
# np.isclose is used to handle floating-point precision issues
is_y_close_to_target = np.isclose(points_array[:, 1], target_y, atol=0.5)
# print(is_y_close_to_target)


filtered_points = points_array[is_y_close_to_target]

_, unique_indices = np.unique(filtered_points[:, 1], return_index=True)
unique_filtered_points = filtered_points[unique_indices]
# Print filtered points to verify
# print("Filtered Points:\n", filtered_points)

# Prepare data for Plotly
x_filtered, y_filtered, z_filtered = (
    unique_filtered_points[:, 0],
    unique_filtered_points[:, 1],
    unique_filtered_points[:, 2],
)

print("len(y_filtered):", len(y_filtered))

fig = go.Figure(
    data=[
        go.Scatter3d(
            x=x_filtered,
            y=y_filtered,
            z=z_filtered,
            mode="markers",
            marker=dict(size=5, color="blue", opacity=0.8),
        )
    ]
)

# Update layout
fig.update_layout(
    title="Filtered 3D Points",
    scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z"),
)

# Show the plot
# fig.show()
# pyo.iplot(fig)  # Use `fig.show()` if running in a script outside Jupyter

