import open3d as o3d
import numpy as np
from collections import defaultdict

def oriented_bbox(pcd, labels, min_points=7, max_points=47):
    obbs = []
    label_to_indices = defaultdict(list)

    # Aggregate indices by label
    for index, label in enumerate(labels):
        label_to_indices[label].append(index)

    # Process each cluster
    for indices in label_to_indices.values():
        num_points = len(indices)
        if min_points < num_points < max_points:
            # Select points for the current cluster
            points = np.asarray(pcd.points)[indices]
            if points.size > 0:
                # Create a new point cloud for the cluster
                cluster_pcd = o3d.geometry.PointCloud()
                cluster_pcd.points = o3d.utility.Vector3dVector(points)

                # Compute the axis-aligned bounding box
                obb = cluster_pcd.get_axis_aligned_bounding_box()
                obb.color = (0, 0, 0)
                obbs.append(obb)

    print(f"Number of Bounding Boxes calculated {len(obbs)}")

    return obbs

