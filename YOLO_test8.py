import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

if __name__ == "__main__":
    print("Read SUN dataset")
    color_raw = o3d.io.read_image(
        "rgbd-scenes/meeting_small/meeting_small_1/meeting_small_1_29.png")
    depth_raw = o3d.io.read_image(
        "rgbd-scenes/meeting_small/meeting_small_1/meeting_small_1_29_depth.png")
    rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(
        color_raw, depth_raw)
    print(rgbd_image)
    plt.subplot(1, 2, 1)
    plt.title('SUN grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('SUN depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        pcd.cluster_dbscan(eps=0.1, min_points=10, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
'''colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd])
#print(np.asarray(pcd.points))'''
print(labels)

