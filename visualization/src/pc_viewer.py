import argparse
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import os
import yaml

# Load a .bin file and prepare the point cloud for plotting
def load_bin_file(file_path, z_min, z_max):
    points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    xyz = points[:, :3]
    z_clipped = np.clip(xyz[:, 2], z_min, z_max)
    z_mapped = (z_clipped - z_min) / (z_max - z_min)
    colormap = plt.cm.viridis
    colors = colormap(z_mapped)[:, :3]
    return xyz, colors

def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str,
                        help="Path to the YAML configuration file")
    args = parser.parse_args()
    config_file = args.config_file

    # Load the config file
    with open(config_file, 'r') as file:
        cfg = yaml.safe_load(file)

    pose_file = cfg['pose_file']
    pc_folder_path = cfg['pc_folder_path']
    z_min = cfg['z_min']
    z_max = cfg['z_max']
    pt_size = cfg['pt_size']
    voxel_size = cfg['voxel_size']
    visualize_scans = cfg['visualize_scans']
    make_map = cfg['make_map']
    map_file_name = cfg['map_output_file']

    # Load the .bin scan files and the pose estimates
    # The pose estimates must be in the KITTI format
    pc_files = sorted(os.listdir(pc_folder_path))
    pose_estimates = np.loadtxt(pose_file)

    # Create Open3D PointCloud containers for plotting
    voxel_map = o3d.geometry.PointCloud()
    point_cloud = o3d.geometry.PointCloud()
    map_with_new_pts = o3d.geometry.PointCloud()

    # Set the initial axis size using the first scan
    xyz, colors = load_bin_file(pc_folder_path+"/"+pc_files[0], z_min, z_max)
    point_cloud.points = o3d.utility.Vector3dVector(xyz)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    if visualize_scans:
        # Create a visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(point_cloud) 
        vis.add_geometry(voxel_map)
        render_option = vis.get_render_option()
        render_option.point_size = pt_size

        # Set the camera viewpoint
        front = [1, 1, 1]
        up = [0, 0, 1]
        view_control = vis.get_view_control()
        view_control.set_front(front)
        view_control.set_up(up)
        view_control.set_zoom(1.0)
        vis.poll_events()
        vis.update_renderer()

    # Loop over all point cloud files
    mapNotSet = True
    counter = 0
    for file_name in pc_files:
        print(f"\rFile: {file_name}", end="", flush=True)

        # Load the point cloud file
        xyz, colors = load_bin_file(pc_folder_path+"/"+file_name, z_min, z_max)
        point_cloud.points = o3d.utility.Vector3dVector(xyz)
        point_cloud.colors = o3d.utility.Vector3dVector(colors)

        # Construct the homogeneous pose matrix
        pose = np.eye(4)
        pose[0,:] = pose_estimates[counter][0:4]
        pose[1,:] = pose_estimates[counter][4:8]
        pose[2,:] = pose_estimates[counter][8:]
        point_cloud.transform(pose)

        # Construct a voxelized map using the sequential point cloud data
        if mapNotSet:
            downsampled_pcd = point_cloud.voxel_down_sample(voxel_size)
            mapNotSet = False
        else:
            map_with_new_pts.points = o3d.utility.Vector3dVector(
                np.vstack((np.asarray(point_cloud.points),
                        np.asarray(downsampled_pcd.points))))
            downsampled_pcd = map_with_new_pts.voxel_down_sample(voxel_size)
        xyz = np.asarray(downsampled_pcd.points)
        z_clipped = np.clip(xyz[:, 2], z_min, z_max)
        z_mapped = (z_clipped - z_min) / (z_max - z_min)
        colormap = plt.cm.viridis
        colors = colormap(z_mapped)[:, :3]
        downsampled_pcd.colors = o3d.utility.Vector3dVector(colors)

        # Update the voxel map points
        voxel_map.points = o3d.utility.Vector3dVector(
            np.asarray(downsampled_pcd.points))
        voxel_map.colors = o3d.utility.Vector3dVector(colors)
        
        if visualize_scans:
            # Update the visualizer
            # Set the camera at the current pose
            view_control.set_lookat([pose[0][3], pose[1][3], pose[2][3]])
            vis.update_geometry(point_cloud)
            vis.update_geometry(voxel_map)
            vis.poll_events()
            vis.update_renderer()
        counter += 1
        
    if visualize_scans:
        vis.destroy_window()

    if make_map:
        o3d.io.write_point_cloud(map_file_name, voxel_map, compressed=True)


if __name__=="__main__":
    main()