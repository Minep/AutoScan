import open3d as o3d
import numpy as np
import cv2
import torch

from simplerecon.utils.generic_utils import reverse_imagenet_normalize

class Open3DFuser():
    """ 
    Wrapper class for the open3d fuser. 
    
    This wrapper does not support fusion of tensors with higher than batch 1.
    """
    def __init__(
            self, 
            intrinsic_components,
            depth_size,
            fusion_resolution=0.02, 
            max_fusion_depth=256, 
        ):
        self.fusion_max_depth = max_fusion_depth

        fx, fy, cx, cy = intrinsic_components
        self.width, self.height = depth_size
        self.K = o3d.camera.PinholeCameraIntrinsic(
                                width=self.width, 
                                height=self.height, 
                                fx=fx, fy=fy,cx=cx, cy=cy)

        self.voxel_size = fusion_resolution * 100
        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=float(self.voxel_size) / 100,
            sdf_trunc=2 * float(self.voxel_size) / 100,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        self.alignment_options = o3d.pipelines.odometry.OdometryOption()
        self.prev_rgbd = None
        self.gpcd = None
        self.prev_T = np.identity(4)
        self.nn = o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size,
                                                max_nn=30)

        self.enable_icp_finetune = False

    def fuse_frames(self, depths_hw, cam_T_world_44, color_hw):
        color_hw = cv2.resize(color_hw, dsize=(self.width, self.height))

        depth_pred = o3d.geometry.Image(depths_hw)
        color_im = o3d.geometry.Image(color_hw.copy(order='C'))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                                        color_im, 
                                        depth_pred, 
                                        depth_scale=1.0,
                                        depth_trunc=self.fusion_max_depth,
                                        convert_rgb_to_intensity=False)        
        T = cam_T_world_44
        if self.enable_icp_finetune:
            if self.gpcd is None:
                self.gpcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.K)  
                self.gpcd = self.preprocess_point_cloud(self.gpcd).transform(T)
            else:
                T = self.local_refine(rgbd, T)
        self.volume.integrate(rgbd, self.K, T)

    def preprocess_point_cloud(self, pcd):
        pcd_down = pcd.voxel_down_sample(self.voxel_size / 2)
        pcd_down.estimate_normals(self.nn)
        return pcd_down
        
    def local_refine(self, rgbd_src, extrinsic_prior):
        pcd_src = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_src, self.K)
        pcd_ds_src = self.preprocess_point_cloud(pcd_src)
        result_icp = o3d.pipelines.registration.registration_icp(
                        pcd_ds_src, self.gpcd, 0.01, extrinsic_prior, 
                        o3d.pipelines.registration.TransformationEstimationPointToPlane())
        T = result_icp.transformation
        self.gpcd += pcd_ds_src.transform(T)
        self.gpcd.estimate_normals(self.nn)
        return T

    def export_mesh(self, path):
        o3d.io.write_triangle_mesh(path, self.volume.extract_triangle_mesh())
    
    def get_mesh(self):
        mesh = self.volume.extract_triangle_mesh()
        return mesh
