import numpy as np
import torch
import torch.nn.functional as F
import torchvision.transforms.functional as TF

from queue import Queue
from utils.geometry_utils import rotx, pose_distance
from utils.generic_utils import imagenet_normalize, to_gpu
from depth_model import DepthModel

class FrameHistory:
    def __init__(self, store_count=5) -> None:
        self.__queue = Queue(store_count)
        self.__count = store_count
    
    def build_complete(self):
        return len(self.__queue) >= self.__count

    def add_history(self, frame):
        if (len(self.__queue) >= self.__count):
            self.__queue.get()
        self.__queue.put(frame)
        self.__queue

    def get_history(self):
        return list(self.__queue)

class SimpleRecon:
    def __init__(self, depth_res, frame_res, weights) -> None:
        self.depth_width, self.depth_height = depth_res
        self.img_width, self.img_height = frame_res
        self.frame_history = FrameHistory(store_count=7)
        
        self.model = DepthModel.load_from_checkpoint(weights, args=None)
        self.model = self.model.cuda().eval()

    def setup_intrinsic(self, cam_K):
        K = cam_K.clone()
        K[0] *= (self.depth_width/self.img_width) 
        K[1] *= (self.depth_height/self.img_height)
        
        self.__camK_data = {}
        # Get the intrinsics of all the scales
        for i in range(5):
            K_scaled = K.clone()
            K_scaled[:2] /= 2 ** i
            invK_scaled = np.linalg.inv(K_scaled)
            self.__camK_data[f"K_s{i}_b44"] = K_scaled
            self.__camK_data[f"invK_s{i}_b44"] = invK_scaled
    
    def __get_extrinsic(self, world_T_cam):
        # TODO should we do a coordinate system conversion?
        world_T_cam = world_T_cam.numpy()
        rot_mat = world_T_cam[:3,:3]
        trans = world_T_cam[:3,3]

        rot_mat = rotx(-np.pi / 2) @ rot_mat
        trans = rotx(-np.pi / 2) @ trans

        world_T_cam[:3, :3] = rot_mat
        world_T_cam[:3, 3] = trans
        
        world_T_cam = world_T_cam
        cam_T_world = np.linalg.inv(world_T_cam)

        return world_T_cam, cam_T_world
    
    def __get_frame(self, rgb, pose):
        output_dict = {}
        world_T_cam, cam_T_world = self.__get_extrinsic(pose)

        image = TF.to_tensor(rgb)
        image = imagenet_normalize(image)

        output_dict.update({
            "image_b3hw": image,
            "world_T_cam_b44": world_T_cam,
            "cam_T_world_b44": cam_T_world,
        })

        output_dict.update(self.__camK_data)

        return output_dict
    
    def __stack_src_data(self, src_data):
        """ Stacks source image data into tensors. """

        tensor_names = src_data[0].keys()
        stacked_src_data = {}
        for tensor_name in tensor_names:
            if "frame_id_string" in tensor_name: 
                stacked_src_data[tensor_name] = [t[tensor_name] for t in 
                                                                    src_data]
            else:
                stacked_src_data[tensor_name] = np.stack([t[tensor_name] 
                                                    for t in src_data], axis=0)

        return stacked_src_data
    
    def get_depth(self, cur_data, src_data):
        cur_data = to_gpu(cur_data, key_ignores=["frame_id_string"])
        src_data = to_gpu(src_data, key_ignores=["frame_id_string"])

        outputs = self.model.forward(cur_data, src_data)
        torch.cuda.synchronize()

        upsampled_depth_pred_b1hw = F.interpolate(
                                outputs["depth_pred_s0_b1hw"], 
                                size=(self.depth_width, self.depth_height),
                                mode="nearest",
                            )
        
        return upsampled_depth_pred_b1hw[0,0,:,:]

    def process_frame(self, rgb, pose):
        cur_data = self.__get_frame(rgb, pose)
        if not self.frame_history.build_complete():
            self.frame_history.add_history(cur_data)
            return False, None
        
        src_data_list = self.frame_history.get_history()
        src_data = self.__stack_src_data(src_data_list)

        src_world_T_cam = torch.tensor(src_data["world_T_cam_b44"])
        cur_cam_T_world = torch.tensor(cur_data["cam_T_world_b44"])

        # Compute cur_cam_T_src_cam
        cur_cam_T_src_cam = cur_cam_T_world.unsqueeze(0) @ src_world_T_cam

        # get penalties.
        frame_penalty_k, _, _ = pose_distance(cur_cam_T_src_cam)

        # order based on indices
        indices = torch.argsort(frame_penalty_k).tolist()
        src_data_list = [src_data_list[index] for index in indices]

        # stack again
        src_data = self.__stack_src_data(src_data_list)

        self.frame_history.add_history(cur_data)

        return self.get_depth(cur_data, src_data)
