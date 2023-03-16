import numpy as np
import torch
import torch.nn.functional as F
import torchvision.transforms.functional as TF
import cv2

from collections import deque
from .utils.geometry_utils import rotx, pose_distance
from .utils.generic_utils import imagenet_normalize, to_gpu
from .depth_model import DepthModel

def deep_copy(data):
        data_cpy = {}
        for k, v in data.items():
            data_cpy[k] = torch.clone(data[k])
        return data_cpy

class FrameHistory:
    def __init__(self, store_count=5) -> None:
        self.__queue = deque(maxlen=store_count)
        self.__count = store_count
    
    def build_complete(self):
        return len(self.__queue) >= self.__count

    def add_history(self, frame):
        self.__queue.popleft()
        self.__queue.append(deep_copy(frame))

    def populate(self, frame):
        while not self.build_complete():
            self.__queue.append(deep_copy(frame))

    def get_history(self):
        return [deep_copy(x) for x in self.__queue]

class SimpleRecon:
    def __init__(self, depth_res, frame_res, weights) -> None:
        torch.set_grad_enabled(False)

        self.depth_width, self.depth_height = depth_res
        self.img_width, self.img_height = frame_res
        
        self.model = DepthModel.load_from_checkpoint(weights)
        self.model = self.model.cuda().eval()
        self.frame_history = FrameHistory(store_count=self.model.run_opts.model_num_views - 1)


    def setup_intrinsic(self, fx, fy, cy, cx):
        K = torch.eye(4, dtype=torch.float32)
        K[0, 0] = float(fx)
        K[1, 1] = float(fy)
        K[0, 2] = float(cx)
        K[1, 2] = float(cy)
        K[0] *= (self.depth_width/self.img_width) 
        K[1] *= (self.depth_height/self.img_height)
        
        self.__camK_data = {}
        # Get the intrinsics of all the scales
        for i in range(5):
            K_scaled = K.clone()
            K_scaled[:2] /= 2 ** i
            invK_scaled = np.linalg.inv(K_scaled)
            self.__camK_data[f"K_s{i}_b44"] = K_scaled
            self.__camK_data[f"invK_s{i}_b44"] = torch.tensor(invK_scaled)
    
    def __get_extrinsic(self, world_T_cam):
        # TODO should we do a coordinate system conversion?
        rot_mat = world_T_cam[:3,:3]
        trans = world_T_cam[:3,3]

        rot_mat = rotx(-np.pi / 2) @ rot_mat
        trans = rotx(-np.pi / 2) @ trans

        world_T_cam[:3, :3] = rot_mat
        world_T_cam[:3, 3] = trans
        
        world_T_cam = world_T_cam
        cam_T_world = np.linalg.inv(world_T_cam)

        return torch.tensor(world_T_cam), torch.tensor(cam_T_world)
    
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

        output_dict.update(deep_copy(self.__camK_data))

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
                stacked_src_data[tensor_name] = torch.tensor(np.stack([t[tensor_name] 
                                                    for t in src_data], axis=0))

        return stacked_src_data

    def __to_batch_size1(self, data):
        for k, v in data.items():
            data[k] = data[k].unsqueeze_(0)
        return data
    
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
        
        return upsampled_depth_pred_b1hw[0,0,:,:].cpu().detach().numpy()

    def process_frame(self, rgb, pose):
        rgb = cv2.resize(rgb, (self.model.run_opts.image_width, self.model.run_opts.image_height))
        cur_data = self.__get_frame(rgb, pose)
        if not self.frame_history.build_complete():
            self.frame_history.populate(cur_data)
        
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

        cur_data = self.__to_batch_size1(cur_data)
        src_data = self.__to_batch_size1(src_data)
        return self.get_depth(cur_data, src_data)

