from simplerecon import SimpleRecon
import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt

if __name__ == "__main__":
    srecon = SimpleRecon(
        (1280, 720), (1280, 720), 
        "data/simplerecon/hero_model.ckpt")
    
    img = cv2.imread("output.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    srecon.setup_intrinsic(382, 382, 320, 236)
    for i in range(2):
        depth = srecon.process_frame(img, np.identity(4))

        dimg = cv2.resize(depth, (1280, 720))
        plt.imshow(dimg)
        plt.show()
    