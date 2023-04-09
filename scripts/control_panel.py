#!/usr/bin/env python

import rclpy
import numpy as np
import threading
import time

import tkinter as tk
from tkinter import ttk
from PIL import ImageTk, Image
import cv2
import open3d as o3d

from rclpy.node import Node
from cv_bridge import CvBridge

from options.utils import get_shared_param, pose_msg2matrix44
from auto_scanner.srv import NodeReady, CaptureState
from auto_scanner.msg import PosedRGBD

def threaded(fn):
    def wrapper(*args, **kwargs):
        threading.Thread(target=fn, args=args, kwargs=kwargs).start()
    return wrapper

class ControlPanelWindow(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.__wmsize = (256, 400)
        self.__node_panels = {}

        self.wm_title("AutoScan - Control Panel")
        self.geometry("%sx%s"%self.__wmsize)
        self.resizable(False, False)

        self.rowconfigure(0, weight=2)
        self.rowconfigure(1, weight=1)

        self.__setup_layout()

        self.notify_state_change = None
        self.button_state = "Start"

    def __setup_layout(self):
        master_frame = tk.Frame(self, width=self.__wmsize[0], height=self.__wmsize[1])
        master_frame.pack()
        master_frame.pack_propagate(False)

        self.__node_state_frame = tk.Frame(master_frame, height=self.__wmsize[1] - 50, width=256)
        self.__node_state_frame.pack(side=tk.TOP)
        self.__node_state_frame.pack_propagate(False)
        bottom_frame = tk.Frame(master_frame, height=50, width=256)
        bottom_frame.pack(side=tk.TOP)
        bottom_frame.pack_propagate(False)
        self.__start_button = tk.Button(bottom_frame, text="Start", command=self.__on_button_clicked)
        self.__start_button.place(relx=0.5, rely=0.5, anchor="center")

    def __on_button_clicked(self):
        if self.notify_state_change is not None:
            preview_state = "Stop" if self.button_state == "Start" else "Start"
            can_change = self.notify_state_change(self.button_state == "Start")
            if not can_change:
                return
            self.button_state = preview_state
            self.__start_button.config(text=self.button_state)

    def add_node(self, name):
        if name in self.__node_panels:
            return
        container = tk.Frame(self.__node_state_frame, width=self.__wmsize[0], height=16, padx=10)
        container.pack_propagate(False)
        indicator = tk.Frame(container, width=16, height=16, bg='red')
        indicator.pack(side=tk.LEFT)
        node_name = tk.Label(container, text=name, padx=5)
        node_name.pack(side=tk.LEFT)

        container.pack(side=tk.TOP, ipady=10)

        self.__node_panels[name] = indicator
    
    def change_node_state(self, name, state):
        if name not in self.__node_panels:
            return
        indicator: tk.Frame = self.__node_panels[name]
        indicator["bg"] = "red" if not state else "green"

class ControlPanel(Node):
    def __init__(self):
        super().__init__("control_panel")
        self.shared_param = get_shared_param(self)
        self.__control_panel_ui = ControlPanelWindow()
        
        self.declare_parameter("node_list", [])
        self.barrier = {}
        for node_name in self.get_parameter("node_list").value:
            self.barrier[node_name] = False
            self.__control_panel_ui.add_node(node_name)
        
        
        self.create_service(NodeReady, "inform_ready", lambda req, resp: self.__on_node_ready(req, resp))
        self.capture_state = self.create_client(CaptureState, "capture_state")

        self.__control_panel_ui.notify_state_change = lambda x: self.__publish_state_change(x)
        self.__control_panel_ui.mainloop()
        self.__navigator_ui.start_navigator_loop(30)

    def __change_capture_state(self, state: bool):
        state_msg = CaptureState.Request()
        state_msg.desire_state = state
        future = self.capture_state.call_async(state_msg)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                break
    
    def __on_node_ready(self, req: NodeReady.Request, resp: NodeReady.Response):
        if req.node_name in self.barrier and not self.barrier[req.node_name]:
            self.barrier[req.node_name] = True
            self.__control_panel_ui.change_node_state(req.node_name, True)
            resp.ack_name = req.node_name
        else:
            resp.ack_name = "invalid"
        return resp
    
    def __publish_state_change(self, state):
        if np.all([x for x in self.barrier.values()]):
            self.__change_capture_state(state)
        return True
    
    def release(self):
        self.__control_panel_ui.destroy()
            

def main():
    rclpy.init()

    control_panel = ControlPanel()
    rclpy.spin(control_panel)

    control_panel.release()

    rclpy.shutdown()

if __name__ == "__main__":
    main()