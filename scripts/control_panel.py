#!/usr/bin/env python

import rclpy
import numpy as np
import threading

import tkinter as tk
import queue

from rclpy.node import Node

from options.utils import get_shared_param
from auto_scanner.srv import NodeReady, CaptureState
from std_msgs.msg import Bool

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=fn, daemon=True, args=args, kwargs=kwargs).start()
    return wrapper

class ControlPanelWindow(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.button_state = "Capture"
        self.button_state2 = "Start Recon."

        self.__wmsize = (256, 400)
        self.__node_panels = {}

        self.wm_title("AutoScan - Control Panel")
        self.geometry("%sx%s"%self.__wmsize)
        self.resizable(False, False)

        self.rowconfigure(0, weight=2)
        self.rowconfigure(1, weight=1)

        self.__setup_layout()

        self.notify_state_change = None

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

        bottom_innerframe = tk.Frame(bottom_frame)
        bottom_innerframe.place(relx=0.5, rely=0.5, anchor="center")
        
        self.__start_button = tk.Button(bottom_innerframe, text=self.button_state, command=self.__on_button_clicked)
        self.__start_button.pack(side=tk.LEFT)

        self.__start_recon_button = tk.Button(bottom_innerframe, text=self.button_state2, command=self.__on_start_recon_clicked)
        self.__start_recon_button.pack(side=tk.LEFT)

    def __on_button_clicked(self):
        if self.notify_state_change is not None:
            preview_state = "Stop" if self.button_state == "Capture" else "Capture"
            can_change = self.notify_state_change(("capture", self.button_state == "Capture"))
            if not can_change:
                return
            self.button_state = preview_state
            self.__start_button.config(text=self.button_state)

    def __on_start_recon_clicked(self):
        if self.notify_state_change is not None:
            preview_state = "Stop Recon." if self.button_state2 == "Start Recon." else "Start Recon."
            can_change = self.notify_state_change(("recon", self.button_state2 == "Start Recon."))
            if not can_change:
                return
            self.button_state2 = preview_state
            self.__start_recon_button.config(text=self.button_state2)

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
        
        self.declare_parameter("node_list", [""])
        self.barrier = {}
        for node_name in self.get_parameter("node_list").value:
            self.barrier[node_name] = False
        
        self.create_service(NodeReady, "/control_panel/inform_ready", self.__on_node_ready)
        self.recon_state = self.create_publisher(Bool, "/control_panel/recon_state", 5)
        self.capture_state = self.create_client(CaptureState, "/gstreaming/capturing")
        #self.create_timer(0.5, callback=self.__check_ros_msg)

        self.__queue2ui = queue.Queue()
        self.__lock_ui = threading.Lock()

        # self.capture_state.wait_for_service(30)
        # if not self.capture_state.service_is_ready():
        #     self.get_logger().warn("no gstreaming node detected")

        self.__ui_loop()
    
    @threaded
    def __ui_loop(self):    
        self.__control_panel_ui = ControlPanelWindow()
        for node in self.barrier.keys():
            self.__control_panel_ui.add_node(node)
        self.__control_panel_ui.notify_state_change = self.__send_msg_to_ros
        self.__control_panel_ui.after(1000, self.__check_ui_msg)
        self.__control_panel_ui.mainloop()

    def __send_msg_to_ros(self, x):
        t, state = x
        self.get_logger().info("%s, %s"%(t, str(state)))
        if t == 'capture':
            self.__change_capture_state(self.capture_state, state)
        elif t == 'recon':
            b = Bool()
            b.data = state
            self.recon_state.publish(b)
        return True

    def __send_msg_to_ui(self, x):
        with self.__lock_ui:
            self.__queue2ui.put(x)

    def __check_ui_msg(self):
        with self.__lock_ui:
            while not self.__queue2ui.empty():
                node, state = self.__queue2ui.get()
                self.__control_panel_ui.change_node_state(node, state)
        self.__control_panel_ui.after(1000, self.__check_ui_msg)

    def __change_capture_state(self, client, state: bool):
        state_msg = CaptureState.Request()
        state_msg.desire_state = state
        future = client.call_async(state_msg)
        #rclpy.spin_until_future_complete(self, future)

    def __on_node_ready(self, req: NodeReady.Request, resp: NodeReady.Response):
        self.get_logger().info("ready: %s"%(req.node_name))
        if req.node_name in self.barrier and not self.barrier[req.node_name]:
            self.barrier[req.node_name] = True
            self.__send_msg_to_ui((req.node_name, True))
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