#! /usr/bin/env python

import rclpy
try:
    import customtkinter as ctk
except:
    print("ERROR: customtkinter not installed\nRun \"pip install customtkinter\" to fix this issue")
    quit()
try:
    from tktooltip import ToolTip
except:
    print("ERROR: tkinter-tooltip not installed\nRun \"pip install tkinter-tooltip\" to fix this issue")
    quit()  
from tkinter import *
import tkinter as tk
from tktooltip import ToolTip
from tkinter import ttk, filedialog
from functools import partial
from PIL import Image, ImageTk
from rclpy.node import Node
import threading
from aprs_interfaces.srv import MoveToNamedPose, Pick, Place, GenerateInitState, GeneratePlan
from aprs_interfaces.action import ExecutePlan
from aprs_interfaces.msg import Tray, Trays
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from industrial_msgs.msg import RobotStatus
import os
from rclpy.executors import MultiThreadedExecutor
from difflib import SequenceMatcher
from rclpy.action import ActionClient
import numpy as np
from copy import deepcopy
import cv2

FRAMEWIDTH=700
FRAMEHEIGHT=900
FAR_LEFT_COLUMN = 1
LEFT_COLUMN=2
MIDDLE_COLUMN = 3
RIGHT_COLUMN = 4
FAR_RIGHT_COLUMN = 5

SERVICE_TYPES = ["MoveToNamedPose", "Pick", "Place"]
SERVICE_TYPES_DICT = {t : f"aprs_interfaces/srv/{t}" for t in SERVICE_TYPES}
SERVICE_OBJ = {"MoveToNamedPose": MoveToNamedPose,
               "Pick": Pick,
               "Place": Place}

ROBOTS = ["fanuc", "motoman"]

class GUI_CLASS(Node):
    def __init__(self):
        super().__init__("aprs_demo_gui")
        
        ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
        ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
        
        self.gui = ctk.CTk()
        
        self.gui.title("NIST APRS DEMO")

        self.gui.grid_rowconfigure(0, weight=1)
        self.gui.grid_rowconfigure(100, weight=1)
        self.gui.grid_columnconfigure(0, weight=1)
        self.gui.grid_columnconfigure(6, weight=1)
        
        s = ttk.Style()
        s.theme_use('clam')
        s.configure('TNotebook', font='Arial Bold')
        
        # Demo variables
        self.robots_to_use = {robot: ctk.StringVar() for robot in ROBOTS}
        
        # Service variables
        self.service_type = ctk.StringVar()
        self.service_type.set(SERVICE_TYPES[0])
        self.temp_widgets = []
        self.temp_vars = []
        
        self.available_topics = self.get_avalailable_topics()
        
        # Map variables
        self.fanuc_trays_info = None
        self.motoman_trays_info = None
        self.teach_trays_info = None
        
        # Live video variables
        self.most_recent_frames = {"floor_robot": None, "ceiling_robot": None}
        self.bridge = CvBridge()

        # Connection to robots
        self.connections = {robot: ctk.IntVar() for robot in ROBOTS}
        for robot in ROBOTS:
            self.connections[robot].set(0)
        for robot, var in self.connections.items():
            var.trace_add('write', partial(self.update_connection_label, robot))

        # ROS Subscriptions
        self.fanuc_trays_sub_ = self.create_subscription(
            Trays,
            '/fanuc/trays_info',
            self.fanuc_trays_cb_,
            10
        )
        self.motoman_trays_sub_ = self.create_subscription(
            Trays,
            '/motoman/trays_info',
            self.motoman_trays_cb_,
            10
        )
        self.teach_trays_sub_ = self.create_subscription(
            Trays,
            '/teach/trays_info',
            self.teach_trays_cb_,
            10
        )
        
        # Cameras
        self._floor_robot_cam_sub = self.create_subscription(ImageMsg, 
                                                             '/ariac/sensors/floor_robot_camera/rgb_image', 
                                                             self.floor_robot_image_cb, 
                                                             rclpy.qos.qos_profile_sensor_data)
        
        self._ceiling_robot_cam_sub = self.create_subscription(ImageMsg, 
                                                             '/ariac/sensors/ceiling_robot_camera/rgb_image', 
                                                             self.ceiling_robot_image_cb, 
                                                             rclpy.qos.qos_profile_sensor_data)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS service clients
        self.generate_init_state_client = self.create_client(
            GenerateInitState,
            '/generate_init_state'
        )
        
        self.generate_plan_client = self.create_client(
            GeneratePlan,
            "/generate_plan"
        )
        
        # Ros action clients
        self.execute_plan_action_client = ActionClient(self, ExecutePlan, "/execute_plan")
        
        # Notebook pages
        self.notebook = ttk.Notebook(self.gui)
        
        self.demo_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.demo_frame.pack(fill='both', expand=True)
        self.notebook.add(self.demo_frame, text="Run Demo")
        self.add_demo_widgets_to_frame()

        self.topics_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.topics_frame.pack(fill='both', expand=True)
        self.notebook.add(self.topics_frame, text="Topics")
        self.add_topic_widgets_to_frame()
        
        self.service_call_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.service_call_frame.pack(fill='both', expand=True)
        self.notebook.add(self.service_call_frame, text="Services")
        self.add_service_widgets_to_frame()
        
        self.connections_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.connections_frame.pack(fill='both', expand=True)
        self.notebook.add(self.connections_frame, text="Connections")
        self.robot_connection_status_labels = {robot: ctk.CTkLabel(self.connections_frame, text=("Connected" if self.connections[robot].get()==1 else "Not Connected")) for robot in ROBOTS}
        self.add_connections_widgets_to_frame()

        self.init_plan_execute_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.init_plan_execute_frame.pack(fill='both', expand=True)
        self.notebook.add(self.init_plan_execute_frame, text="Init Plan Execute")
        self.add_init_plan_execute_widgets_to_frame()
        
        self.motoman_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.motoman_frame.pack(fill='both', expand=True)
        self.notebook.add(self.motoman_frame, text="Motoman")
        self.add_motoman_widgets_to_frame()
        
        self.fanuc_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.fanuc_frame.pack(fill='both', expand=True)
        self.notebook.add(self.fanuc_frame, text="Fanuc")
        self.add_fanuc_widgets_to_frame()
        
        self.teach_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.teach_frame.pack(fill='both', expand=True)
        self.notebook.add(self.teach_frame, text="Teach Table")
        self.add_teach_widgets_to_frame()
        
        self.live_video_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.live_video_frame.pack(fill='both', expand=True)
        self.notebook.add(self.live_video_frame, text="Live video")
        self.update_video_var = ctk.IntVar(value=0)
        self.add_live_video_widgets_to_frame()
        
        self.notebook.grid(pady=10, column=LEFT_COLUMN, columnspan = 2, sticky=tk.E+tk.W+tk.N+tk.S)
        
        self.connection_timer = self.create_timer(5, self.connection_timer_cb_)

    def add_demo_widgets_to_frame(self):
        self.demo_frame.grid_rowconfigure(0, weight=1)
        self.demo_frame.grid_rowconfigure(100, weight=1)
        self.demo_frame.grid_columnconfigure(0, weight=1)
        self.demo_frame.grid_columnconfigure(10, weight=1)

        current_row = 2

        ctk.CTkLabel(self.demo_frame, text="Select the robots to use:").grid(column = LEFT_COLUMN, columnspan = 3, row = current_row, pady=10)
        current_row+=1

        for robot, var in self.robots_to_use.items():
            ctk.CTkCheckBox(self.demo_frame, text=robot, variable=var, onvalue="1", offvalue = "0", height=1, width=20).grid(column = LEFT_COLUMN, columnspan = 3, row = current_row)
            current_row+=1
    
    def add_topic_widgets_to_frame(self):
        self.topics_frame.grid_rowconfigure(0, weight=1)
        self.topics_frame.grid_rowconfigure(100, weight=1)
        self.topics_frame.grid_columnconfigure(0, weight=1)
        self.topics_frame.grid_columnconfigure(10, weight=1)
        
        current_topic_selection = ctk.StringVar()
        current_topic_selection.set("")
        topic_menu = ctk.CTkComboBox(self.topics_frame, variable=current_topic_selection, values=self.available_topics)
        topic_menu.grid(column = LEFT_COLUMN, row = 2, pady = 15)
        
        self.topic_sub_frame = ctk.CTkScrollableFrame(self.topics_frame, width = 650, height=600)
        self.topic_sub_frame.grid(column = LEFT_COLUMN, row = 3, columnspan = 3)
        self.topic_sub_frame.bind_all("<Button-4>", self.mouse_wheel_up_current_file)
        self.topic_sub_frame.bind_all("<Button-5>", self.mouse_wheel_down_current_file)
        
        topic_output_label = ctk.CTkLabel(self.topic_sub_frame, text="")
        topic_output_label.pack()
        
        update_button = ctk.CTkButton(self.topics_frame, text="Update", command=partial(self.update_topic_text, current_topic_selection, topic_output_label))
        update_button.grid(column = RIGHT_COLUMN, row = 2)
        
        current_topic_selection.trace_add("write", partial(self.only_show_matching, current_topic_selection, topic_menu))
    
    def only_show_matching(self, current_topic_selection, topic_menu, _, __, ___):
        selection = current_topic_selection.get()
        if selection in self.available_topics:
            topic_menu.configure(values = self.available_topics)
        else:
            options = []
            for topic in self.available_topics:
                if selection in topic:
                    options.append(topic)
                elif SequenceMatcher(None, selection, topic[:len(selection)]).ratio() > 0.5:
                    options.append(topic)
            topic_menu.configure(values = options)
    
    def get_avalailable_topics(self):
        topics = os.popen("ros2 topic list").read().split("\n")
        for topic in ["/rosout","/parameter_events", "/map", ""]:
            if topic in topics:
                topics.remove(topic)
        return topics
        
    def update_topic_text(self, current_topic_selection, topic_output_label):
        topic_output = os.popen(f"ros2 topic echo {current_topic_selection.get()} --once -f").read()
        topic_output = topic_output.replace('\\n', '\n')
        topic_output_label.configure(text = topic_output)
    
    def add_service_widgets_to_frame(self):
        self.service_call_frame.grid_rowconfigure(0, weight=1)
        self.service_call_frame.grid_rowconfigure(100, weight=1)
        self.service_call_frame.grid_columnconfigure(0, weight=1)
        self.service_call_frame.grid_columnconfigure(10, weight=1)
        
        command_label = ctk.CTkLabel(self.service_call_frame, text="ros2 service call  ")
        command_label.grid(column = FAR_LEFT_COLUMN, row = 2)
        
        service_selection_menu = ctk.CTkOptionMenu(self.service_call_frame, variable=self.service_type, values=SERVICE_TYPES)
        service_selection_menu.grid(column = LEFT_COLUMN, row = 2)
        
        service_type_label = ctk.CTkLabel(self.service_call_frame, text=f"  {SERVICE_TYPES_DICT[self.service_type.get()]}  ")
        service_type_label.grid(column = MIDDLE_COLUMN, row = 2)
        self.service_type.trace_add('write', partial(self.update_service_type, service_type_label))
        self.update_service_type(service_type_label, None, None, None)
        
        service_call_button = ctk.CTkButton(self.service_call_frame, text="Call Service", command=self.call_service)
        service_call_button.grid(column = FAR_LEFT_COLUMN, columnspan=6, row = 5, pady=10)
        
    def update_service_type(self, serice_type_label, _, __, ___):
        serice_type_label.configure(text=f"  {SERVICE_TYPES_DICT[self.service_type.get()]}  ")
        
        for widget in self.temp_widgets:
            widget.grid_forget()
            
        self.temp_widgets.clear()
        self.temp_vars.clear()
        
        starting_column = RIGHT_COLUMN
        for arg_name, arg_type in SERVICE_OBJ[self.service_type.get()].Request.get_fields_and_field_types().items():
            self.temp_widgets.append(ctk.CTkLabel(self.service_call_frame, text=f"{arg_name}({arg_type})"))
            self.temp_widgets[-1].grid(column = starting_column, row = 1)
            self.temp_vars = [(arg_name, ctk.StringVar())]
            self.temp_widgets.append(ctk.CTkEntry(self.service_call_frame, textvariable=self.temp_vars[-1][1]))
            self.temp_widgets[-1].grid(column = starting_column, row = 2, padx = 5)
            starting_column += 1
        
    def call_service(self):
        service_call_command = f"ros2 service call {self.service_type.get()} {SERVICE_TYPES_DICT[self.service_type.get()]} "+"\"{" +", ".join([arg_pair[0]+":"+arg_pair[1].get() for arg_pair in self.temp_vars]) + "}\""
        print("Running "+service_call_command)

    def add_connections_widgets_to_frame(self):
        self.connections_frame.grid_rowconfigure(0, weight=1)
        self.connections_frame.grid_rowconfigure(100, weight=1)
        self.connections_frame.grid_columnconfigure(0, weight=1)
        self.connections_frame.grid_columnconfigure(10, weight=1)

        robot_name_labels = []
        for robot in ROBOTS:
            robot_name_labels.append(ctk.CTkLabel(self.connections_frame, text=robot.capitalize()+": "))
            robot_name_labels[-1].grid(column = LEFT_COLUMN, row = 2+ROBOTS.index(robot), padx = 10, pady = 10)
            self.robot_connection_status_labels[robot].grid(column = MIDDLE_COLUMN, row = 2+ROBOTS.index(robot))

    def connection_timer_cb_(self):
        info = os.popen("ls -a").read()
        motoman_found = False
        fanuc_found = False
        for n in info.split('\n'):
            if "motoman" in n:
                motoman_found = True
            elif "fanuc" in n:
                fanuc_found = True
        self.connections["motoman"].set(1 if motoman_found else 0)
        self.connections["fanuc"].set(1 if fanuc_found else 0 )

        self.available_topics = os.popen("ros2 topic list").read().split("\n")
        for topic in ["/rosout","/parameter_events", "/map", ""]:
            if topic in self.available_topics:
                self.available_topics.remove(topic)

    def update_connection_label(self, robot, _, __, ___):
        self.robot_connection_status_labels[robot].configure(text=("Connected" if self.connections[robot].get()==1 else "Not Connected"))
    
    # ======================================================================================
    #                                     Init Plan Execute
    # ======================================================================================
    
    def add_init_plan_execute_widgets_to_frame(self):
        self.init_plan_execute_frame.grid_rowconfigure(0, weight=1)
        self.init_plan_execute_frame.grid_rowconfigure(100, weight=1)
        self.init_plan_execute_frame.grid_columnconfigure(0, weight=1)
        self.init_plan_execute_frame.grid_columnconfigure(10, weight=1)
        
        self.generate_init_state_button = ctk.CTkButton(self.init_plan_execute_frame, text="Generate Init State", command = self.call_generate_init_state)
        self.generate_init_state_button.grid(column = MIDDLE_COLUMN, row = 2, pady = 10)
        
        self.generate_plan_button = ctk.CTkButton(self.init_plan_execute_frame, text="Generate plan", state=DISABLED, command = self.call_generate_plan)
        self.generate_plan_button. grid(column = MIDDLE_COLUMN, row = 3, pady = 10)
        
        self.plan_sub_frame = ctk.CTkScrollableFrame(self.init_plan_execute_frame, width = 650, height=500)
        self.plan_sub_frame.grid(column = LEFT_COLUMN, row = 4, columnspan = 3)
        
        self.plan_label = ctk.CTkLabel(self.plan_sub_frame, text="")
        self.plan_label.pack()
        
        self.execute_plan_button = ctk.CTkButton(self.init_plan_execute_frame, text="Execute plan", state=DISABLED, command=self.execute_plan)
        self.execute_plan_button. grid(column = MIDDLE_COLUMN, row = 5, pady = 10)
        
    
    def call_generate_init_state(self):
        """Generates the init state
        """
        request = GenerateInitState.Request()

        future = self.generate_init_state_client.call_async(request)

        while not future.done():
            pass
        
        response: GenerateInitState.Response = future.result()
        
        self.get_logger().info(f"Generated Init State: {response.status}")
        
        self.generate_init_state_button.grid_forget()
        self.generate_plan_button.configure(state=NORMAL)
    
    def call_generate_plan(self):
        """Generates a plan
        """
        request = GeneratePlan.Request()

        future = self.generate_plan_client.call_async(request)

        while not future.done():
            pass
        
        response: GeneratePlan.Response = future.result()
        
        self.get_logger().info(f"Generated Init State: {response.status}")
        
        self.plan_label.configure(text = response.plan)
        self.execute_plan_button.configure(state=NORMAL)
    
    def execute_plan(self):
        pass
        # self.execute_plan_action_client.wait_for_server()
        
        # execute_plan = ExecutePlan.Goal()
        
        # self.get_logger().info(f"Goal:\n\n{execute_plan}\n\n")
        
        # future = self._action_client.send_goal_async(execute_plan)
        
        # while not future.done():
        #     pass
        
        # future.add_done_callback(self.goal_response_callback)
        
    # ======================================================================================
    #                                       Motoman Tab
    # ======================================================================================
    def add_motoman_widgets_to_frame(self):
        pass
    
    # ======================================================================================
    #                                        Fanuc Tab
    # ======================================================================================
    def add_fanuc_widgets_to_frame(self):
        pass
    
    # ======================================================================================
    #                                     Teach Table Tab
    # ======================================================================================
    def add_teach_widgets_to_frame(self):
        pass
    
    # ======================================================================================
    #                                        Callbacks
    # ======================================================================================
    def fanuc_trays_cb_(self, msg):
        self.motoman_trays_info = msg
    
    def motoman_trays_cb_(self, msg):
        self.motoman_trays_info = msg
    
    def teach_trays_cb_(self, msg):
        self.teach_trays_info = msg
    
    # ======================================================================================
    #                                        Callbacks
    # ======================================================================================
    def add_live_video_widgets_to_frame(self):
        self.live_video_frame.grid_rowconfigure(0, weight=1)
        self.live_video_frame.grid_rowconfigure(100, weight=1)
        self.live_video_frame.grid_columnconfigure(0, weight=1)
        self.live_video_frame.grid_columnconfigure(10, weight=1)
        
        camera_types = [cam for cam in self.most_recent_frames.keys()]
        self.camera_selection = ctk.StringVar(value=camera_types[0])
        
        ctk.CTkLabel(self.live_video_frame, text="Select the camera you would like to view").grid(column=MIDDLE_COLUMN, row=1)
        
        camera_selection_menu = ctk.CTkOptionMenu(self.live_video_frame, variable=self.camera_selection, values=camera_types)
        camera_selection_menu.grid(column=MIDDLE_COLUMN, row=2, pady=10)
        
        self.img_label = ctk.CTkLabel(self.live_video_frame, text="")
        self.img_label.grid(column=MIDDLE_COLUMN, row=3)
        
        self.update_video_var.trace_add("write", self.update_image_label)
        
    def update_image_label(self, _, __, ___):
        cv_image = self.bridge.imgmsg_to_cv2(self.most_recent_frames[self.camera_selection.get()], desired_encoding="passthrough")
        frame_to_show = Image.fromarray(cv_image)
        self.img_label.configure(image=ctk.CTkImage(frame_to_show, size=(640, 480)))
        # self.img_label.configure(text="AFTER")
        
    def floor_robot_image_cb(self, msg: ImageMsg):
        self.most_recent_frames["floor_robot"] = msg
        self.update_video_var.set((self.update_video_var.get()+1)%2)
    
    def ceiling_robot_image_cb(self, msg: ImageMsg):
        self.most_recent_frames["ceiling_robot"] = msg
        self.update_video_var.set((self.update_video_var.get()+1)%2)
    # ======================================================================================
    #                                     General Utilities
    # ======================================================================================
    
    def mouse_wheel_up_current_file(self, event):
        if self.notebook.index("current") == 1:
            self.topic_sub_frame._parent_canvas.yview_scroll(int(-2), "units")
        elif self.notebook.index("current") == 4:
            self.plan_sub_frame._parent_canvas.yview_scroll(int(-2), "units")
    
    def mouse_wheel_down_current_file(self, event):
        if self.notebook.index("current") == 1:
            self.topic_sub_frame._parent_canvas.yview_scroll(int(2), "units")
        elif self.notebook.index("current") == 4:
            self.plan_sub_frame._parent_canvas.yview_scroll(int(2), "units")

def main(args=None):
    rclpy.init(args=args)
    
    main_gui = GUI_CLASS()
    executor = MultiThreadedExecutor()
    executor.add_node(main_gui)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    main_gui.gui.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()