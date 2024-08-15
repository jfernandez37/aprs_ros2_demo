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
from PIL import Image
from rclpy.node import Node
import threading
from aprs_interfaces.srv import MoveToNamedPose, Pick, Place

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
        
        # Service variables
        self.service_type = ctk.StringVar()
        self.service_type.set(SERVICE_TYPES[0])
        self.temp_widgets = []
        self.temp_vars = []
        
        
        # Notebook pages
        self.notebook = ttk.Notebook(self.gui)
        
        self.service_call_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.service_call_frame.pack(fill='both', expand=True)
        self.notebook.add(self.service_call_frame, text="Services")
        self.add_service_widgets_to_frame()
        
        self.notebook.grid(pady=10, column=LEFT_COLUMN, columnspan = 2, sticky=tk.E+tk.W+tk.N+tk.S)
        
        

    def add_service_widgets_to_frame(self):
        self.service_call_frame.grid_rowconfigure(0, weight=1)
        self.service_call_frame.grid_rowconfigure(100, weight=1)
        self.service_call_frame.grid_columnconfigure(0, weight=1)
        self.service_call_frame.grid_columnconfigure(10, weight=1)
        
        command_label = ctk.CTkLabel(self.service_call_frame, text="ros2 service call  ")
        command_label.grid(column = FAR_LEFT_COLUMN, row = 2)
        
        service_selection_menu = OptionMenu(self.service_call_frame, self.service_type, *SERVICE_TYPES)
        service_selection_menu.grid(column = LEFT_COLUMN, row = 2)
        
        serice_type_label = ctk.CTkLabel(self.service_call_frame, text=f"  {SERVICE_TYPES_DICT[self.service_type.get()]}  ")
        serice_type_label.grid(column = MIDDLE_COLUMN, row = 2)
        self.service_type.trace_add('write', partial(self.update_service_type, serice_type_label))
        self.update_service_type(serice_type_label, None, None, None)
        
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
    
        
def main(args=None):
    rclpy.init(args=args)
    print("TEST")
    main_gui = GUI_CLASS()
    main_gui.gui.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()