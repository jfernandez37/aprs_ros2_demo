#!/usr/bin/env python3

import socket
import re

CONFIG_FANUC = {
    "port":5001,
    "host_ip":"viz.campus.nist.gov",
}

CONFIG_TEACH = {
    "port":6001,
    "host_ip":"viz.campus.nist.gov",
}

CONFIG_MOTOMAN = {
    "port":5002,
    "host_ip":"viz.campus.nist.gov",
}

class VisionUtility():
    def __init__(self, config):
        if config == "CONFIG_FANUC":
            self.config = CONFIG_FANUC
        elif config == "CONFIG_TEACH":
            self.config = CONFIG_TEACH
        elif config == "CONFIG_MOTOMAN":
            self.config = CONFIG_MOTOMAN
        self.buffer = b''
    
    def get_object_locations(self) -> list[tuple[str, float, float, float, str]]:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.config['host_ip'], self.config['port']))

        while b'\n\r' not in self.buffer:
            data = s.recv(1024)
            self.buffer += data
        line, _ , self.buffer = self.buffer.partition(b'\n\r')
        line.decode()
        raw_string = line.decode('ascii')

        all_items = raw_string.split(',')

        part_locations = []

        while len(all_items) >= 6:
            name, rotation, x, y, _, item_type = all_items[:6]

            try:
                rotation = float(rotation)
            except ValueError:
                rotation = 0.0

            try:
                x = float(x) * 0.001
            except ValueError:
                x = 0.0

            try:
                y = float(y) * 0.001
            except ValueError:
                y = 0.0
            
            if y != 0.0 and x != 0.0:
                part_locations.append((name, rotation, x, y, item_type))
            
            all_items = all_items[6:]
        
        s.close()

        return part_locations