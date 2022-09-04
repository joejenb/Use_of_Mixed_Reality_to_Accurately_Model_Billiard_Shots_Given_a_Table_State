import socket
import math
import json
import time

HOST = '192.168.1.138'  # Standard loopback interface address (localhost)
PORT = 8000        # Port to listen on (non-privileged ports are > 1023)

class Shot_Data():
    def __init__(self):
        self.white_x = 0
        self.white_y = 0
        self.object_x = 0
        self.object_y = 0
        self.radi = 890
        #Color object_colour;
        self.cue_x1 = 524
        self.cue_y1 = 524
        self.cue_x2 = 724
        self.cue_y2 = 524

HOST = "192.168.1.164"  # The server's hostname or IP address
PORT = 8000        # The port used by the server

to_send = Shot_Data()
angle = 0
print(to_send.__dict__)
while True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        angle += 1
        angle = angle % 360
        to_send.cue_y2 = to_send.cue_y1 + (200 * math.sin((angle/360) * 2 * math.pi))
        to_send.cue_x2 = to_send.cue_x1 + (200 * math.cos((angle/360) * 2 * math.pi))
        json_data = bytes(json.dumps(to_send.__dict__), encoding='utf-8')
        print(json_data)

        s.connect((HOST, PORT))
        s.sendall(json_data)
        time.sleep(0.003)
