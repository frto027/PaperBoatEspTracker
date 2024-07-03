import struct
import socket

import cv2
import numpy as np


import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0",9001))

range_x_min = 800
range_x_max = 0
range_y_min = 800
range_y_max = 0

sensors:list[tuple[int,int,int,int]] = [(0,0,0,0)]*3


colors = [
    [128,0,0],
    [0,128,0],
    [0,0,128]
]

frame_dropper = 0

dx = 0
dy = 0
scale = 1

STATION_COUNT = 2
AXIS_COUNT = 2
AXIS_VALUE_COUNT = 3

class Payload:
    def __init__(self, payload:bytes) -> None:
        assert(len(payload) % (STATION_COUNT * AXIS_COUNT * (8+ 8 * (AXIS_VALUE_COUNT))) == 0)
        self.payload = payload
    def sensor_count(self)-> int:
        return len(self.payload) // (STATION_COUNT*AXIS_COUNT* (8+ 8 * (AXIS_VALUE_COUNT)))
    def get_axis(self, sensor_id, axis) -> list[tuple[int,int]]: # tuple[start, width]
        ary = self.payload[sensor_id * AXIS_COUNT * (8+ 8 * (AXIS_VALUE_COUNT)) + axis * (8+ 8 * (AXIS_VALUE_COUNT)):]
        ary = ary[:8 + 8 * AXIS_VALUE_COUNT]
        (count,) = struct.unpack("@i", ary[:4])
        ret = []
        for i in range(count):
            ret.append(struct.unpack("@ii", ary[4+4+i*8:4+4+i*8+8]))
        print(ret, sensor_id, axis,ary)
        return ret

GSCALE = 800/40/8333
while True:
    data, addr = s.recvfrom(1024)

    # frame_dropper += 1
    # if frame_dropper < 10:
    #     continue
    # else:
    #     frame_dropper = 0
    
    payload = Payload(data)

    sensor_count = payload.sensor_count()

    img = np.zeros((800,800,3))
    for sensor_id in range(sensor_count):
        # print(sensor_id)
        mhs = payload.get_axis(sensor_id, 0)
        mvs = payload.get_axis(sensor_id, 1)

        for (hstart, hwidth) in mhs:
            for (vstart, vwidth) in mvs:
                cv2.rectangle(img,
                                [int((-dx + hstart) * scale * GSCALE), int((-dy + vstart)*scale * GSCALE)],
                                [int((-dx + hstart+hwidth)*scale * GSCALE), int((-dy + vstart+vwidth)*scale * GSCALE)],
                                colors[sensor_id % len(colors)]
                                )

    # print("",end='\r')

    cv2.imshow('preview', img)
    k = cv2.waitKey(1)
    if k != -1:
        if k == 119:#w
            dy -= 20
        if k == 115:#s
            dy += 20
        if k == 97:#a
            dx -= 20
        if k == 100:#d
            dx += 20
        if k == 113:#q
            scale = scale - 1
            if scale < 1:
                scale = 1
        if k == 101:#e
            scale = scale + 1
        # print(k)

