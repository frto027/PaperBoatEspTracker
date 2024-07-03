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

while True:
    data, addr = s.recvfrom(1024)

    frame_dropper += 1
    if frame_dropper < 10:
        continue
    else:
        frame_dropper = 0

    if len(data) % (4*8) == 0:
        sensor_count = len(data)//(4*8)
        for sub_sensor in range(sensor_count):
            (mh,mv,sh,sv) = struct.unpack("@QQQQ",data[sub_sensor * 4 * 8:(sub_sensor+1)*4*8])
            sensors[sub_sensor] = (mh,mv,sh,sv)



        img = np.zeros((800,800,3))
        idx = 0
        for mh,mv,sh,sv in sensors:
            (x,y) = (mh*800/40/8333,mv*800/40/8333)

            x = x - dx
            y = y - dy

            x *= scale
            y *= scale
            # x = x * 10
            # y = y * 10

            # print("% 6.3f % 6.3f % 6.3f % 6.3f(%6.3f %6.3f)" %(mh/40,mv/40,sh/40,sv/40, x,y), end=' ')
            x=int(x)
            y=int(y)
            cv2.circle(img, (x,800-y), 5, colors[idx % len(colors)])
            idx += 1

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

