import socket
import glob
#from PIL import Image
import numpy as np
import io
import threading
import collections
import cv2
from utils import ArducamUtils
import os

# image_paths = glob.glob('*png')
# images = []

# print(image_paths)
# for path in image_paths:
#     with Image.open(path) as img:
#         image = img.resize((1280*4,800))
#         images.append(np.array(image))

# print("Done")
# print(len(images))

HOST = '172.20.10.1'  # The server's hostname or IP address
PORT = 1000        # The port used by the server

fifo = collections.deque(maxlen=150)

import sched
import time

scheduler = sched.scheduler(time.time, time.sleep)

def fourcc(a, b, c, d):
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc('R','G','G','B'))

    arducam_utils = ArducamUtils(0)

    _, firmware_version = arducam_utils.read_dev(ArducamUtils.FIRMWARE_VERSION_REG)
    _, sensor_id = arducam_utils.read_dev(ArducamUtils.FIRMWARE_SENSOR_ID_REG)
    _, serial_number = arducam_utils.read_dev(ArducamUtils.SERIAL_NUMBER_REG)
    print("Firmware Version: {}".format(firmware_version))
    print("Sensor ID: 0x{:04X}".format(sensor_id))
    print("Serial Number: 0x{:08X}".format(serial_number))
    
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280*4)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)


    s.connect((HOST, PORT))
    print("Connected")

    mode = 0
    detail = 0
    capture = True
    record = False

    def readCommandThread() :
        global mode
        global detail
        run = True
        while run:
            try :
                data = s.recv(2)
                command = data[0]
                value = data[1]
                print('Received', repr(data))
                if command == 0 : # stream all
                    mode = 0
                    detail = 0
                elif command == 1 : # stream one
                    mode = 1
                    detail = value
                elif command == 2  : # exposure
                    #print("v4l2-ctl -c exposure={}".format(int(value/255.0 * 10000)))
                    os.system("v4l2-ctl -c exposure={}".format(int(value/255.0 * 1500) ))
                elif command == 3 :
                    record = True

            except :
                run = False


    def streamingThread() :
        next_run = threading.Timer(0.1, streamingThread)
        next_run.start()
        if len(fifo) > 0 :
            image_array = fifo[-1]
            images_indices = []
            image_arrays = []

            image_array = arducam_utils.convert(image_array)

            if mode == 0 :
                for idx in range(4) :
                    offset = idx * 1280
                    test = image_array[::4,offset:offset+1280:4]
                    images_indices.append(idx)
                    image_arrays.append(test)

            elif mode == 1 :
                offset = detail * 1280
                image_arrays.append(image_array[:,offset:offset+1280])
                images_indices.append(detail)

            for i in range(len(images_indices)) :
                idx = images_indices[i]
                image = image_arrays[i]
                _, encoded_image = cv2.imencode(".jpeg",image)
                with io.BytesIO() as image_buffer:
                    image_buffer.write((48).to_bytes(1,"little"))
                    image_buffer.write(idx.to_bytes(1,"little"))
                    image_buffer.write(len(encoded_image).to_bytes(4,"little"))
                    image_buffer.write(encoded_image)
                    try :
                        s.send(image_buffer.getvalue())
                    except:
                        next_run.cancel()

    command_thread = threading.Thread(target=readCommandThread).start()
    threading.Timer(0.1,streamingThread).start()

    frames = 1
    start = time.time()
    num_frames_to_record = 10
    record_frame = 100
    while True:
        ret, raw_frame = cap.read()
        raw_frame = raw_frame.reshape(800,1280*4)
        
        if record == True :
            record_frame = frames + num_frames_to_record
            record = False
        
        if frames == record_frame :
            print("Starting to save")
            idx = 0
            for frame in fifo :
                cv2.imwrite("img_{:03d}.png".format(idx), frame)
                img = cv2.imread("img_{:03d}.png".format(idx))
                print((img[:,:,0] - frame).max())
                idx += 1

            print("Done saving")


        if capture :
            if len(fifo) >= num_frames_to_record :
                old = fifo.popleft()
                del old
            fifo.append(raw_frame)

        if not (frames % 100) :
            now = time.time()
            print(100 / (now - start))
            start = now

        frames += 1


        #rgb_frame = arducam_utils.convert(raw_frame)
        #rgb_frame = rgb_frame[::4,:1280:4]
        #cv2.imshow("debug",rgb_frame)
        #cv2.waitKey(1)

#    image_idx = 0
#    start = time.time()
#    def cameraStream() :
#        scheduler.enter(0.03333,1,cameraStream,())
#
#        global image_idx
#        if image_idx >= len(images) :
#            image_idx = 0
#        image = images[image_idx]
#        image_idx += 1
#
#        if len(fifo) >= 150:
#            old = fifo.popleft()
#            del old
#        fifo.append(image)
#
#    scheduler.enter(0,1,cameraStream,())
#    schedule.run(True)
