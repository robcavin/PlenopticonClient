import socket
import glob
from PIL import Image
import numpy as np
import io
import threading
import collections

image_paths = glob.glob('*png')
images = []

print(image_paths)
for path in image_paths:
    with Image.open(path) as img:
        image = img.resize((1280*4,800))
        images.append(np.array(image))

print("Done")
print(len(images))

HOST = '172.20.10.1'  # The server's hostname or IP address
PORT = 1000        # The port used by the server

fifo = collections.deque(maxlen=150)

import sched
import time

scheduler = sched.scheduler(time.time, time.sleep)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    s.connect((HOST, PORT))
    print("Connected")

    mode = 0
    detail = 0

    def readCommandThread() :
        global mode
        global detail
        while True:
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
            except :
                break


    def streamingThread() :
        next_run = threading.Timer(0.1, streamingThread)
        next_run.start()
        if len(fifo) > 0 :
            image_array = fifo[-1]
            images_indices = []
            image_arrays = []

            if mode == 0 :
                for idx in range(4) :
                    offset = idx * 1280
                    test = image_array[::4,offset:offset+1280:4,::-1]
                    images_indices.append(idx)
                    image_arrays.append(test)

            elif mode == 1 :
                offset = detail * 1280
                image_arrays.append(image_array[:,offset:offset+1280,::-1])
                images_indices.append(detail)

            for i in range(len(images_indices)) :
                idx = images_indices[i]
                image = image_arrays[i]
                with Image.fromarray(image) as pil_image:
                    with io.BytesIO() as image_buffer:
                        image_buffer.write((48).to_bytes(1,"little"))
                        image_buffer.write(idx.to_bytes(1,"little"))
                        image_buffer.write((0).to_bytes(4,"little"))
                        pil_image.save(image_buffer, format="JPEG")
                        end = image_buffer.tell()
                        image_buffer.seek(2)
                        image_buffer.write((end - 6).to_bytes(4,"little"))
                        try :
                            s.send(image_buffer.getvalue())
                        except:
                            next_run.cancel()

    command_thread = threading.Thread(target=readCommandThread).start()
    threading.Timer(0.1,streamingThread).start()

    image_idx = 0
    start = time.time()
    def cameraStream() :
        scheduler.enter(0.03333,1,cameraStream,())

        global image_idx
        if image_idx >= len(images) :
            image_idx = 0
        image = images[image_idx]
        image_idx += 1

        if len(fifo) >= 150:
            old = fifo.popleft()
            del old
        fifo.append(image)

    scheduler.enter(0,1,cameraStream,())
    scheduler.run(True)
