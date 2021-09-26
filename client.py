import socket
import io
import threading
import collections
import cv2
from utils import ArducamUtils
import os
from enum import Enum
import time
import sys
import signal

FRAME_BUFFER_SIZE = 10
INDEX_OFFSET = 0
DEBUG = True
DEBUG_STREAM = False

class ConnectionState(Enum):
    NOT_CONNECTED = 0
    READY_TO_CONNECT = 1
    CONNECTED = 2

class StreamingMode(Enum):
    ALL_FRAMES_LOW_RES = 0
    ONE_FRAME_HIGH_RES = 1

g_streaming_mode = StreamingMode.ALL_FRAMES_LOW_RES
g_streaming_detail_img = -1

g_state = ConnectionState.NOT_CONNECTED
g_server_ip = "172.20.10.9"
g_server_port = 1000
g_frame_fifo = collections.deque(maxlen=FRAME_BUFFER_SIZE)
g_record_video = False
g_arducam_utils = None

g_shutdown = False

g_logfile = open("/home/rob/pleno_log/pleno.log","w",1)

def log(message) :
    g_logfile.write(message+"\n")
    g_logfile.flush()
    if DEBUG : print(message)


def readCommandThread(s) :
    global g_streaming_mode
    global g_streaming_detail_img
    global g_record_video

    class Command(Enum):
        STREAM_ALL_FRAMES_LOW_RES = 0
        STREAM_ONE_FRAME_HIGH_RES = 1
        SET_EXPOSURE = 2
        RECORD_FRAMES_TO_DISK = 3

    run = True
    while run and not g_shutdown:
        try :
            data = s.recv(2)
            command = data[0]
            value = data[1]
            if command ==  Command.STREAM_ALL_FRAMES_LOW_RES :
                g_streaming_mode = 0
                g_streaming_detail_img = -1
            elif command == Command.STREAM_ONE_FRAME_HIGH_RES :
                g_streaming_mode = 1
                g_streaming_detail_img = value
            elif command == Command.SET_EXPOSURE :
                os.system("v4l2-ctl -c exposure={}".format(int(value/255.0 * 1500) ))
            elif command == Command.RECORD_FRAMES_TO_DISK :
                g_record_video = True
        except socket.timeout :
            None
        except :
            log("Unexpected error:{}".format(sys.exc_info()))
            run = False
    
    log("Ending readCommandThread")

def streamingThread(s) :
    if g_shutdown : 
        log("Ending streamingThread")
        return
    next_run = threading.Timer(0.1, streamingThread, args=[s])
    next_run.start()
    if len(g_frame_fifo) > 0 :
        raw_frame = g_frame_fifo[-1]
        images_indices = []
        images_to_stream = [] # Could not put these numpy arrays in dicts

        image_array = g_arducam_utils.convert(raw_frame)

        if g_streaming_mode == StreamingMode.ALL_FRAMES_LOW_RES :
            for idx in range(4) :
                offset = idx * 1280
                images_to_stream.append(image_array[::4,offset:offset+1280:4])
                images_indices.append(idx + INDEX_OFFSET)

        elif g_streaming_mode == StreamingMode.ONE_FRAME_HIGH_RES :
            offset = g_streaming_detail_img % 4 * 1280
            images_to_stream.append(image_array[:,offset:offset+1280])
            images_indices.append(g_streaming_detail_img)

        for i in range(len(images_indices)) :
            idx = images_indices[i]
            image = images_to_stream[i]
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
                    log("Ending streamingThread")
                    break

def wakeupThread() :
    global g_state
    # UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # UDPServerSocket.bind(("172.20.10.5", 8081))
    # bufferSize  = 1024
    # log("Listening for UDP broadcast on port 8081")

    # while(True):
    #     (msg,addr) = UDPServerSocket.recvfrom(bufferSize)
    #     print(msg)
    #     if msg == "connect" and (g_state == ConnectionState.NOT_CONNECTED):
    #         g_server_ip = addr
    #         g_state = ConnectionState.READY_TO_CONNECT
    #         log("Server up notification - IP is ",g_server_ip)
    while g_state == ConnectionState.NOT_CONNECTED and not g_shutdown :
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try : 
                s.settimeout(5)
                s.connect((g_server_ip, g_server_port))
                g_state = ConnectionState.CONNECTED
                log("Connected")

                if g_state == ConnectionState.CONNECTED :
                    command_thread = threading.Thread(target=readCommandThread, args=[s])
                    command_thread.start()
                    threading.Timer(0.1,streamingThread, args=[s]).start()
                    command_thread.join()
                    try:
                        s.shutdown()
                        s.close()
                    except:
                        print("Error shutting down socket")
                    g_state = ConnectionState.NOT_CONNECTED
                    log("Connection closed")
                    time.sleep(10)

            except : 
                g_state = ConnectionState.NOT_CONNECTED
                log("Connection failed")
                time.sleep(5)

    log("Ending wakeupThread")



def fourcc(a, b, c, d):
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

def videoCaptureThread() :
    global g_record_video
    global g_frame_fifo
    global g_arducam_utils

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc('R','G','G','B'))

    g_arducam_utils = ArducamUtils(0)

    _, firmware_version = g_arducam_utils.read_dev(ArducamUtils.FIRMWARE_VERSION_REG)
    _, sensor_id = g_arducam_utils.read_dev(ArducamUtils.FIRMWARE_SENSOR_ID_REG)
    _, serial_number = g_arducam_utils.read_dev(ArducamUtils.SERIAL_NUMBER_REG)
    log("Firmware Version: {}".format(firmware_version))
    log("Sensor ID: 0x{:04X}".format(sensor_id))
    log("Serial Number: 0x{:08X}".format(serial_number))
    
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280*4)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

    frame_idx = 1
    start = time.time()
    record_frame_idx = -1
    while not g_shutdown:
        ret, raw_frame = cap.read()
        raw_frame = raw_frame.reshape(800,1280*4)
        
        if g_record_video == True :
            record_frame_idx = frame_idx + FRAME_BUFFER_SIZE / 2  # Capture before and after
            g_record_video = False
        
        if frame_idx == record_frame_idx :
            log("saving queue at frame {}".format(frame_idx))
            idx = 0
            for frame in g_frame_fifo :
                cv2.imwrite("img_{:03d}.png".format(idx), frame)
                idx += 1
            log("Done saving")


        if len(g_frame_fifo) >= FRAME_BUFFER_SIZE :
            old = g_frame_fifo.popleft()
            del old
        g_frame_fifo.append(raw_frame)

        if DEBUG :
            if not (frame_idx % 100) :
                now = time.time()
                print("FPS : ",100 / (now - start))
                start = now

        if DEBUG_STREAM :
            image = g_arducam_utils.convert(raw_frame)
            image = cv2.resize(image,(1280,200))
            cv2.imshow("Debug",image)
            cv2.waitKey(1)

        frame_idx += 1

    log("Ending videoCaptureThread")



def signal_handler(sig, frame):
    global g_shutdown
    g_shutdown = True
    print("Shutting down")

def start() :

    global INDEX_OFFSET

    signal.signal(signal.SIGINT, signal_handler)

    if len(sys.argv) > 1 :
        INDEX_OFFSET = 4 * int(sys.argv[1])

    log("Starting plenopticon client with index offset {}".format(INDEX_OFFSET))
    wakeup_thread = threading.Thread(target=wakeupThread)
    video_capture_thread = threading.Thread(target=videoCaptureThread)

    wakeup_thread.start()
    video_capture_thread.start()

    wakeup_thread.join()
    video_capture_thread.join()

    log("Terminated unexpectedly")


if __name__ == "__main__" :

    start()
