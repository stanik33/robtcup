from threading import Thread, Lock
import cv2
import time


class CameraReader:
    def __init__(self, src = 0, width = 320, height = 240):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.thread: Thread = None
        self.read_lock = Lock()
        self.cnt_time = 0
        self.cnt_value = 0

    def start(self):
        if self.started:
            print("Camera reader already started!")
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.cnt_time = time.time()
        self.cnt_value = 0
        self.thread.start()
        return self

    def update(self):
        while self.started:
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed = grabbed
            self.frame = frame
            self.process()
            self.read_lock.release()

    def process(self):
        self.cnt_value += 1
        if time.time() - self.cnt_time > 1:
            #print(self.cnt_value, ' fps')
            self.cnt_value = 0
            self.cnt_time = time.time()

    def read(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stream.release()
