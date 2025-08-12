#!/usr/bin/env python3

import zmq
import numpy as np
import cv2

class ZMQFrameReceiver:
    def __init__(self, connect_addr="tcp://localhost:5555", topic_filter="frame"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(connect_addr)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)

    def receive_frame(self):
        try:
            if self.socket.poll(timeout=100):  # wait max 100 ms for a message
                msg = self.socket.recv(flags=zmq.NOBLOCK)
                
                # msg starts with topic prefix "frame " (6 bytes)
                prefix = b"frame "
                if not msg.startswith(prefix):
                    print("Warning: message does not start with topic prefix")
                    return None

                # Strip prefix
                raw_img = msg[len(prefix):]

                np_img = np.frombuffer(raw_img, dtype=np.uint8)
                frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
                if frame is None:
                    print("Warning: failed to decode frame")
                return frame
            else:
                return None  # no new frame available
        except zmq.Again:
            return None
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print("ZMQ receive error:", e)
            return None

if __name__ == "__main__":
    recv = ZMQFrameReceiver()
    while True:
        frame = recv.receive_frame()
        if frame is None:
            continue
        print("Received frame shape:", frame.shape)
