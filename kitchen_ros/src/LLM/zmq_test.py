import zmq
import cv2
import numpy as np





class ZMQFrameReceiver:
    def __init__(self, connect_addr="tcp://localhost:5555", topic_filter="frame"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(connect_addr)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)

    def receive_frame(self):
        try:
            # Poll socket for 100 ms
            if self.socket.poll(timeout=100):
                last_frame = None
                # Drain all waiting messages, keep only the last one
                while True:
                    try:
                        msg = self.socket.recv(flags=zmq.NOBLOCK)
                        # Strip topic prefix (e.g., "frame ")
                        if isinstance(msg, bytes):
                            msg_str = msg.decode(errors='ignore')
                            if msg_str.startswith("frame "):
                                raw_img = msg[len("frame "):]
                            else:
                                raw_img = msg
                        else:
                            raw_img = msg

                        np_img = np.frombuffer(raw_img, dtype=np.uint8)
                        last_frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
                    except zmq.Again:
                        # No more messages available
                        break

                if last_frame is None:
                    print("Warning: no valid frame decoded")
                return last_frame
            else:
                # No message available
                return None
        except Exception as e:
            print(f"ZMQ receive error: {e}")
            return None

if __name__ == "__main__":
    receiver = ZMQFrameReceiver(connect_addr="tcp://localhost:5555", topic_filter="frame")
    import time
    while True:
        frame = receiver.receive_frame()
        if frame is not None:
            print("Received frame of shape:", frame.shape)
        else:
            print("No frame received")
        time.sleep(0.1)