#!/usr/bin/env python3

import os
import time
import gradio as gr
import cv2

from zmq_receiver import ZMQFrameReceiver
from vision_caption import VisionCaptioner
from llm_client import LLMClient

# Initialize once
zmq_addr = os.environ.get("ZMQ_ADDR", "tcp://127.0.0.1:5555")
receiver = ZMQFrameReceiver(connect_addr=zmq_addr)
captioner = VisionCaptioner()
llm = LLMClient()

FIXED_QUESTION = "What do you see?"

def build_prompt(caption: str) -> str:
    return (
        "You are a helpful assistant that answers questions about images.\n\n"
        f"Image description: {caption}\n\n"
        f"Question: {FIXED_QUESTION}\n\n"
        "Answer concisely and mention if the image description seems uncertain.\n"
    )

def describe_image():
    frame = None
    start = time.time()
    while frame is None and time.time() - start < 5.0:
        frame = receiver.receive_frame()
        if frame is None:
            time.sleep(0.01)
    if frame is None:
        return None, "No frame received from stream."

    caption = captioner.image_to_caption(frame)

    # (Optional) You can still query LLM internally if needed, 
    # but since you want to remove answer display, skip outputting it.

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame_rgb, caption

with gr.Blocks() as demo:
    gr.Markdown("# 📷 What Do You See?\nClick the button to capture the frame and get a description.")

    with gr.Row():
        describe_button = gr.Button("What do you see?")
    with gr.Row():
        frame_output = gr.Image(label="Captured Frame")
        caption_output = gr.Textbox(label="Image Caption")

    describe_button.click(fn=describe_image, inputs=None, outputs=[frame_output, caption_output])

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7860)