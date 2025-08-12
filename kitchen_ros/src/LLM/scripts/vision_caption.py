#!/usr/bin/env python3

import torch
from PIL import Image
import numpy as np
import cv2

from transformers import BlipProcessor, BlipForConditionalGeneration

class VisionCaptioner:
    def __init__(self, model_name="Salesforce/blip-image-captioning-base", device=None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = BlipProcessor.from_pretrained(model_name)
        self.model = BlipForConditionalGeneration.from_pretrained(model_name).to(self.device)
        self.model.eval()

    def _bgr_to_pil(self, frame_bgr):

        img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(img_rgb)

    def image_to_caption(self, frame_bgr, max_new_tokens=20):
        """
        frame_bgr: OpenCV BGR numpy array
        Returns: short caption string
        """
        image = self._bgr_to_pil(frame_bgr)
        inputs = self.processor(images=image, return_tensors="pt").to(self.device)
        with torch.no_grad():
            output_ids = self.model.generate(**inputs, max_new_tokens=max_new_tokens, num_beams=3)
        caption = self.processor.decode(output_ids[0], skip_special_tokens=True)
        return caption
