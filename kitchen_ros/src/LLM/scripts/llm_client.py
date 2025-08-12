#!/usr/bin/env python3

import os
import json
import torch
from transformers import pipeline


OPENAI_KEY = os.environ.get("OPENAI_API_KEY")

# Option 1: OpenAI
if OPENAI_KEY:
    import openai
    openai.api_key = OPENAI_KEY

class LLMClient:
    def __init__(self, hf_model_name="facebook/opt-125m", device=None):
        # device handling: device = 0 for cuda, -1 for cpu
        self.device = 0 if (device is None and torch.cuda.is_available()) else -1
        self.hf_model_name = hf_model_name
        self._hf_pipe = None

    def _init_hf(self):
        if self._hf_pipe is None:
            self._hf_pipe = pipeline("text-generation", model=self.hf_model_name, device=self.device, torch_dtype=torch.float16 if self.device==0 else None)
    
    def query(self, prompt, max_tokens=256, temperature=0.2):
        """
        Returns a string reply. Uses OpenAI if API key present, otherwise HF pipeline.
        """
        if OPENAI_KEY:
            return self._query_openai(prompt, max_tokens, temperature)
        else:
            return self._query_hf(prompt, max_tokens, temperature)

    def _query_openai(self, prompt, max_tokens, temperature):
        try:
            resp = openai.ChatCompletion.create(
                model="gpt-4o-mini" if False else "gpt-4o" if False else "gpt-3.5-turbo", # safe default to gpt-3.5-turbo; change as you like
                messages=[{"role":"user","content":prompt}],
                max_tokens=max_tokens,
                temperature=temperature
            )
            return resp["choices"][0]["message"]["content"].strip()
        except Exception as e:
            return f"[OpenAI error] {e}"

    def _query_hf(self, prompt, max_tokens, temperature):
        try:
            self._init_hf()
            out = self._hf_pipe(prompt, max_new_tokens=max_tokens, do_sample=True, temperature=temperature)
            return out[0]["generated_text"]
        except Exception as e:
            return f"[HF local model error] {e}"

if __name__ == "__main__":
    c = LLMClient()
    print(c.query("Write a two-sentence poem about coffee."))
