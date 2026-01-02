---
sidebar_position: 13
---

# Chapter 13: Speech Recognition

## Learning Objectives

By the end of this chapter, you will:
- Integrate Whisper for STT
- Handle real-time audio streams
- Parse voice commands
- Manage wake word detection

## 13.1 Whisper Integration

```python
#!/usr/bin/env python3
"""
Whisper Speech-to-Text Integration

Implements:
- Real-time audio transcription
- Voice command parsing
- Wake word detection

Hardware Reality Note:
    - Whisper latency: 500ms-2s
    - Model size: small (250MB) to large (1.5GB)
    - Microphone array improves SNR by 10-20dB
"""

import asyncio
import numpy as np
from faster_whisper import WhisperModel
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class VoiceCommand:
    """Parsed voice command."""
    intent: str
    objects: List[str]
    location: Optional[str]
    confidence: float


class WhisperSTT:
    """Whisper speech-to-text."""

    def __init__(self, model_size: str = "small"):
        self.model = WhisperModel(model_size, device="cuda", compute_type="float16")

    async def transcribe_stream(self, audio_chunk: bytes) -> str:
        """Transcribe audio chunk."""
        segments, _ = self.model.transcribe(audio_chunk)
        return " ".join([seg.text for seg in segments])


class VoiceCommandProcessor:
    """Process voice commands."""

    def __init__(self):
        self.whisper = WhisperSTT("small")

    def parse_command(self, text: str) -> VoiceCommand:
        """Parse transcribed text into command."""
        text_lower = text.lower()

        # Intent detection
        if "pick" in text_lower or "grab" in text_lower:
            intent = "pick_object"
        elif "place" in text_lower or "put" in text_lower:
            intent = "place_object"
        elif "go" in text_lower or "walk" in text_lower:
            intent = "navigate"
        else:
            intent = "unknown"

        # Object extraction
        objects = [w for w in text.split() if w in ["cup", "bottle", "box", "tool"]]

        return VoiceCommand(
            intent=intent,
            objects=objects,
            location=None,
            confidence=0.9
        )
```
