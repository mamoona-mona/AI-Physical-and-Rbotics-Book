---
sidebar_position: 14
---

# Chapter 14: VLA Model Integration

## Learning Objectives

By the end of this chapter, you will:
- Implement vision-language-action models
- Fuse visual and language inputs
- Generate continuous actions
- Validate safety constraints

## 14.1 VLA Architecture

```python
#!/usr/bin/env python3
"""
Vision-Language-Action (VLA) Model Integration

Implements:
- Vision encoder (ViT)
- Language encoder (LLaMA)
- Cross-modal fusion
- Action decoder

Hardware Reality Note:
    - Model size: 1-10B parameters
    - Inference latency: 50-500ms
    - GPU memory: 8-24GB VRAM
"""

import torch
import torch.nn as nn
from typing import Dict, List
import numpy as np


class VisionEncoder(nn.Module):
    """Vision transformer encoder."""

    def __init__(self, embed_dim: int = 768):
        super().__init__()
        self.patch_embed = nn.Conv2d(3, embed_dim, kernel_size=16, stride=16)
        self.pos_embed = nn.Parameter(torch.randn(1, 196, embed_dim) * 0.02)
        self.transformer = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model=embed_dim, nhead=12),
            num_layers=12
        )

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        x = self.patch_embed(images)
        x = x.flatten(2).transpose(1, 2)
        x = x + self.pos_embed[:, :x.size(1)]
        return self.transformer(x)


class CrossModalDecoder(nn.Module):
    """Cross-modal decoder for action generation."""

    def __init__(self, vision_dim: int = 768, language_dim: int = 4096, action_dim: int = 56):
        super().__init__()
        self.vision_proj = nn.Linear(vision_dim, 2048)
        self.language_proj = nn.Linear(language_dim, 2048)
        self.cross_attn = nn.MultiheadAttention(2048, 8)
        self.action_head = nn.Sequential(
            nn.Linear(2048, 2048),
            nn.ReLU(),
            nn.Linear(2048, action_dim)
        )

    def forward(self, vision_features, language_features) -> torch.Tensor:
        v = self.vision_proj(vision_features)
        l = self.language_proj(language_features)
        fused, _ = self.cross_attn(v.unsqueeze(0), l.unsqueeze(0), l.unsqueeze(0))
        return self.action_head(fused)


class VLAModel(nn.Module):
    """Complete VLA model."""

    def __init__(self):
        super().__init__()
        self.vision_encoder = VisionEncoder()
        self.decoder = CrossModalDecoder()
        self.safety_checker = SafetyValidator()

    def forward(self, image: torch.Tensor, instruction: str) -> torch.Tensor:
        vision_features = self.vision_encoder(image)
        language_features = self._encode_language(instruction)
        actions = self.decoder(vision_features, language_features)

        # Safety validation
        if not self.safety_checker.validate(actions):
            raise ValueError("Unsafe action generated")

        return actions

    def _encode_language(self, text: str) -> torch.Tensor:
        """Encode text (simplified)."""
        return torch.randn(1, 4096)


class SafetyValidator:
    """Validate VLA actions for safety."""

    def __init__(self):
        self.joint_limits = {
            'left_hip_pitch': (-2.0, 0.5),
            'right_hip_pitch': (-2.0, 0.5),
            # ... more joints
        }

    def validate(self, actions: torch.Tensor) -> bool:
        """Check actions are within safe limits."""
        for i, (name, (low, high)) in enumerate(self.joint_limits.items()):
            if i < len(actions):
                if actions[i] < low or actions[i] > high:
                    return False
        return True
```

## Chapter Summary

| Component | Function |
|-----------|----------|
| Vision Encoder | Image → features |
| Language Encoder | Text → features |
| Cross Attention | Fuse modalities |
| Safety Validator | Constraint checking |

---

*Next: Chapter 15 - Capstone*
