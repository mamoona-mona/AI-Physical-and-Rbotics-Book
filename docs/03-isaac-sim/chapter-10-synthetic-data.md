---
sidebar_position: 10
---

# Chapter 10: Synthetic Data Generation

## Learning Objectives

By the end of this chapter, you will:
- Use Replicator for synthetic data
- Implement domain randomization
- Generate annotated datasets
- Train perception models

## 10.1 Replicator Pipeline

```python
#!/usr/bin/env python3
"""
Synthetic Data Generation with Replicator

Implements:
- Randomizer graphs for domain randomization
- Annotator setup for ground truth
- Dataset export in standard formats

Hardware Reality Note:
    - Generate 100K+ images for training
    - Annotation includes: depth, semantics, 3D bboxes
    - Randomization covers: lighting, texture, pose
"""

import omni.replicator.core as rep
from omni.replicator.isaac import utils


class SyntheticDataGenerator:
    """Generate synthetic training data."""

    def __init__(self):
        self.camera = None
        self.annotators = {}

    def setup_camera(self, resolution=(640, 480)):
        """Setup synthetic camera."""
        self.camera = rep.create.camera(
            position=(0.5, 0.5, 1.0),
            rotation=(0, 0, 0),
            focal_length=24.0,
            focus_distance=1.0,
            resolution=resolution
        )

    def setup_annotators(self):
        """Setup annotation annotators."""
        # RGB
        self.annotators['rgb'] = rep.AnnotatorRegistry.get_annotator(
            'rgb', device='cuda'
        )

        # Depth
        self.annotators['depth'] = rep.AnnotatorRegistry.get_annotator(
            'distance_to_camera', device='cuda'
        )

        # Semantic segmentation
        self.annotators['semantic'] = rep.AnnotatorRegistry.get_annotator(
            'semantic_segmentation', device='cuda'
        )

        # Bounding boxes
        self.annotators['bbox'] = rep.AnnotatorRegistry.get_annotator(
            'bounding_box_2d_tight', device='cuda'
        )

    def setup_randomizer(self):
        """Setup domain randomizer."""
        with rep.trigger.on_frame():
            # Randomize lighting
            rep.modify.light(
                light_type="dome",
                intensity=rep.distribution.uniform(500, 1500),
                color=rep.distribution.uniform(0.8, 1.0)
            )

            # Randomize materials
            rep.randomizer.materials(
                materials=rep.assets.get_materials_from_path("/World/Looks/*"),
                distribution=rep.distribution.uniform(0.0, 1.0)
            )

    def generate_grasp_dataset(self, num_samples: int = 10000):
        """Generate grasp dataset."""
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir="/tmp/synthetic_data", rgb=True)
        writer.attach(self.annotators.values())

        for i in range(num_samples):
            rep.orchestrator.step()

        print(f"Generated {num_samples} samples")
```

## Chapter Summary

| Technique | Use Case |
|-----------|----------|
| Domain randomization | Robust perception |
| Replicator | Synthetic data |
| Annotators | Ground truth |

---

*Next: Chapter 11 - VSLAM*
