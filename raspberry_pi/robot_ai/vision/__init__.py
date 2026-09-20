"""Desktop-only GPU vision tools.

The Raspberry Pi runtime deliberately keeps using OpenCV DNN + ONNX.  The
modules in this package are for training and validating a detector on the
Windows GPU before an exported ONNX model is copied to the Pi.
"""

from .unified_yolo import UnifiedYolo, canonical_name

__all__ = ["UnifiedYolo", "canonical_name"]
