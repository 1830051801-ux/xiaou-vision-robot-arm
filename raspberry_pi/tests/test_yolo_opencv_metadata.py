from __future__ import annotations

import unittest

from robot_ai.yolo_opencv import _parse_onnx_names_metadata


class YoloOnnxMetadataTests(unittest.TestCase):
    def test_ultralytics_dictionary_names_are_parsed_in_id_order(self) -> None:
        self.assertEqual(
            _parse_onnx_names_metadata("{0: 'pen', 1: 'bottle', 2: 'cola', 3: 'earphone'}"),
            ["pen", "bottle", "cola", "earphone"],
        )

    def test_json_list_names_are_accepted(self) -> None:
        self.assertEqual(_parse_onnx_names_metadata('["pen", "bottle"]'), ["pen", "bottle"])

    def test_invalid_or_non_contiguous_metadata_is_rejected(self) -> None:
        self.assertIsNone(_parse_onnx_names_metadata("{1: 'bottle'}"))
        self.assertIsNone(_parse_onnx_names_metadata("__import__('os').system('bad')"))
        self.assertIsNone(_parse_onnx_names_metadata("['pen', 'pen']"))


if __name__ == "__main__":
    unittest.main()
