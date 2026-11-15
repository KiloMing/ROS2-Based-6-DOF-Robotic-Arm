import unittest
import numpy as np
from capture_helpers import encode_preview, marker_label
from builtin_interfaces.msg import Time
import cv2

class CaptureTests(unittest.TestCase):
    def test_preview_preserves_stamp_frame_and_pixels(self):
        stamp=Time(sec=123,nanosec=456)
        message=encode_preview(np.full((12,16,3),128,np.uint8),stamp,'camera')
        self.assertEqual(message.header.stamp,stamp)
        self.assertEqual(message.header.frame_id,'camera')
        decoded=cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape,(12,16,3))
        self.assertLess(abs(float(decoded.mean())-128),2)
    def test_markers_reject_unknown_input(self):
        self.assertEqual(marker_label('S'),'start')
        self.assertEqual(marker_label('e'),'end')
        self.assertIsNone(marker_label('garbage'))

if __name__=='__main__':unittest.main()
