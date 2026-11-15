"""Small shared helpers for timestamped observation recording."""
import cv2
from sensor_msgs.msg import CompressedImage

def encode_preview(bgr, stamp, frame_id):
    ok, encoded=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,85])
    if not ok:
        raise RuntimeError('Could not encode camera preview')
    msg=CompressedImage()
    msg.header.stamp=stamp
    msg.header.frame_id=frame_id
    msg.format='jpeg'
    msg.data=encoded.tobytes()
    return msg

def marker_label(command):
    return {'s':'start','m':'waypoint','e':'end'}.get(command.strip().lower())
