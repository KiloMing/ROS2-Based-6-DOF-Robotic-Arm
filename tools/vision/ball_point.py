"""Local CPU ball detection with registered Orbbec depth; estimates visible surface Z."""
import argparse
import sys
# Reuse system ROS dependencies while retaining the camera virtual environment.
sys.path.append("/usr/lib/python3/dist-packages")
import json
import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from capture_helpers import encode_preview
import time
import cv2
import numpy as np
import pyorbbecsdk as ob
from ultralytics import YOLO
from camera_utils import frame_to_bgr_image


def surface_depth(depth, box):
    h, w = depth.shape
    x1, y1, x2, y2 = box
    u, v = (x1+x2)/2, (y1+y2)/2
    radius = max(2, int(min(x2-x1, y2-y1)*0.15))
    x, y = int(u), int(v)
    roi = depth[max(0,y-radius):min(h,y+radius+1), max(0,x-radius):min(w,x+radius+1)]
    good = roi[(roi > 100) & (roi < 10000)]
    if good.size < 5 or good.size < roi.size * 0.5:
        return None
    z = float(np.median(good))
    if np.percentile(good,90)-np.percentile(good,10) > max(80,z*0.1):
        return None
    return z / 1000


def project_surface(u, v, z, intrinsic, distortion):
    """Return optical-frame metres; SDK RGB distortion is Brown rational."""
    k = np.array([[intrinsic.fx, 0, intrinsic.cx],
                  [0, intrinsic.fy, intrinsic.cy], [0, 0, 1]], dtype=float)
    d = np.array([distortion.k1, distortion.k2, distortion.p1, distortion.p2,
                  distortion.k3, distortion.k4, distortion.k5, distortion.k6])
    if not np.isfinite(k).all() or intrinsic.fx <= 0 or intrinsic.fy <= 0:
        raise ValueError("Invalid RGB intrinsics")
    ray = cv2.undistortPoints(np.array([[[u, v]]], dtype=float), k, d)[0, 0]
    return float(ray[0]*z), float(ray[1]*z), float(z)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--headless',action='store_true')
    ap.add_argument('--frames',type=int,default=0)
    ap.add_argument('--conf',type=float,default=0.25)
    ap.add_argument('--model', default='/home/kiloming/Documents/Codex/2026-11-13/qign/outputs/tennis_test/yolo11n.pt')
    ap.add_argument('--frame-id', default='camera_color_optical_frame')
    args=ap.parse_args()
    rclpy.init(args=[])
    node=rclpy.create_node('ball_surface_detector')
    point_pub=node.create_publisher(PointStamped, '/vision/ball_surface_point', 1)
    status_pub=node.create_publisher(String, '/vision/ball_status', 1)
    image_pub=node.create_publisher(CompressedImage, '/vision/preview/compressed', 1)
    started=False
    pipeline=None
    count=0
    last=time.monotonic()
    try:
        model=YOLO(args.model)
        pipeline=ob.Pipeline()
        config=ob.Config()
        for sensor in (ob.OBSensorType.COLOR_SENSOR,ob.OBSensorType.DEPTH_SENSOR):
            config.enable_stream(pipeline.get_stream_profile_list(sensor).get_default_video_stream_profile())
        config.set_align_mode(ob.OBAlignMode.SW_MODE)
        pipeline.start(config); started=True
        param=pipeline.get_camera_param()
        intrinsic=param.rgb_intrinsic
        print(f'RGB intrinsics: {intrinsic}; optical frame: {args.frame_id}', flush=True)
        print('Camera ready. Keep ball still. Q or Esc exits.',flush=True)
        while not args.frames or count < args.frames:
            frames=pipeline.wait_for_frames(500)
            if frames is None or frames.get_color_frame() is None or frames.get_depth_frame() is None:
                if time.monotonic()-last>10: raise RuntimeError('No RGB/depth frames for 10 seconds')
                continue
            last=time.monotonic()
            # Host receive time BEFORE inference, not a claimed hardware capture time.
            stamp=node.get_clock().now().to_msg()
            color=frame_to_bgr_image(frames.get_color_frame())
            df=frames.get_depth_frame()
            depth=np.frombuffer(df.get_data(),np.uint16).reshape(df.get_height(),df.get_width()).astype(np.float32)*df.get_depth_scale()
            if color is None: raise RuntimeError('Unsupported color format')
            if color.shape[:2]!=depth.shape: raise RuntimeError('RGB/depth alignment failed')
            if (intrinsic.width, intrinsic.height) != (color.shape[1], color.shape[0]):
                raise RuntimeError('RGB intrinsics resolution mismatch')
            result=model.predict(color,device='cpu',classes=[32],conf=args.conf,imgsz=640,verbose=False)[0]
            labels=[]
            candidates=[]
            for box in result.boxes:
                xy=box.xyxy[0].cpu().numpy()
                z=surface_depth(depth,xy)
                text=f'ball {float(box.conf[0]):.2f} | '+(f'Z surface ~{z:.3f} m' if z else 'NO RELIABLE DEPTH')
                labels.append(text)
                if z is not None:
                    u,v=(xy[0]+xy[2])/2,(xy[1]+xy[3])/2
                    xyz=project_surface(u,v,z,intrinsic,param.rgb_distortion)
                    if np.isfinite(xyz).all():
                        candidates.append((xyz,float(box.conf[0])))
                        text += f" XYZ=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})m"
                        labels[-1]=text
                x1,y1,x2,y2=xy.astype(int)
                cv2.rectangle(color,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(color,text,(max(0,x1),max(22,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,0.48,(0,255,0),1)
            cv2.putText(color,'Sports-ball candidates | Q: quit | Depth is surface Z',(8,22),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,255),1)
            status={'valid': False, 'frame_id': args.frame_id,
                    'stamp_sec': stamp.sec, 'stamp_nanosec': stamp.nanosec,
                    'timestamp_source': 'host_receive_before_inference',
                    'meaning': 'visible_surface_not_ball_center',
                    'detections': len(result.boxes), 'valid_depth_candidates': len(candidates)}
            if len(result.boxes)==1 and len(candidates)==1:
                xyz, confidence=candidates[0]
                msg=PointStamped()
                msg.header.stamp=stamp
                msg.header.frame_id=args.frame_id
                msg.point.x,msg.point.y,msg.point.z=xyz
                point_pub.publish(msg)
                status.update(valid=True, xyz_m=xyz, confidence=confidence)
            else:
                status['reason']='no_unique_detection_with_reliable_depth'
            status_pub.publish(String(data=json.dumps(status)))
            if image_pub.get_subscription_count():
                image_pub.publish(encode_preview(color,stamp,args.frame_id))
            rclpy.spin_once(node,timeout_sec=0)
            count+=1
            if args.headless:
                print(f'Frame {count}: depth valid={np.mean(depth>0):.1%}; '+('; '.join(labels) or 'no ball detected'),flush=True)
            else:
                view=cv2.applyColorMap(np.clip(depth/4000*255,0,255).astype(np.uint8),cv2.COLORMAP_JET)
                view[depth==0]=0
                cv2.imshow('YOLO ball / aligned depth',np.hstack((color,view)))
                if cv2.waitKey(1)&255 in (27,ord('q')): break
    finally:
        if started: pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__':
    try: main()
    except KeyboardInterrupt: pass
