"""Stationary eye-in-hand sample collection. No robot command interfaces."""
import sys
sys.path.append('/usr/lib/python3/dist-packages')
import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time
import cv2
import numpy as np
import pyorbbecsdk as ob
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from camera_utils import frame_to_bgr_image


def matrix(tf):
    q=tf.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    norm=np.linalg.norm([x,y,z,w])
    if norm < 1e-8: raise ValueError('Invalid quaternion')
    x,y,z,w=np.array([x,y,z,w])/norm
    t=np.eye(4)
    t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    t[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z]
    return t


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--square-mm',type=float,default=18.)
    ap.add_argument('--output',default=str(Path(__file__).resolve().parent/'captures'/datetime.now().strftime('%Y%m%d_%H%M%S')))
    ap.add_argument('--frames',type=int,default=0)
    args=ap.parse_args()
    if args.square_mm<=0: ap.error('square-mm must be positive')
    out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    obj=np.zeros((48,3),np.float64)
    obj[:,:2]=np.mgrid[0:8,0:6].T.reshape(-1,2)*args.square_mm/1000
    rclpy.init(args=[])
    node=rclpy.create_node('handeye_sample_collector')
    buf=Buffer(); listener=TransformListener(buf,node,spin_thread=True)
    pipe=None; started=False; history=deque(); saved=[]; count=0
    for path in sorted(out.glob('*.json')):
        record=json.loads(path.read_text())
        if record['square_m'] != args.square_mm/1000 or record['parent_frame'] != 'wrist_pitch_link':
            raise ValueError('Existing session calibration settings differ')
        saved.append(np.array(record['T_base_mount']))
    notice=''; notice_until=0.
    try:
        pipe=ob.Pipeline(); config=ob.Config()
        config.enable_stream(pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile())
        pipe.start(config); started=True
        param=pipe.get_camera_param(); k=param.rgb_intrinsic; dist=param.rgb_distortion
        K=np.array([[k.fx,0,k.cx],[0,k.fy,k.cy],[0,0,1.]])
        D=np.array([dist.k1,dist.k2,dist.p1,dist.p2,dist.k3,dist.k4,dist.k5,dist.k6])
        print(f'Output: {out}\n18 mm default; S saves, Q exits. Fixed board, move camera between samples.',flush=True)
        last_frame=time.monotonic()
        previous_reason=None
        while rclpy.ok():
            frames=pipe.wait_for_frames(500)
            if frames is None or frames.get_color_frame() is None:
                if time.monotonic()-last_frame>10: raise RuntimeError('No color frame for 10 seconds')
                continue
            last_frame=time.monotonic(); stamp=node.get_clock().now()
            color=frame_to_bgr_image(frames.get_color_frame())
            if color is None: raise RuntimeError('Unsupported color format')
            if (k.width,k.height)!=(color.shape[1],color.shape[0]): raise RuntimeError('Intrinsic resolution mismatch')
            raw=color.copy(); gray=cv2.cvtColor(color,cv2.COLOR_BGR2GRAY)
            found,corners=cv2.findChessboardCornersSB(gray,(8,6),cv2.CALIB_CB_NORMALIZE_IMAGE)
            reason='Board not detected'; board=None; rms=None
            if found:
                cv2.drawChessboardCorners(color,(8,6),corners,True)
                ok,rv,tv=cv2.solvePnP(obj,corners,K,D)
                if ok:
                    projected,_=cv2.projectPoints(obj,rv,tv,K,D)
                    rms=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-corners.reshape(-1,2))**2,axis=1))))
                    board=np.eye(4); board[:3,:3]=cv2.Rodrigues(rv)[0]; board[:3,3]=tv.ravel()
                    if rms>0.8 or tv[2,0]<=0: board=None
                reason='Board pose poor' if board is None else 'No fresh robot TF'
            robot=None; tfstamp=None
            try:
                tf=buf.lookup_transform('base_link','wrist_pitch_link',Time())
                tfstamp=Time.from_msg(tf.header.stamp).nanoseconds
                age=(node.get_clock().now().nanoseconds-tfstamp)/1e9
                if not -0.05<=age<=0.3: raise ValueError('Stale robot TF')
                robot=matrix(tf.transform)
                history.append((time.monotonic(),robot))
                while history and history[0][0]<time.monotonic()-2.5: history.popleft()
                stable=len(history)>=3 and history[-1][0]-history[0][0]>=2.
                if stable:
                    for _,old in history:
                        angle=np.linalg.norm(cv2.Rodrigues(old[:3,:3].T@robot[:3,:3])[0])
                        if np.linalg.norm(old[:3,3]-robot[:3,3])>0.002 or angle>np.deg2rad(0.5): stable=False; break
                if board is not None: reason='READY: S to save' if stable else 'Wait: robot must be still 2 sec'
                if not stable: robot=None
            except (TransformException,ValueError):
                history.clear(); robot=None
            duplicate=robot is not None and any(
                np.linalg.norm(robot[:3,3]-p[:3,3])<.01 and
                np.linalg.norm(cv2.Rodrigues(p[:3,:3].T@robot[:3,:3])[0])<np.deg2rad(5)
                for p in saved)
            if duplicate and board is not None:
                reason='POSE ALREADY SAVED: change camera pose'
            if reason != previous_reason:
                print(f'{reason}; corners={found}; RMS={rms}',flush=True)
                previous_reason=reason
            cv2.putText(color,f'8x6 corners | {args.square_mm:g} mm | saved={len(saved)}',(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            cv2.putText(color,reason,(8,45),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0) if robot is not None and board is not None else (0,0,255),1)
            cv2.putText(color,'S: save | Q: exit | Keep board fixed',(8,68),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            if time.monotonic()<notice_until:
                cv2.rectangle(color,(0,80),(color.shape[1],112),(0,0,0),-1)
                cv2.putText(color,notice,(8,102),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),1)
            cv2.imshow('Hand-eye calibration - 18mm board',color)
            key=cv2.waitKey(1)&255
            if key in (27,ord('q')): break
            if key in (ord('s'),ord('S')):
                notice_until=time.monotonic()+5
                if board is None or robot is None:
                    notice='NOT SAVED: '+reason
                    print(notice,flush=True)
                elif duplicate:
                    notice='NOT SAVED: change position >1cm or angle >5deg'
                    print(notice,flush=True)
                else:
                    stem=str(time.time_ns())
                    record={'square_m':args.square_mm/1000,'inner_corners':[8,6],
                            'parent_frame':'wrist_pitch_link','base_frame':'base_link',
                            'camera_frame':'camera_color_optical_frame',
                            'host_receive_stamp_ns':stamp.nanoseconds,'tf_stamp_ns':tfstamp,
                            'timestamp_policy':'latest fresh TF, stationary capture only',
                            'T_base_mount':robot.tolist(),'T_camera_board':board.tolist(),
                            'K':K.tolist(),'D':D.tolist(),'corners':corners.reshape(-1,2).tolist(),
                            'reprojection_rms_px':rms,'image':stem+'.png'}
                    if not cv2.imwrite(str(out/(stem+'.png')),raw): raise RuntimeError('Image save failed')
                    (out/(stem+'.json')).write_text(json.dumps(record,indent=2))
                    saved.append(robot)
                    notice=f'SAVED sample {len(saved)} RMS={rms:.3f}px'
                    print(notice,flush=True)
            count+=1
            if args.frames and count>=args.frames: break
    finally:
        if started: pipe.stop()
        cv2.destroyAllWindows(); listener.unregister(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__':
    try: main()
    except KeyboardInterrupt: pass
