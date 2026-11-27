"""Planar four-bar closure from STEP cylinder centres, millimetres."""
import math
import numpy as np
# X,Z source-CAD coordinates; Y is rotation axis.
A=np.array([2521.99416,357.63245]);B=np.array([2531.74416,377.63245])
C=np.array([2500.88366,378.94786]);D=np.array([2510.44645,398.76081])
NAMES=['gripper_drive_joint','left_rocker_joint','left_finger_joint','right_drive_joint','right_rocker_joint','right_finger_joint']
def angle(v):return math.atan2(v[1],v[0])
def solve(q):
    # q positive rotates left drive about +Y, moves left finger inward.
    c=A+np.array([[math.cos(q),math.sin(q)],[-math.sin(q),math.cos(q)]])@(C-A)
    v=c-B;dist=np.linalg.norm(v);r=np.linalg.norm(D-B);s=np.linalg.norm(D-C)
    along=(r*r-s*s+dist*dist)/(2*dist);h2=r*r-along*along
    if h2<0:raise ValueError('Four-bar cannot close')
    u=v/dist;mid=B+along*u;perp=np.array([-u[1],u[0]])
    ds=[mid+math.sqrt(h2)*perp,mid-math.sqrt(h2)*perp]
    # Assembly branch has the finger's upper pivot to the right of lower pivot.
    d=max(ds,key=lambda p:p[1])
    rocker=-(angle(d-B)-angle(D-B))
    finger_abs=-(angle(d-c)-angle(D-C))
    rel=finger_abs-q
    return [q,rocker,rel,-q,-rocker,-rel],float(np.linalg.norm(d-c)-s)
if __name__=='__main__':
 for q in [0,.1,.3,.5,.7]:print(q,solve(q))
