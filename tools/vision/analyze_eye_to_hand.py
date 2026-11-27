import argparse,json,pathlib,numpy as np,cv2
ap=argparse.ArgumentParser(description='Offline eye-to-hand consistency analysis; no TF publication or motion')
ap.add_argument('captures',type=pathlib.Path)
args=ap.parse_args();p=args.captures;fs=sorted(p.glob('*.json'));ds=[json.loads(f.read_text()) for f in fs]
if len(ds)<4:raise ValueError('Need at least four samples for leave-one-out analysis')
if any(d.get('calibration_mode')!='eye-to-hand' for d in ds):raise ValueError('Eye-to-hand samples required')
B=np.array([d['T_base_mount'] for d in ds]);C=np.array([d['T_camera_board'] for d in ds]);A=np.linalg.inv(B)
def solve(a,c,method):
 r,t=cv2.calibrateHandEye(list(a[:,:3,:3]),list(a[:,:3,3]),list(c[:,:3,:3]),list(c[:,:3,3]),method=method);x=np.eye(4);x[:3,:3]=r;x[:3,3]=t.ravel();return x
def mean(ts):
 t=np.eye(4);u,_,v=np.linalg.svd(ts[:,:3,:3].sum(axis=0));t[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;t[:3,3]=ts[:,:3,3].mean(axis=0);return t
def errors(ts,m):
 return np.linalg.norm(ts[:,:3,3]-m[:3,3],axis=1)*1000,np.array([np.degrees(np.arccos(np.clip((np.trace(m[:3,:3].T@t[:3,:3])-1)/2,-1,1))) for t in ts])
out={}
for name in ['TSAI','PARK','HORAUD','ANDREFF','DANIILIDIS']:
 try:
  method=getattr(cv2,'CALIB_HAND_EYE_'+name);x=solve(A,C,method);ys=A@x@C;m=mean(ys);e,r=errors(ys,m);le=[];lr=[]
  for i in range(len(ds)):
   mask=np.arange(len(ds))!=i;xx=solve(A[mask],C[mask],method);mm=mean(A[mask]@xx@C[mask]);ee,rr=errors((A[i]@xx@C[i])[None],mm);le.append(float(ee[0]));lr.append(float(rr[0]))
  out[name]={'T_base_camera':x.tolist(),'fit_mm':e.tolist(),'fit_deg':r.tolist(),'heldout_mm':le,'heldout_deg':lr}
  print(name,'camera xyz',np.round(x[:3,3],4),'fit mean/max mm',np.round([e.mean(),e.max()],2),'deg',np.round([r.mean(),r.max()],2),'leaveout mean/max mm',np.round([np.mean(le),max(le)],2))
 except Exception as ex: print(name,str(ex))
for i,f in enumerate(fs):print(i+1,f.name,'RMS',round(ds[i]['reprojection_rms_px'],3),'PARK fit',round(out['PARK']['fit_mm'][i],1),'deg',round(out['PARK']['fit_deg'][i],1),'heldout',round(out['PARK']['heldout_mm'][i],1))
path= p.parent.parent/'analysis';path.mkdir(exist_ok=True);(path/f'eye_to_hand_{p.name}_analysis.json').write_text(json.dumps({'samples':[f.name for f in fs],'results':out,'status':'diagnostic only; not published as TF'},indent=2))
