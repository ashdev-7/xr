"""Diagnose an estimate: tilt error (gravity direction in body frame) and speed errors over time."""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from score import load_tum, associate, umeyama
def R_of(q):  # x,y,z,w
    x,y,z,w=q.T
    return np.stack([np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],-1),
                     np.stack([2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],-1),
                     np.stack([2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],-1)],1)
est, d = sys.argv[1], sys.argv[2]
meta=json.load(open(os.path.join(d,'meta.json')))
tg,pg,qg=load_tum(os.path.join(d,'groundtruth.txt')); te,pe,qe=load_tum(est)
ie,ig=associate(te,tg); te,pe,qe,pg,qg=te[ie],pe[ie],qe[ie],pg[ig],qg[ig]
pre=te<meta['fail_time']
ge=np.einsum('nji,j->ni',R_of(qe),[0,0,1]); gg=np.einsum('nji,j->ni',R_of(qg),[0,0,1])
tilt=np.degrees(np.arccos(np.clip((ge*gg).sum(1),-1,1)))
R,t=umeyama(pe[pre],pg[pre]); err=np.linalg.norm(pe@R.T+t-pg,axis=1)
T=te-te[0]
for s in [1,5,20,40,60,90,120,int(T[pre][-1])]:
    i=np.searchsorted(T,s); 
    if i<len(T): print(f"t={s:4d}s tilt_err={tilt[i]:6.2f}deg pos_err={err[i]:6.2f}m")
print("tilt err mean %.2f deg, p95 %.2f"%(tilt[pre].mean(),np.percentile(tilt[pre],95)))
