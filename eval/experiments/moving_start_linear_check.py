# Offline check of the moving-start linear system against the GT (Unreal city sequence).
# Input: the dump of eval/experiments/moving_start_trials.patch (GRANITE_INIT_DUMP=<file>).
# Usage: python3 moving_start_linear_check.py DUMP [STEP] [PARALLAX_STEP]
import numpy as np,sys
sys.path.insert(0,'/home/keunmo/workspace/granite_reliability/eval/scripts')
from eval_pose import rotvec,R_RI
g=np.loadtxt('/home/keunmo/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt'); gt=g[:,7]; gp=g[:,4:7]
gv=np.gradient(gp,gt*1e-9,axis=0)
G=np.array([0,9.81,0])
def orth(b):
    ax=np.argmin(np.abs(b)); e=np.zeros(3); e[ax]=1
    t1=np.cross(b,e); t1/=np.linalg.norm(t1); t2=np.cross(b,t1); t2/=np.linalg.norm(t2); return np.vstack([t1,t2])
def windows(path):
    W=None
    for line in open(path):
        t=line.split()
        if t[0]=='W':
            if W: yield W
            W={'newest':int(t[1]),'P':[],'B':[],'X':None}
        elif t[0]=='P': W['P'].append([float(v) for v in t[1:]])
        elif t[0]=='B': W['B'].append([float(v) for v in t[1:]])
        elif t[0]=='X': W['X']=np.array([float(v) for v in t[1:]])
        elif t[0]=='T': W['T']=np.array([float(v) for v in t[1:]]).reshape(3,4)
    if W: yield W
step=int(sys.argv[2]) if len(sys.argv)>2 else 40
for n,W in enumerate(windows(sys.argv[1])):
    if n%step: continue
    P=np.array(W['P']); N=len(P); B=np.array(W['B']); x=W['X']; Tci=W['T']; Rci,tci=Tci[:,:3],Tci[:,3]
    pt=P[:,0].astype(np.int64); DT=P[:,1]; Rk=P[:,2:11].reshape(-1,3,3); al=P[:,11:14]
    npnt=int(B[:,0].max())+1
    i0=np.argmin(np.abs(gt-pt[0])); Rw0=rotvec(g[i0,1:4])@R_RI
    v_gt=Rw0.T@gv[i0]; g_gt=Rw0.T@G
    # GT positions in I0 vs model
    pk_gt=np.array([Rw0.T@(np.array([np.interp(t,gt,gp[:,c]) for c in range(3)])-gp[i0]) for t in pt])
    pk_mod=v_gt[None]*DT[:,None]+0.5*g_gt[None]*DT[:,None]**2+al
    # rotation check
    rerr=[]
    for k in range(N):
        ik=np.argmin(np.abs(gt-pt[k])); Rgt=Rw0.T@rotvec(g[ik,1:4])@R_RI
        rerr.append(np.degrees(np.arccos(np.clip((np.trace(Rgt.T@Rk[k])-1)/2,-1,1))))
    def build(v,gg,pk=None):
        cost=0; pts=[]
        for f in range(npnt):
            rows=B[B[:,0]==f]; As=[];bs=[]
            for r in rows:
                k=int(r[1]); b=r[2:5]; T=orth(b); Y=T@Rci@Rk[k].T
                p_k=v*DT[k]+0.5*gg*DT[k]**2+al[k] if pk is None else pk[k]
                As.append(Y); bs.append(Y@p_k-T@tci)
            A=np.vstack(As); bb=np.concatenate(bs)
            pF=np.linalg.lstsq(A,bb,rcond=None)[0]; cost+=np.sum((A@pF-bb)**2); pts.append(pF)
        return cost,pts
    v_ls=x[3*npnt:3*npnt+3]; g_ls=x[-3:]
    c_ls,_=build(v_ls,g_ls); c_gt,pts=build(v_gt,g_gt); c_gtpos,_=build(None,None,pk_gt)
    # cost along scale: v,g fixed direction? scan speed factor c with g_gt
    depths=[np.linalg.norm(p) for p in pts]
    print(f"W{n} t={pt[-1]*1e-9-1793002753.4946:.2f} N={N} pts={npnt} |v_gt|={np.linalg.norm(v_gt):.2f} |v_ls|={np.linalg.norm(v_ls):.2f} gang={np.degrees(np.arccos(np.clip(g_ls@g_gt/9.81/9.81,-1,1))):.1f} cost_ls={c_ls:.3e} cost_gt={c_gt:.3e} cost_gtpos={c_gtpos:.3e} pkerr={np.abs(pk_gt-pk_mod).max():.2f} roterr={max(rerr):.2f} depth_med={np.median(depths):.1f}")
# parallax per track
print('--- parallax (deg) of the rotation-compensated rays, first vs last pose of each track, per window')
for n,W in enumerate(windows(sys.argv[1])):
    if n%int(sys.argv[3] if len(sys.argv)>3 else 100): continue
    P=np.array(W['P']); B=np.array(W['B']); Tci=W['T']; Rci=Tci[:,:3]
    Rk=P[:,2:11].reshape(-1,3,3); pt=P[:,0]
    par=[];span=[]
    for f in range(int(B[:,0].max())+1):
        rows=B[B[:,0]==f]; k0,k1=int(rows[0,1]),int(rows[-1,1])
        d0=Rk[k0]@Rci.T@rows[0,2:5]; d1=Rk[k1]@Rci.T@rows[-1,2:5]
        par.append(np.degrees(np.arccos(np.clip(d0@d1,-1,1)))); span.append((pt[k1]-pt[k0])*1e-9)
    par=np.array(par)
    print(f"W{n}: parallax deg quantiles 10/50/90 = {np.percentile(par,10):.3f}/{np.median(par):.3f}/{np.percentile(par,90):.3f}, <0.1deg: {np.sum(par<0.1)}/{len(par)}, span med {np.median(span):.2f}s")
