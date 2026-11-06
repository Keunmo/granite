# Angular triangulation residual of the dumped tracks for the GT translation scaled by c (0: pure rotation).
# Usage: python3 moving_start_angular_check.py DUMP STEP (needs moving_start_linear_check.py in the same folder)
import numpy as np,sys
exec(open('' + __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'moving_start_linear_check.py') + '').read().split('step=int')[0])
from scipy.optimize import least_squares
for n,W in enumerate(windows(sys.argv[1])):
    if n%int(sys.argv[2]): continue
    P=np.array(W['P']); N=len(P); B=np.array(W['B']); Tci=W['T']; Rci,tci=Tci[:,:3],Tci[:,3]
    pt=P[:,0].astype(np.int64); Rk=P[:,2:11].reshape(-1,3,3)
    i0=np.argmin(np.abs(gt-pt[0])); Rw0=rotvec(g[i0,1:4])@R_RI
    pk_gt=np.array([Rw0.T@(np.array([np.interp(t,gt,gp[:,c]) for c in range(3)])-gp[i0]) for t in pt])
    npnt=int(B[:,0].max())+1
    def angcost(pk, Rs):
        # triangulate each point (angular, nonlinear) and return median angular residual [deg]
        res=[]
        for f in range(npnt):
            rows=B[B[:,0]==f]
            # linear init: algebraic LS
            As=[];bs=[]
            for r in rows:
                k=int(r[1]); T=orth(r[2:5]); Y=T@Rci@Rs[k].T; As.append(Y); bs.append(Y@pk[k]-T@tci)
            pF=np.linalg.lstsq(np.vstack(As),np.concatenate(bs),rcond=None)[0]
            def fun(p):
                out=[]
                for r in rows:
                    k=int(r[1]); pc=Rci@Rs[k].T@(p-pk[k])+tci; out.append(np.cross(pc/np.linalg.norm(pc),r[2:5]))
                return np.concatenate(out)
            s=least_squares(fun,pF)
            res.append(np.degrees(np.sqrt(np.mean(s.fun**2))))
        return np.median(res), np.percentile(res,90)
    # GT rotations too
    Rgt=np.array([Rw0.T@rotvec(g[np.argmin(np.abs(gt-t)),1:4])@R_RI for t in pt])
    out=[]
    for c in [0,0.25,0.5,1.0,1.5]:
        out.append(f"c={c}: imuR {angcost(c*pk_gt,Rk)[0]:.4f} gtR {angcost(c*pk_gt,Rgt)[0]:.4f}")
    print(f"W{n} |dp|={np.linalg.norm(pk_gt[-1]):.1f}m", ' | '.join(out))
