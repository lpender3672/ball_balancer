"""
Servo + camera-delay augmented plant, full-order Hinf via projection-form LMI.
Camera/CV latency T_cam modelled as a Pade RHP zero on the OUTPUT (measurement) path.
"""
import numpy as np, cvxpy as cp, control as ct
from scipy.linalg import null_space, block_diag
b=5/7*9.81; wn=2*np.pi*10; tau=1/wn; Td=0.015; r0=0.15; cfric=0.2
Tcam=0.030   # camera + CV total measurement latency

def build(gain, wb, Mhf, eps, wuc, uhf, noise, ws):
    # input side: servo lag * servo Pade ; plant: friction double integrator ;
    # output side: camera Pade (measurement delay)
    Ai=np.array([[-cfric,1],[0,-cfric]],float); Bi=np.array([[0],[gain]]); Ci=np.array([[1,0]])
    servo=ct.tf2ss(ct.tf([1.0],[tau,1.0])*ct.tf([-Td/2,1],[Td/2,1]))
    cam  =ct.tf2ss(ct.tf([-Tcam/2,1],[Tcam/2,1]))
    # series cmd -> servo -> tilt -> ball -> cam -> measured
    Gss = ct.series(servo, ct.ss(Ai,Bi,Ci,0.0))   # cmd -> position
    Gss = ct.series(Gss, cam)                       # position -> measured (delayed)
    Ap,Bp,Cp,Dp = Gss.A,Gss.B,Gss.C,Gss.D
    nx=Ap.shape[0]
    We=ct.tf2ss(ct.tf([1.0/Mhf, wb],[1.0, wb*eps]))
    Wu=ct.tf2ss(ct.tf([uhf, wuc*0.05],[1.0, wuc]))
    Awe,Bwe,Cwe,Dwe=We.A,We.B,We.C,We.D; Awu,Bwu,Cwu,Dwu=Wu.A,Wu.B,Wu.C,Wu.D
    nWe=Awe.shape[0];nWu=Awu.shape[0];nP=nx+nWe+nWu
    A=np.zeros((nP,nP));A[:nx,:nx]=Ap
    A[nx:nx+nWe,nx:nx+nWe]=Awe;A[nx:nx+nWe,:nx]=-Bwe@Cp;A[nx+nWe:,nx+nWe:]=Awu
    B1=np.zeros((nP,2));B1[nx:nx+nWe,0:1]=Bwe
    B2=np.zeros((nP,1));B2[:nx,:]=Bp;B2[nx+nWe:,:]=Bwu
    # NOTE: We sees -Cp x (delayed position) and Dwe feedthrough of (ref - delayed pos).
    C1=np.zeros((2,nP));C1[0:1,nx:nx+nWe]=Cwe;C1[0:1,:nx]=-Dwe@Cp;C1[1:2,nx+nWe:]=Cwu
    D11=np.zeros((2,2));D11[0,0]=Dwe[0,0];D12=np.zeros((2,1));D12[1:2,:]=Dwu
    # measured y = ref - (delayed position) + noise  ; delayed position = Cp x + Dp u
    C2=np.zeros((1,nP));C2[:,:nx]=-Cp
    D21=np.zeros((1,2));D21[0,0]=1.0;D21[0,1]=noise
    D22=-Dp   # direct feedthrough cmd->measured via cam Pade D (nonzero!)
    Pphys=dict(A=A,B1=B1,B2=B2,C1=C1,D11=D11,D12=D12,C2=C2,D21=D21,D22=D22,n=nP)
    Asc =dict(A=A/ws,B1=B1/ws,B2=B2/ws,C1=C1,D11=D11,D12=D12,C2=C2,D21=D21,D22=D22,n=nP)
    return Pphys,Asc

def proj_solve(gain, wb, Mhf, eps, wuc, uhf, noise, ws):
    Pphys,Asc=build(gain,wb,Mhf,eps,wuc,uhf,noise,ws)
    A,B1,B2=Asc['A'],Asc['B1'],Asc['B2']; C1,D11,D12=Asc['C1'],Asc['D11'],Asc['D12']
    C2,D21,D22,n=Asc['C2'],Asc['D21'],Asc['D22'],Asc['n']
    nw=B1.shape[1];nz=C1.shape[0];nu=B2.shape[1];ny=C2.shape[0]
    # projection form assumes D22=0; absorb D22 by loop-shifting is complex.
    # Since |D22| here is tiny (cam Pade D = -1 *only at HF*, but Dp includes the
    # ball integrator which kills DC); check magnitude and treat as ~0 for synthesis,
    # then VERIFY on the true plant including D22.
    NY=null_space(np.hstack([B2.T,D12.T])); NX=null_space(np.hstack([C2,D21]))
    PiY=np.block([[NY,np.zeros((NY.shape[0],nw))],[np.zeros((nw,NY.shape[1])),np.eye(nw)]])
    PiX=np.block([[NX,np.zeros((NX.shape[0],nz))],[np.zeros((nz,NX.shape[1])),np.eye(nz)]])
    def feas(gv,ftol):
        Y=cp.Variable((n,n),symmetric=True);X=cp.Variable((n,n),symmetric=True)
        IY=cp.bmat([[A@Y+Y@A.T,Y@C1.T,B1],[C1@Y,-gv*np.eye(nz),D11],[B1.T,D11.T,-gv*np.eye(nw)]])
        IX=cp.bmat([[A.T@X+X@A,X@B1,C1.T],[B1.T@X,-gv*np.eye(nw),D11.T],[C1,D11,-gv*np.eye(nz)]])
        cons=[PiY.T@IY@PiY<<-1e-9*np.eye(PiY.shape[1]),PiX.T@IX@PiX<<-1e-9*np.eye(PiX.shape[1]),
              cp.bmat([[X,np.eye(n)],[np.eye(n),Y]])>>1e-7*np.eye(2*n),X>>1e-7*np.eye(n),Y>>1e-7*np.eye(n)]
        prob=cp.Problem(cp.Minimize(0),cons)
        try:
            prob.solve(solver=cp.CVXOPT,kktsolver='robust',max_iters=500,abstol=ftol,reltol=ftol,feastol=ftol)
            if prob.status=='optimal': return (X.value,Y.value)
        except Exception: pass
        return None
    lo,hi=0.8,4.0; sol=None
    for _ in range(15):
        mid=0.5*(lo+hi); s=feas(mid,1e-8)
        if s is not None: hi=mid; sol=s
        else: lo=mid
    if sol is None: return None
    Xv,Yv=sol; gv=hi*1.05
    Ah=cp.Variable((n,n));Bh=cp.Variable((n,ny));Ch=cp.Variable((nu,n));Dh=cp.Variable((nu,ny))
    AY=A@Yv+B2@Ch;XA=Xv@A+Bh@C2
    R11=AY+AY.T;R12=Ah.T+(A+B2@Dh@C2);R22=XA+XA.T
    B1t=B1+B2@Dh@D21;XB1=Xv@B1+Bh@D21
    C1Y=C1@Yv+D12@Ch;C1X=C1+D12@Dh@C2;D11e=D11+D12@Dh@D21
    Mm=cp.bmat([[R11,R12,B1t,C1Y.T],[R12.T,R22,XB1,C1X.T],[B1t.T,XB1.T,-gv*np.eye(nw),D11e.T],[C1Y,C1X,D11e,-gv*np.eye(nz)]])
    cp.Problem(cp.Minimize(0),[Mm<<-1e-9*np.eye(Mm.shape[0])]).solve(solver=cp.CVXOPT,kktsolver='robust',max_iters=500,abstol=1e-9,reltol=1e-9,feastol=1e-9)
    N2T=np.eye(n)-Xv@Yv;N2Ti=np.linalg.inv(N2T)
    Dk=Dh.value;Ck=(Ch.value-Dk@(C2@Yv))@N2Ti;Bk=(Bh.value-(Xv@B2)@Dk)
    Ak=(Ah.value-Xv@(A+B2@Dk@C2)@Yv-(Bk@C2)@Yv-(Xv@B2)@Ck@N2T)@N2Ti
    Ak=ws*Ak;Bk=ws*Bk
    # VERIFY on physical plant INCLUDING D22 (true LFT with D22!=0)
    Pp=Pphys; D22p=Pp['D22']
    # closed loop with D22: u=Ck xk+Dk(y), y=C2 x+D21 w+D22 u -> solve for u
    # (I-Dk D22)^-1 etc. Build exact LFT.
    Im=np.eye(nu)
    Minv=np.linalg.inv(Im-Dk@D22p)
    Ac=Pp['A']; B2c=Pp['B2']; C2c=Pp['C2']; B1c=Pp['B1']; C1c=Pp['C1']; D11c=Pp['D11']; D12c=Pp['D12']; D21c=Pp['D21']
    # u = Minv(Ck xk + Dk(C2 x + D21 w)); then x,xk dynamics
    Acl=np.block([[Ac+B2c@Minv@Dk@C2c, B2c@Minv@Ck],
                  [Bk@C2c+Bk@D22p@Minv@Dk@C2c, Ak+Bk@D22p@Minv@Ck]])
    Bcl=np.block([[B1c+B2c@Minv@Dk@D21c],[Bk@D21c+Bk@D22p@Minv@Dk@D21c]])
    Ccl=np.block([C1c+D12c@Minv@Dk@C2c, D12c@Minv@Ck]); Dcl=D11c+D12c@Minv@Dk@D21c
    ev=np.linalg.eigvals(Acl).real.max()
    cl=ct.ss(Acl,Bcl,Ccl,Dcl);w=np.logspace(-3,4,7000);H=cl(1j*w)
    sv=max(np.linalg.svd(H[:,:,k],compute_uv=False)[0] for k in range(len(w)))
    return Ak,Bk,Ck,Dk,ev,sv,hi,float(np.abs(D22p).max())

if __name__=='__main__':
    ws=wn
    print("Tcam=%.0f ms, camera RHP zero at %.1f rad/s"%(Tcam*1000,2/Tcam),flush=True)
    R=proj_solve(b, 0.45,3.0,0.3,8.0,0.5,3e-3, ws)
    print("radial : CLeig=%+.4f trueNorm=%.4f LMIgamma=%.4f |D22|=%.3f"%(R[4],R[5],R[6],R[7]),flush=True)
    Aax=proj_solve(b/r0, 0.35,3.0,0.3,8.0,0.5,3e-3, ws)
    print("angular: CLeig=%+.4f trueNorm=%.4f LMIgamma=%.4f"%(Aax[4],Aax[5],Aax[6]),flush=True)
    if R[4]<0 and Aax[4]<0:
        Ak=block_diag(R[0],Aax[0]);Bk=block_diag(R[1],Aax[1]);Ck=block_diag(R[2],Aax[2]);Dk=block_diag(R[3],Aax[3])
        np.savez('controller_cam.npz',Ak=Ak,Bk=Bk,Ck=Ck,Dk=Dk,
                 gamma=max(R[5],Aax[5]),hinf_r=R[5],hinf_p=Aax[5],
                 wn=wn,tau=tau,Td=Td,Tcam=Tcam,b=b,r0=r0,method='projection-LMI+camera')
        print("SAVED servo+camera controller order %d, max true norm=%.4f"%(Ak.shape[0],max(R[5],Aax[5])),flush=True)
