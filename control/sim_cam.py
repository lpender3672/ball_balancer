"""Nonlinear sim WITH servo (input) + camera (output) delay, both in the loop."""
import numpy as np, control as ct, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

d=np.load('controller_cam.npz')
Ak,Bk,Ck,Dk=d['Ak'],d['Bk'],d['Ck'],d['Dk']
b=float(d['b']);wn=float(d['wn']);tau=float(d['tau']);Td=float(d['Td']);Tcam=float(d['Tcam']);r0=float(d['r0'])
cfric=0.2
servo=ct.tf2ss(ct.tf([1.0],[tau,1.0])*ct.tf([-Td/2,1],[Td/2,1])); As,Bs,Cs,Ds=servo.A,servo.B,servo.C,servo.D; ns=As.shape[0]
cam=ct.tf2ss(ct.tf([-Tcam/2,1],[Tcam/2,1])); Ac_,Bc_,Cc_,Dc_=cam.A,cam.B,cam.C,cam.D; nc=Ac_.shape[0]
nk=Ak.shape[0]

def dyn(t,s,ref):
    i=0
    r,rd,phi,phid=s[0:4]; i=4
    xsr=s[i:i+ns];i+=ns
    xsp=s[i:i+ns];i+=ns
    xcr=s[i:i+nc];i+=nc          # camera state radial
    xcp=s[i:i+nc];i+=nc          # camera state angular
    xk=s[i:i+nk]
    r=max(r,1e-3)
    # measured (delayed) positions via camera Pade
    r_meas=(Cc_@xcr+Dc_.flatten()*r).item()
    phi_meas=(Cc_@xcp+Dc_.flatten()*phi).item()
    y=np.array([ref[0]-r_meas, ref[1]-phi_meas])
    ucmd=Ck@xk+Dk@y; xkd=Ak@xk+Bk@y
    th_r=(Cs@xsr+Ds.flatten()*ucmd[0]).item(); xsrd=As@xsr+Bs.flatten()*ucmd[0]
    th_p=(Cs@xsp+Ds.flatten()*ucmd[1]).item(); xspd=As@xsp+Bs.flatten()*ucmd[1]
    th_r=np.clip(th_r,-0.4,0.4); th_p=np.clip(th_p,-0.4,0.4)
    a_r=b*th_r; a_phi=b*th_p
    rdd=(r*phid**2+a_r)-2*cfric*rd
    phidd=(-2*rd*phid+a_phi)/r-2*cfric*phid
    xcrd=Ac_@xcr+Bc_.flatten()*r
    xcpd=Ac_@xcp+Bc_.flatten()*phi
    return np.concatenate([[rd,rdd,phid,phidd],xsrd,xspd,xcrd,xcpd,xkd])

ref=np.array([0.15,0.0])
s0=np.concatenate([[-0.12,0,0.30,0],np.zeros(ns),np.zeros(ns),np.zeros(nc),np.zeros(nc),np.zeros(nk)])
sol=solve_ivp(dyn,[0,15],s0,args=(ref,),t_eval=np.linspace(0,15,900),method='LSODA',rtol=1e-6,atol=1e-9)
r=sol.y[0];phi=sol.y[2];t=sol.t
x=r*np.cos(phi);y=r*np.sin(phi);xr,yr=0.15,0.0

# per-axis loop incl both delays for plots
def loop(gain,Akx,Bkx,Ckx,Dkx):
    P=ct.tf([gain],[1,2*cfric,cfric**2])*ct.tf([1.0],[tau,1.0])*ct.tf([-Td/2,1],[Td/2,1])*ct.tf([-Tcam/2,1],[Tcam/2,1])
    K=ct.ss(Akx,Bkx,Ckx,Dkx); return ct.feedback(1,P*K),ct.feedback(P*K,1)
Sr,Tr=loop(b,Ak[:7,:7],Bk[:7,:1],Ck[:1,:7],Dk[:1,:1])
Sp,Tp=loop(b/r0,Ak[7:,7:],Bk[7:,1:],Ck[1:,7:],Dk[1:,1:])
w=np.logspace(-2,3,600)

fig,ax=plt.subplots(2,2,figsize=(13,9))
ax[0,0].plot(t,r,label='r [m]');ax[0,0].axhline(0.15,ls='--',c='k',lw=.8)
ax[0,0].plot(t,phi,label='phi [rad]');ax[0,0].axhline(0,ls=':',c='gray',lw=.8)
ax[0,0].set_title('States: servo + camera delay in loop');ax[0,0].set_xlabel('t [s]');ax[0,0].legend();ax[0,0].grid(alpha=.3)
ax[0,1].plot(x,y,lw=1.4);ax[0,1].plot(x[0],y[0],'go',label='start');ax[0,1].plot(xr,yr,'r*',ms=14,label='target')
th=np.linspace(0,2*np.pi,100);ax[0,1].plot(0.2*np.cos(th),0.2*np.sin(th),'k-',lw=.5,alpha=.4)
ax[0,1].set_aspect('equal');ax[0,1].set_title('Ball trajectory');ax[0,1].set_xlabel('x [m]');ax[0,1].set_ylabel('y [m]');ax[0,1].legend();ax[0,1].grid(alpha=.3)
ax[1,0].loglog(w,np.abs(Sr(1j*w)),label='|S| radial');ax[1,0].loglog(w,np.abs(Sp(1j*w)),label='|S| angular')
ax[1,0].axvline(2/Tcam,ls=':',c='g',lw=1,label='cam RHP zero 66.7');ax[1,0].axvline(2/Td,ls=':',c='m',lw=1,label='servo RHP zero 133')
ax[1,0].axvline(wn,ls=':',c='r',lw=.8,label='servo bw')
ax[1,0].set_title('Sensitivity (both delay limits marked)');ax[1,0].set_xlabel('rad/s');ax[1,0].legend(fontsize=7);ax[1,0].grid(alpha=.3,which='both')
ax[1,1].loglog(w,np.abs(Tr(1j*w)),label='|T| radial');ax[1,1].loglog(w,np.abs(Tp(1j*w)),label='|T| angular')
ax[1,1].axvline(2/Tcam,ls=':',c='g',lw=1);ax[1,1].set_title('Complementary sensitivity');ax[1,1].set_xlabel('rad/s');ax[1,1].legend();ax[1,1].grid(alpha=.3,which='both')
plt.tight_layout();plt.savefig('cam_results.png',dpi=110)

ir=np.where(np.abs(r-0.15)<0.02*0.15)[0]
print("settling r 2%%: %.2f s"%(t[ir[0]] if len(ir) else -1))
print("final r err %.2f mm, phi err %.3f deg"%(abs(r[-1]-0.15)*1000,np.degrees(abs(phi[-1]))))
print("peak |S| radial=%.3f angular=%.3f"%(np.abs(Sr(1j*w)).max(),np.abs(Sp(1j*w)).max()))
bwr=w[np.where(np.abs(Tr(1j*w))>=1/np.sqrt(2))[0][-1]] if np.any(np.abs(Tr(1j*w))>=1/np.sqrt(2)) else 0
print("radial bandwidth ~ %.2f rad/s"%bwr)
