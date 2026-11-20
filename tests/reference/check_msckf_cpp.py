"""Check production results against the independent matrix-exponential oracle."""
import subprocess
import sys
import numpy as np
from msckf import mean, rotation, transition, physical_transition, gauge
from scipy.linalg import block_diag

rng=np.random.default_rng(513)
maximum=0.
for duration in (1, 1000, 5_000_000, 10_000_000):
    for case in range(8):
        angle=rng.normal(size=3); p,v=rng.normal(size=(2,3))
        a=rng.normal(size=3); a*=100/np.linalg.norm(a)
        w=rng.normal(size=3); w*=10/np.linalg.norm(w)
        if case==0: w[:]=0
        if case==1: w*=1e-12
        # Stay inside the input bound despite double rounding of the direction.
        a*=1-1e-15; w*=1-1e-15
        g=np.array([0.,0.,9.81]); h=duration*1e-9
        values=np.r_[angle,p,v,a,w,g]
        text=str(duration)+' '+' '.join(format(x,'.17g') for x in values)+'\n'
        result=subprocess.run([sys.argv[1]],input=text,text=True,capture_output=True,check=True)
        actual=np.fromstring(result.stdout,sep=' ')
        Rn,pn,vn=mean(rotation(angle),p,v,a,w,h,g)
        expected=np.r_[pn,vn,Rn.ravel()]
        np.testing.assert_allclose(actual,expected,rtol=1e-12,atol=1e-12)
        maximum=max(maximum,float(np.max(np.abs(actual-expected))))
print(f'Production nominal prediction: 32 cases; maximum coefficient error {maximum:.3g}')

for duration in (1, 1000, 5_000_000, 10_000_000):
    for case in range(8):
        angle,first_angle=rng.normal(size=(2,3))*.3
        p,v,p0,v0=rng.normal(size=(4,3))
        a=rng.normal(size=3)*20; w=rng.normal(size=3)*2
        if case==0: w[:]=0
        if case==1: w*=1e-12
        if case==2: first_angle=angle; p0=p; v0=v
        if case==4:
            a*=100*(1-1e-15)/np.linalg.norm(a); w*=10*(1-1e-15)/np.linalg.norm(w)
        if case in (6,7):
            w*=((np.pi/360)*(1+(-1 if case==6 else 1)*1e-6))/np.linalg.norm(w)
        factor=rng.normal(size=(12,12))*.001
        Qc=factor@factor.T
        if case==3: Qc[:]=0
        g=np.array([0.,0.,9.81]); h=duration*1e-9
        values=np.r_[angle,p,v,a,w,g,first_angle,p0,v0,Qc.ravel()]
        text=str(duration)+' '+' '.join(format(x,'.17g') for x in values)+'\n'
        result=subprocess.run([sys.argv[1],'covariance'],input=text,text=True,capture_output=True,check=True)
        actual=np.fromstring(result.stdout,sep=' ')
        end=mean(rotation(angle),p,v,a,w,h,g)
        first=(rotation(first_angle),p0,v0)
        Psi,G=transition(first,end,a,w,h,g)
        Qd=G@(Qc/h)@G.T
        np.testing.assert_allclose(actual[15:240].reshape(15,15),Psi,atol=1e-11,rtol=1e-11)
        actual_Qd=actual[240:465].reshape(15,15)
        if np.any(Qc):
            relative=np.linalg.norm(actual_Qd-Qd,np.inf)/np.linalg.norm(Qd,np.inf)
            assert relative<=1e-9, relative
        else:
            np.testing.assert_array_equal(actual_Qd,Qd)
        P=.1*np.eye(27)+.0001*np.outer(np.arange(1,28),np.arange(1,28))
        F=block_diag(Psi,np.eye(12))
        expected=F@P@F.T+block_diag(Qd,np.zeros((12,12)))
        np.testing.assert_allclose(actual[465:].reshape(27,27),expected,atol=1e-11,rtol=1e-11)
        np.testing.assert_allclose(actual[15:240].reshape(15,15)@gauge(*first),gauge(*end),atol=1e-11,rtol=1e-11)
        if case==2:
            np.testing.assert_allclose(actual[15:240].reshape(15,15),physical_transition(rotation(angle),a,w,h),atol=1e-11,rtol=1e-11)
print('Production FEJ/Qd/joint covariance: 32 cases including correlated noise, zero noise and differing first estimates')
