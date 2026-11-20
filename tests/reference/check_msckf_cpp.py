"""Check production results against the independent matrix-exponential oracle."""
import subprocess
import sys
import numpy as np
from msckf import mean, rotation

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
