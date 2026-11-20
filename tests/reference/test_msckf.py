"""Numerical checks of Draft equations; not exhaustive or machine-checked proofs."""
import unittest
import numpy as np
import mpmath as mp
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation
from msckf import coefficients, mean, transition, physical_transition, gauge, hat, rotation, integrals


class Equations(unittest.TestCase):
    def test_integrals_high_precision_and_limits(self):
        for w in ([0., 0., 0.], [1e-12, -1e-12, 0.], [6., -8., 0.]):
            with mp.workdps(60):
                system = mp.zeros(9)
                W = hat(np.array(w))
                for i in range(3):
                    for j in range(3):
                        system[i,j] = mp.mpf(float(W[i,j]))
                    system[i,i+3] = system[i+3,i+6] = 1
                exact = mp.expm(mp.mpf('0.01')*system)
                _, A1, A2 = integrals(np.array(w), .01)
                np.testing.assert_allclose(A1, np.array(exact[:3,3:6].tolist(), float), atol=1e-17, rtol=1e-14)
                np.testing.assert_allclose(A2, np.array(exact[:3,6:9].tolist(), float), atol=1e-19, rtol=1e-14)
        a = np.array([3., -2., 4.]); h = .01
        _, A1, A2, B1, B2 = coefficients(a, np.zeros(3), h)
        for actual, expected in ((A1,h*np.eye(3)), (A2,h*h/2*np.eye(3)),
                                 (B1,h*h/2*hat(a)), (B2,h**3/6*hat(a))):
            np.testing.assert_allclose(actual, expected, atol=1e-18, rtol=1e-14)

    def test_physical_derivative_and_fej_gauge(self):
        rng = np.random.default_rng(20261121)
        gravity = np.array([0., 0., 9.81])
        for _ in range(16):
            R = rotation(rng.normal(size=3)); p, v = rng.normal(size=(2,3))
            a, w = rng.normal(size=(2,3)); h = .01
            end = mean(R,p,v,a,w,h,gravity)
            Phi, _ = transition((R,p,v),end,a,w,h,gravity)
            np.testing.assert_allclose(Phi,physical_transition(R,a,w,h),atol=1e-12,rtol=1e-12)
            numerical = np.zeros((15,15)); epsilon = 1e-5
            for axis in range(15):
                values = []
                for sign in (1,-1):
                    d = np.eye(15)[axis]*epsilon*sign
                    Rn,pn,vn = mean(R@rotation(d[6:9]),p+d[:3],v+d[3:6],a-d[9:12],w-d[12:],h,gravity)
                    values.append(np.r_[pn-end[1],vn-end[2],Rotation.from_matrix(end[0].T@Rn).as_rotvec(),d[9:]])
                numerical[:,axis] = (values[0]-values[1])/(2*epsilon)
            np.testing.assert_allclose(Phi,numerical,atol=2e-9,rtol=2e-9)
            first = (R@rotation(rng.normal(size=3)*.1),p+.1,v-.2)
            Psi,_ = transition(first,end,a,w,h,gravity)
            np.testing.assert_allclose(Psi@gauge(*first),gauge(*end),atol=1e-12,rtol=1e-12)
            self.assertGreater(np.linalg.norm(Psi-Phi),1e-3)

    def test_noise_scaling_and_segment_recurrence(self):
        h=.01; I=np.eye(3); zero=np.zeros(3); gravity=np.array([0.,0.,9.81])
        Qc=np.zeros((12,12)); Qc[:3,:3]=I
        def step(dt):
            end=mean(I,zero,zero,zero,zero,dt,gravity)
            F,G=transition((I,zero,zero),end,zero,zero,dt,gravity)
            return F,G@(Qc/dt)@G.T
        _,Q=step(h)
        np.testing.assert_allclose(Q[:3,:3],I*h**3/4,atol=1e-20)
        np.testing.assert_allclose(Q[:3,3:6],I*h*h/2,atol=1e-20)
        np.testing.assert_allclose(Q[3:6,3:6],I*h,atol=1e-18)
        Fhalf,Qhalf=step(h/2)
        total=Fhalf@Qhalf@Fhalf.T+Qhalf
        np.testing.assert_allclose(total[:3,:3],I*5*h**3/16,atol=1e-20)
        self.assertGreater(np.linalg.norm(total-Q),1e-9)
        Qc[:3,:3]=0; Qc[6:,6:]=np.eye(6)
        _,Q=step(h)
        np.testing.assert_allclose(Q,block_diag(np.zeros((9,9)),h*np.eye(6)),atol=1e-18)

    def test_clone_selection_joseph_and_identity_reset(self):
        rng=np.random.default_rng(13)
        L=rng.normal(size=(15,12)); P=L@L.T
        select=np.eye(15)[[0,1,2,6,7,8]]
        augment=np.vstack((np.eye(15),select)); Pa=augment@P@augment.T
        np.testing.assert_allclose(Pa[:15,15:],P@select.T)
        np.testing.assert_allclose(Pa[15:,15:],select@P@select.T)
        keep=np.eye(21)[:15]
        np.testing.assert_allclose(keep@Pa@keep.T,P)
        H=rng.normal(size=(4,21)); V=np.eye(4)
        K=np.linalg.solve(H@Pa@H.T+V,H@Pa).T
        A=np.eye(21)-K@H
        joseph=A@Pa@A.T+K@V@K.T
        np.testing.assert_allclose(joseph,Pa-K@H@Pa,atol=1e-12,rtol=1e-12)
        self.assertGreaterEqual(np.linalg.eigvalsh(joseph).min(),-1e-12)
        d=np.array([.1,-.1,.08]); Jr=integrals(-d,1)[1]
        self.assertGreater(np.linalg.norm(Jr-np.eye(3)),.01)
        # Physical reset is Jr; the selected FEJ reset is explicitly identity.
        derivative=np.column_stack([(Rotation.from_matrix(rotation(-d)@rotation(d+np.eye(3)[j]*1e-6)).as_rotvec()-Rotation.from_matrix(rotation(-d)@rotation(d-np.eye(3)[j]*1e-6)).as_rotvec())/2e-6 for j in range(3)])
        np.testing.assert_allclose(derivative,Jr,atol=1e-9,rtol=1e-9)


if __name__ == '__main__':
    unittest.main()
