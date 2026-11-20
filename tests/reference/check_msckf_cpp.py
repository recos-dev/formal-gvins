"""Check production results against the independent matrix-exponential oracle."""
import subprocess
import sys
import numpy as np
from msckf import mean, rotation, transition, physical_transition, gauge, camera
from scipy.linalg import block_diag


def check_nominal(probe, rng):
    maximum = 0.
    for duration in (1, 1000, 5_000_000, 10_000_000):
        for case in range(8):
            angle = rng.normal(size=3)
            p, v = rng.normal(size=(2, 3))
            a = rng.normal(size=3)
            a *= 100/np.linalg.norm(a)
            w = rng.normal(size=3)
            w *= 10/np.linalg.norm(w)
            if case == 0:
                w[:] = 0
            if case == 1:
                w *= 1e-12
            # Stay inside the input bound despite double rounding of the direction.
            a *= 1-1e-15
            w *= 1-1e-15
            g = np.array([0., 0., 9.81])
            h = duration*1e-9
            values = np.r_[angle, p, v, a, w, g]
            text = str(duration)+' '+' '.join(format(x, '.17g') for x in values)+'\n'
            result = subprocess.run([probe], input=text, text=True, capture_output=True, check=True)
            actual = np.fromstring(result.stdout, sep=' ')
            Rn, pn, vn = mean(rotation(angle), p, v, a, w, h, g)
            expected = np.r_[pn, vn, Rn.ravel()]
            np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
            maximum = max(maximum, float(np.max(np.abs(actual-expected))))
    print(f'Production nominal prediction: 32 cases; maximum coefficient error {maximum:.3g}')


def check_covariance(probe, rng):
    for duration in (1, 1000, 5_000_000, 10_000_000):
        for case in range(8):
            angle, first_angle = rng.normal(size=(2, 3))*.3
            p, v, p0, v0 = rng.normal(size=(4, 3))
            a = rng.normal(size=3)*20
            w = rng.normal(size=3)*2
            if case == 0:
                w[:] = 0
            if case == 1:
                w *= 1e-12
            if case == 2:
                first_angle = angle
                p0 = p
                v0 = v
            if case == 4:
                a *= 100*(1-1e-15)/np.linalg.norm(a)
                w *= 10*(1-1e-15)/np.linalg.norm(w)
            if case in (6, 7):
                w *= ((np.pi/360)*(1+(-1 if case == 6 else 1)*1e-6))/np.linalg.norm(w)
            factor = rng.normal(size=(12, 12))*.001
            Qc = factor@factor.T
            if case == 3:
                Qc[:] = 0
            g = np.array([0., 0., 9.81])
            h = duration*1e-9
            values = np.r_[angle, p, v, a, w, g, first_angle, p0, v0, Qc.ravel()]
            text = str(duration)+' '+' '.join(format(x, '.17g') for x in values)+'\n'
            result = subprocess.run([probe, 'covariance'], input=text,
                                    text=True, capture_output=True, check=True)
            actual = np.fromstring(result.stdout, sep=' ')
            end = mean(rotation(angle), p, v, a, w, h, g)
            first = (rotation(first_angle), p0, v0)
            Psi, G = transition(first, end, a, w, h, g)
            Qd = G@(Qc/h)@G.T
            np.testing.assert_allclose(actual[15:240].reshape(15, 15), Psi, atol=1e-11, rtol=1e-11)
            actual_Qd = actual[240:465].reshape(15, 15)
            if np.any(Qc):
                relative = np.linalg.norm(actual_Qd-Qd, np.inf)/np.linalg.norm(Qd, np.inf)
                assert relative <= 1e-9, relative
            else:
                np.testing.assert_array_equal(actual_Qd, Qd)
            P = .1*np.eye(27)+.0001*np.outer(np.arange(1, 28), np.arange(1, 28))
            F = block_diag(Psi, np.eye(12))
            expected = F@P@F.T+block_diag(Qd, np.zeros((12, 12)))
            np.testing.assert_allclose(actual[465:].reshape(
                27, 27), expected, atol=1e-11, rtol=1e-11)
            np.testing.assert_allclose(actual[15:240].reshape(
                15, 15)@gauge(*first), gauge(*end), atol=1e-11, rtol=1e-11)
            if case == 2:
                np.testing.assert_allclose(actual[15:240].reshape(15, 15), physical_transition(
                    rotation(angle), a, w, h), atol=1e-11, rtol=1e-11)
    print('Production FEJ/Qd/joint covariance: 32 cases including correlated noise, zero noise and differing first estimates')


def check_correction(probe, rng):
    for m in (1, 3, 7):
        for trial in range(5):
            H = rng.normal(size=(m, 27))
            A = rng.normal(size=(m, m))
            V = A@A.T+.1*np.eye(m)
            residual = rng.normal(size=m)*.01
            data = np.r_[H.ravel(), V.ravel(), residual]
            text = str(m)+' '+' '.join(format(x, '.17g') for x in data)+'\n'
            result = subprocess.run([probe, 'correct'], input=text,
                                    text=True, capture_output=True, check=True)
            actual = np.fromstring(result.stdout, sep=' ')
            P = .1*np.eye(27)+.0001*np.outer(np.arange(1, 28), np.arange(1, 28))
            K = np.linalg.solve(H@P@H.T+V, H@P).T
            delta = K@residual
            # Conditional covariance is algebraically equivalent to Joseph, but evaluated independently.
            Pc = P-K@H@P
            np.testing.assert_allclose(actual[:27], delta, atol=1e-12, rtol=1e-12)
            np.testing.assert_allclose(actual[27:756].reshape(27, 27), Pc, atol=1e-12, rtol=1e-12)
            expected = np.r_[np.array([1, -2, 3])+delta[:3], np.array([.4, .2, -.1])+delta[3:6],
                             (rotation([.2, -.1, .3])@rotation(delta[6:9])).ravel(),
                             np.array([-.01, .02, .03])+delta[9:12], np.array([.001, .002, .003])+delta[12:15]]
            for c in range(2):
                offset = 15+6*c
                expected = np.r_[expected, np.array([c-1, 2, 3])+delta[offset:offset+3],
                                 (rotation([.1*(c+1), .2, .3])@rotation(delta[offset+3:offset+6])).ravel()]
            np.testing.assert_allclose(actual[756:], expected, atol=1e-12, rtol=1e-12)
    print('Production correction: 15 correlated-noise cases; full Joseph covariance and all IMU/clone means')


def check_camera(probe, rng):
    for trial in range(24):
        angle, first_angle, extrinsic_angle = rng.normal(size=(3, 3))*.05
        p, p0 = rng.normal(size=(2, 3))*.1
        lever = rng.normal(size=3)*.03
        feature = np.array([.4, -.2, 5.])+rng.normal(size=3)*.1
        calibration = np.array([458., 457., 367., 248., -.28, .074, .0002, .00002])
        if trial == 0:
            calibration[4:] = 0
        measured = np.array([400., 240.])
        Rbc = rotation(extrinsic_angle)

        def prediction(R, position, point):
            return camera(Rbc.T@(R.T@(point-position)-lever), calibration)
        values = np.r_[angle, p, first_angle, p0, extrinsic_angle,
                       lever, feature, calibration, measured]
        result = subprocess.run([probe, 'camera'], input=' '.join(format(x, '.17g')
                                for x in values)+'\n', text=True, capture_output=True, check=True)
        actual = np.fromstring(result.stdout, sep=' ')
        expected_r = measured-prediction(rotation(angle), p, feature)
        np.testing.assert_allclose(actual[:2], expected_r, atol=1e-10, rtol=1e-12)
        R0 = rotation(first_angle)
        epsilon = 1e-6
        Hpose = np.zeros((2, 6))
        Hfeature = np.zeros((2, 3))
        for axis in range(3):
            step = np.eye(3)[axis]*epsilon
            Hpose[:, axis] = (prediction(R0, p0+step, feature) -
                              prediction(R0, p0-step, feature))/(2*epsilon)
            Hpose[:, axis+3] = (prediction(R0@rotation(step), p0, feature) -
                                prediction(R0@rotation(-step), p0, feature))/(2*epsilon)
            Hfeature[:, axis] = (prediction(R0, p0, feature+step) -
                                 prediction(R0, p0, feature-step))/(2*epsilon)
        np.testing.assert_allclose(actual[2:14].reshape(2, 6), Hpose, atol=2e-7, rtol=2e-8)
        np.testing.assert_allclose(actual[14:].reshape(2, 3), Hfeature, atol=2e-7, rtol=2e-8)
        Nf = np.column_stack([np.eye(3), np.cross([0, 0, 1], feature)])
        gauge_residual = actual[2:14].reshape(2, 6)@gauge(R0, p0)+actual[14:].reshape(2, 3)@Nf
        np.testing.assert_allclose(gauge_residual, 0, atol=1e-10, rtol=0)
    print('Production camera: 24 distorted/off-axis cases; current residual, frozen pose/feature derivatives and gauge')


def main():
    rng = np.random.default_rng(513)
    probe = sys.argv[1]
    check_nominal(probe, rng)
    check_covariance(probe, rng)
    check_correction(probe, rng)
    check_camera(probe, rng)


if __name__ == '__main__':
    main()
