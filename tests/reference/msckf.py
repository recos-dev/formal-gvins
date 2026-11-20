"""Independent MSCKF mathematical oracle; never imports production code.

Matrix exponentials and complex-step differentiation replace the production
series/quadrature. All vectors use the project's (p,v,theta,ba,bg) order.
"""
import numpy as np
from scipy.linalg import block_diag, expm
from scipy.spatial.transform import Rotation


def hat(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation(v):
    return Rotation.from_rotvec(v).as_matrix()


def integrals(w, h):
    # The exponential of this augmented system contains both rotation integrals.
    system = np.zeros((9, 9), dtype=np.result_type(w, float))
    system[:3, :3] = hat(w)
    system[:3, 3:6] = np.eye(3)
    system[3:6, 6:9] = np.eye(3)
    result = expm(h * system)
    return result[:3, :3], result[:3, 3:6], result[:3, 6:9]


def coefficients(a, w, h):
    E, A1, A2 = integrals(w, h)
    B1, B2 = np.zeros((3, 3)), np.zeros((3, 3))
    epsilon = 1e-25
    for axis in range(3):
        perturbed = np.array(w, dtype=complex)
        perturbed[axis] += 1j * epsilon
        _, first, second = integrals(perturbed, h)
        B1[:, axis] = -(first @ a).imag / epsilon
        B2[:, axis] = -(second @ a).imag / epsilon
    return E, A1, A2, B1, B2


def mean(R, p, v, a, w, h, gravity):
    E, A1, A2 = integrals(w, h)
    return R @ E, p + v*h + R @ A2 @ a + gravity*h*h/2, v + R @ A1 @ a + gravity*h


def physical_transition(R, a, w, h):
    # Constant-coefficient error ODE in rotating coordinates, then back to world.
    system = np.zeros((15, 15))
    for offset in (0, 3, 6):
        system[offset:offset+3, offset:offset+3] = -hat(w)
    system[:3, 3:6] = np.eye(3)
    system[3:6, 6:9] = -hat(a)
    system[3:6, 9:12] = -np.eye(3)
    system[6:9, 12:15] = -np.eye(3)
    next_R = R @ rotation(w*h)
    start = block_diag(R.T, R.T, np.eye(9))
    end = block_diag(next_R, next_R, np.eye(9))
    return end @ expm(h*system) @ start


def transition(first, end, a, w, h, gravity):
    R0, p0, v0 = first
    Rn, pn, vn = end
    _, A1, A2, B1, B2 = coefficients(a, w, h)
    D = Rn.T @ R0
    # integral Exp(+omega*s) ds = h*Jr(-omega*h)
    Phi = np.eye(15)
    Phi[:3, 3:6] = h*np.eye(3)
    Phi[:3, 6:9] = -hat(pn-p0-v0*h-gravity*h*h/2) @ R0
    Phi[3:6, 6:9] = -hat(vn-v0-gravity*h) @ R0
    Phi[:3, 9:12], Phi[3:6, 9:12] = -R0@A2, -R0@A1
    Phi[:3, 12:15], Phi[3:6, 12:15] = R0@B2, R0@B1
    Phi[6:9, 6:9], Phi[6:9, 12:15] = D, -D@A1
    G = np.zeros((15, 12))
    G[:9, :6] = Phi[:9, 9:15]
    G[9:15, 6:12] = h*np.eye(6)
    return Phi, G


def gauge(R, p, v=None):
    result = np.zeros((6 if v is None else 15, 4))
    vertical = np.array([0., 0., 1.])
    result[:3, :3] = np.eye(3)
    result[:3, 3] = np.cross(vertical, p)
    offset = 3 if v is None else 6
    result[offset:offset+3, 3] = R.T @ vertical
    if v is not None:
        result[3:6, 3] = np.cross(vertical, v)
    return result


def camera(point, calibration):
    fx, fy, cx, cy, k1, k2, p1, p2 = calibration
    x, y = point[:2] / point[2]
    rho = x*x+y*y
    radial = 1+k1*rho+k2*rho*rho
    return np.array([fx*(x*radial+2*p1*x*y+p2*(rho+2*x*x))+cx,
                     fy*(y*radial+p1*(rho+2*y*y)+2*p2*x*y)+cy])
