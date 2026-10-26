"""
observer.py
-----------
Generalized-momentum observer for the external joint torque of the SLAVE
(De Luca & Mattone, ICRA 2003; Haddadin et al., IEEE T-RO 2017).

Why the static residual τ̂ = G + C q̇ − τ_motor (paper Eq. 2) is not enough
on a position-controlled arm: during motion the Dynamixel position loop
produces large torques to accelerate the links (M q̈). With q̈ neglected,
those tracking torques appear as "contact" — in simulation the shoulder
estimate had ±12 N·m RMS in FREE motion, which is the rough, random force
the operator felt. Adding M q̈ needs a double-differentiated encoder signal.

The momentum observer avoids q̈. With p = M(q) q̇ and
    ṗ = τ_motor + τ_ext + C(q, q̇)ᵀ q̇ − G(q),
the residual
    r[k] = K_O ( p[k] − p[0] − Σ_{i<k} (τ_motor + Cᵀ q̇ − G + r)[i] Δt )
obeys ṙ = K_O (τ_ext − r): a first-order low-pass of the true external
torque with bandwidth K_O (rad/s), independent of the motion.

On the MASTER (current mode) τ_motor must be the COMMANDED torque; that
variant is left for future work (novelty item 1).
"""

import numpy as np

from .dynamics import ArmModel


class MomentumObserver:
    def __init__(self, model: ArmModel, dt: float, gain):
        self.model = model
        self.dt = dt
        self.K = np.asarray(gain, dtype=float) * np.ones(model.n)
        self._integral = None
        self._p0 = None
        self.r = np.zeros(model.n)

    def reset(self):
        self._integral = None
        self.r[:] = 0.0

    def update(self, q: np.ndarray, dq: np.ndarray, tau_motor: np.ndarray) -> np.ndarray:
        p = self.model.mass(q) @ dq
        if self._integral is None:
            self._p0 = p.copy()
            self._integral = np.zeros_like(p)
            return self.r.copy()
        beta = self.model.gravity(q) - self.model.coriolis_matrix(q, dq).T @ dq
        self._integral += (tau_motor - beta + self.r) * self.dt
        self.r = self.K * (p - self._p0 - self._integral)
        return self.r.copy()
