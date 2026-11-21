/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.ProcessNoise
import FormalGVINS.PropagationDifferential
import FormalGVINS.Injection
import FormalGVINS.JacobianFrames

/-! Frozen translation/yaw gauge for the selected FEJ linear model, without
absolute GNSS, map or heading observations. This is not a statistical claim. -/
namespace FormalGVINS.MSCKF
noncomputable section
open Matrix SO3 Quaternion LinearAlgebra

/-- A general linear combination of the three translation and one yaw columns. -/
def gaugeError (p v : Vector3 ℝ) (R : Matrix3 ℝ) (u translation : Vector3 ℝ) (yaw : ℝ) :
    Fin 5 → Vector3 ℝ :=
  ![translation+yaw • cross u p, yaw • cross u v, yaw • (Rᵀ*ᵥu), 0, 0]

theorem gauge_transport (R Rnext A1 A2 B1 B2 J : Matrix3 ℝ)
    (p v pnext vnext u translation : Vector3 ℝ) (yaw h gravity : ℝ)
    (hR : R*Rᵀ=1) :
    blocks (transitionBlocks R (Rnextᵀ*R) A1 A2 B1 B2 J
      (pnext-p-h • v-(h^2/2) • (gravity • u))
      (vnext-v-h • (gravity • u)) h) *ᵥ
      blockVector (gaugeError p v R u translation yaw) =
      blockVector (gaugeError pnext vnext Rnext u translation yaw) := by
  rw [blocks_mulVec]
  apply congrArg blockVector
  funext b
  have hu : R *ᵥ (Rᵀ*ᵥu) = u := by rw [Matrix.mulVec_mulVec, hR, Matrix.one_mulVec]
  have hv : (Rnextᵀ*R) *ᵥ (Rᵀ*ᵥu) = Rnextᵀ*ᵥu := by
    rw [← Matrix.mulVec_mulVec, hu]
  fin_cases b <;>
    simp [transitionBlocks, gaugeError, Fin.sum_univ_succ, Matrix.mulVec_add,
      Matrix.mulVec_smul, Matrix.smul_mulVec, Matrix.neg_mulVec, ← Matrix.mulVec_mulVec, hu, hv]
  all_goals
    ext i
    fin_cases i <;> simp [hat, cross, Matrix.vecHead, Matrix.vecTail, dotProduct, Fin.sum_univ_succ] <;> ring

def translationColumn (j : Fin 4) : Vector3 ℝ := fun i => if j.val=i.val then 1 else 0

def yawColumn (j : Fin 4) : ℝ := if j.val=3 then 1 else 0

def imuGauge (p v : Vector3 ℝ) (R : Matrix3 ℝ) (u : Vector3 ℝ) : Matrix (Fin 15) (Fin 4) ℝ :=
  fun i j => blockVector (gaugeError p v R u (translationColumn j) (yawColumn j)) i

theorem gauge_transport_matrix (R Rnext A1 A2 B1 B2 J : Matrix3 ℝ)
    (p v pnext vnext u : Vector3 ℝ) (h gravity : ℝ) (hR : R*Rᵀ=1) :
    blocks (transitionBlocks R (Rnextᵀ*R) A1 A2 B1 B2 J
      (pnext-p-h • v-(h^2/2) • (gravity • u)) (vnext-v-h • (gravity • u)) h) *
      imuGauge p v R u = imuGauge pnext vnext Rnext u := by
  ext i j
  exact congrFun (gauge_transport R Rnext A1 A2 B1 B2 J p v pnext vnext u
    (translationColumn j) (yawColumn j) h gravity hR) i

/-- Ordinary right-local held-input transition, with the bias-sensitive blocks
explicit rather than inferred from the gauge identity. -/
def ordinaryBlocks (R E A1 A2 B1 B2 J : Matrix3 ℝ) (a : Vector3 ℝ) (h : ℝ) :
    Fin 5 → Fin 5 → Matrix3 ℝ :=
  !![1,h • 1,-(R*hat (A2*ᵥa)),-R*A2,R*B2;
     0,1,-(R*hat (A1*ᵥa)),-R*A1,R*B1;
     0,0,Eᵀ,0,(-h) • J;
     0,0,0,1,0;
     0,0,0,0,1]

theorem transition_reduces_to_ordinary (q : Quaternion ℝ) (hq : HasUnitNorm q)
    (phi a : Vector3 ℝ) (A1 A2 B1 B2 : Matrix3 ℝ) (h : ℝ) :
    let R := rotationMatrix q
    let E := rotationMatrix (quaternionExp phi)
    transitionBlocks R ((R*E)ᵀ*R) A1 A2 B1 B2 (rightJacobian (-phi))
      (R*ᵥ(A2*ᵥa)) (R*ᵥ(A1*ᵥa)) h =
      ordinaryBlocks R E A1 A2 B1 B2 (rightJacobian phi) a h := by
  dsimp only
  have hD : (rotationMatrix q * rotationMatrix (quaternionExp phi))ᵀ * rotationMatrix q =
      (rotationMatrix (quaternionExp phi))ᵀ := by
    rw [transpose_mul, Matrix.mul_assoc, rotationMatrix_transpose_mul_of_unit hq, Matrix.mul_one]
  rw [hD]
  funext i j
  fin_cases i <;> fin_cases j <;>
    simp [transitionBlocks, ordinaryBlocks, -Matrix.mulVec_mulVec,
      hat_rotated_mul_unit q hq, rightJacobian_transported]

/-- Bind the FEJ endpoint differences to the exact mean, rather than only
supplying matching dp/dv blocks as independent arguments. -/
theorem transition_from_nominal_mean (q : Quaternion ℝ) (hq : HasUnitNorm q)
    (p v a g w : Vector3 ℝ) (B1 B2 : Matrix3 ℝ) (h : ℝ) :
    let R := rotationMatrix q
    transitionBlocks R ((R*rotationAt w h)ᵀ*R) (A1 w h) (A2 w h) B1 B2
      (rightJacobian (-(vectorScale h w)))
      (position p v a g R w h-p-h • v-(h^2/2) • g)
      (velocity v a g R w h-v-h • g) h =
      ordinaryBlocks R (rotationAt w h) (A1 w h) (A2 w h) B1 B2
        (rightJacobian (vectorScale h w)) a h := by
  dsimp only
  have hp : position p v a g (rotationMatrix q) w h-p-h • v-(h^2/2) • g =
      rotationMatrix q *ᵥ (A2 w h *ᵥ a) := by unfold position; abel
  have hv : velocity v a g (rotationMatrix q) w h-v-h • g =
      rotationMatrix q *ᵥ (A1 w h *ᵥ a) := by unfold velocity; abel
  rw [hp, hv, rotationAt_eq_exp]
  exact transition_reduces_to_ordinary q hq (vectorScale h w) a (A1 w h) (A2 w h) B1 B2 h

/-- Nonzero gravity direction and an invertible rotation give four independent
combinations. This does not rule out additional degeneracies of a measurement. -/
theorem gauge_independent (p v : Vector3 ℝ) (R : Matrix3 ℝ) (u translation : Vector3 ℝ)
    (yaw : ℝ) (hR : R*Rᵀ=1) (hu : u ≠ 0)
    (hz : gaugeError p v R u translation yaw = 0) : translation=0 ∧ yaw=0 := by
  have htheta := congrFun hz (2 : Fin 5)
  have ht : yaw • (Rᵀ*ᵥu)=0 := htheta
  have h := congrArg (fun x => R*ᵥx) ht
  have hyu : yaw • u=0 := by simpa [Matrix.mulVec_smul, Matrix.mulVec_mulVec, hR] using h
  have hy : yaw=0 := (smul_eq_zero.mp hyu).resolve_right hu
  have hp := congrFun hz (0 : Fin 5)
  exact ⟨by simpa [gaugeError, hy] using hp, hy⟩

/-- Observation cancellation occurs before the camera projection. J is the
SAME frozen distortion/projection/extrinsic factor in all three terms. -/
theorem visual_gauge_cancellation (q : Quaternion ℝ) (hq : HasUnitNorm q)
    (p feature u translation : Vector3 ℝ) (yaw : ℝ) (J : Matrix (Fin 2) (Fin 3) ℝ) :
    (-(J*(rotationMatrix q)ᵀ)) *ᵥ (translation+yaw • cross u p) +
      (J * hat ((rotationMatrix q)ᵀ*ᵥ(feature-p))) *ᵥ (yaw • ((rotationMatrix q)ᵀ*ᵥu)) +
      (J*(rotationMatrix q)ᵀ) *ᵥ (translation+yaw • cross u feature) = 0 := by
  have hh : hat ((rotationMatrix q)ᵀ*ᵥ(feature-p)) * (rotationMatrix q)ᵀ =
      (rotationMatrix q)ᵀ * hat (feature-p) := by
    simpa [rotationMatrix_conjugate] using hat_rotated_mul_unit (conjugate q) hq.conjugate (feature-p)
  have hk : hat ((rotationMatrix q)ᵀ*ᵥ(feature-p)) *ᵥ ((rotationMatrix q)ᵀ*ᵥu) =
      (rotationMatrix q)ᵀ*ᵥ (cross (feature-p) u) := by
    rw [Matrix.mulVec_mulVec, hh, ← Matrix.mulVec_mulVec, hat_mulVec]
  have hv : -(cross u p) + cross (feature-p) u + cross u feature = 0 := by
    ext i; fin_cases i <;> simp [cross] <;> ring
  simp only [Matrix.neg_mulVec, ← Matrix.mulVec_mulVec, Matrix.mulVec_smul, hk]
  have hc : -( (rotationMatrix q)ᵀ*ᵥ(translation+yaw • cross u p)) +
      yaw • ((rotationMatrix q)ᵀ*ᵥcross (feature-p) u) +
      (rotationMatrix q)ᵀ*ᵥ(translation+yaw • cross u feature) = 0 := by
    rw [← Matrix.mulVec_smul, ← Matrix.mulVec_neg, ← Matrix.mulVec_add, ← Matrix.mulVec_add]
    have he : -(translation+yaw • cross u p) + yaw • cross (feature-p) u +
        (translation+yaw • cross u feature) = yaw • (-(cross u p)+cross (feature-p) u+cross u feature) := by module
    rw [he, hv]; simp
  have he := congrArg (fun x => J*ᵥx) hc
  simpa [Matrix.mulVec_add, Matrix.mulVec_neg, Matrix.mulVec_smul] using he

def positionGauge (p u : Vector3 ℝ) : Matrix (Fin 3) (Fin 4) ℝ :=
  fun i j => (translationColumn j+yawColumn j • cross u p) i

def attitudeGauge (R : Matrix3 ℝ) (u : Vector3 ℝ) : Matrix (Fin 3) (Fin 4) ℝ :=
  fun i j => (yawColumn j • (Rᵀ*ᵥu)) i

theorem visual_gauge_matrix (q : Quaternion ℝ) (hq : HasUnitNorm q)
    (p feature u : Vector3 ℝ) (J : Matrix (Fin 2) (Fin 3) ℝ) :
    fromCols (-(J*(rotationMatrix q)ᵀ)) (J*hat ((rotationMatrix q)ᵀ*ᵥ(feature-p))) *
      fromRows (positionGauge p u) (attitudeGauge (rotationMatrix q) u) +
      (J*(rotationMatrix q)ᵀ)*positionGauge feature u = 0 := by
  rw [fromCols_mul_fromRows]
  ext i j
  exact congrFun (visual_gauge_cancellation q hq p feature u (translationColumn j) (yawColumn j) J) i

theorem frozen_gauge_after_injection {n : ℕ} (x : JointMean n) (d : Fin (dimension n) → ℝ)
    (u : Vector3 ℝ) :
    imuGauge (injectJoint x d).firstPose.p (injectJoint x d).firstVelocity
      (rotationMatrix (injectJoint x d).firstPose.q) u =
      imuGauge x.firstPose.p x.firstVelocity (rotationMatrix x.firstPose.q) u := rfl

/-- Any common row transform, including whitening, retains cancellation. -/
theorem row_transform_gauge {n f g m k : ℕ}
    (H : Matrix (Fin m) (Fin n) ℝ) (Hf : Matrix (Fin m) (Fin f) ℝ)
    (N : Matrix (Fin n) (Fin g) ℝ) (Nf : Matrix (Fin f) (Fin g) ℝ)
    (W : Matrix (Fin k) (Fin m) ℝ) (h : H*N+Hf*Nf=0) :
    (W*H)*N+(W*Hf)*Nf=0 := by
  rw [Matrix.mul_assoc, Matrix.mul_assoc, ← Matrix.mul_add, h, Matrix.mul_zero]

theorem feature_elimination_gauge {n f g m k : ℕ}
    (H : Matrix (Fin m) (Fin n) ℝ) (Hf : Matrix (Fin m) (Fin f) ℝ)
    (N : Matrix (Fin n) (Fin g) ℝ) (Nf : Matrix (Fin f) (Fin g) ℝ)
    (A : Matrix (Fin k) (Fin m) ℝ) (h : H*N+Hf*Nf=0) (ha : A*Hf=0) :
    (A*H)*N=0 := by
  have he := row_transform_gauge H Hf N Nf A h
  simpa [ha] using he

theorem correction_gauge {n m g : ℕ} (H : Matrix (Fin m) (Fin n) ℝ)
    (N : Matrix (Fin n) (Fin g) ℝ) (K : Matrix (Fin n) (Fin m) ℝ) (h : H*N=0) :
    (1-K*H)*N=N := by simp [Matrix.sub_mul, Matrix.mul_assoc, h]

theorem compress_gauge {n m k g : ℕ} (H : Matrix (Fin m) (Fin n) ℝ)
    (N : Matrix (Fin n) (Fin g) ℝ) (C : Matrix (Fin k) (Fin m) ℝ) (h : H*N=0) :
    (C*H)*N=0 := by rw [Matrix.mul_assoc, h, Matrix.mul_zero]

/-- Cloning and removal apply the SAME explicit row map to the gauge basis. -/
theorem augment_gauge (n : ℕ) (N : Matrix (Fin (dimension n)) (Fin 4) ℝ) :
    selection (augmentIndex n)*N = N.submatrix (augmentIndex n) id := selection_mul _ _

theorem remove_gauge {n : ℕ} (r : Fin (n+1)) (N : Matrix (Fin (dimension (n+1))) (Fin 4) ℝ) :
    selection (removeIndex r)*N = N.submatrix (removeIndex r) id := selection_mul _ _

/-- Clones retain their frozen gauge rows during propagation. -/
theorem joint_gauge_transport {n k g : ℕ} (F : Covariance n)
    (N0 N1 : Matrix (Fin n) (Fin g) ℝ) (Nc : Matrix (Fin k) (Fin g) ℝ)
    (h : F*N0=N1) : fromBlocks F 0 0 (1 : Covariance k) * fromRows N0 Nc = fromRows N1 Nc := by
  simp [fromBlocks_mul_fromRows, h]

theorem gauge_composition {n m k g : ℕ} (A : Matrix (Fin m) (Fin n) ℝ)
    (B : Matrix (Fin k) (Fin m) ℝ) (N0 : Matrix (Fin n) (Fin g) ℝ)
    (N1 : Matrix (Fin m) (Fin g) ℝ) (N2 : Matrix (Fin k) (Fin g) ℝ)
    (ha : A*N0=N1) (hb : B*N1=N2) : (B*A)*N0=N2 := by
  rw [Matrix.mul_assoc, ha, hb]

end
end FormalGVINS.MSCKF
