/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.Correction
import FormalGVINS.Clones
import FormalGVINS.ResetDifferential
import Mathlib.Tactic

/-! Joint Hamilton/right Exp injection with frozen first estimates. The
selected FEJ reset is identity; the physical Log reset is a different map. -/
namespace FormalGVINS.MSCKF
noncomputable section
open SO3 Quaternion Matrix LinearAlgebra

structure Pose where
  p : Vector3 ℝ
  q : Quaternion ℝ

structure ImuMean where
  pose : Pose
  v : Vector3 ℝ
  ba : Vector3 ℝ
  bg : Vector3 ℝ

structure JointMean (n : ℕ) where
  imu : ImuMean
  firstPose : Pose
  firstVelocity : Vector3 ℝ
  clones : Fin n → Snapshot Pose

def injectPose (x : Pose) (dp dtheta : Vector3 ℝ) : Pose :=
  ⟨x.p+dp, hamiltonMul x.q (quaternionExp dtheta)⟩

def imuError {n : ℕ} (d : Fin (dimension n) → ℝ) (b : Fin 5) : Vector3 ℝ :=
  fun a => d ⟨3*b.val+a.val, by have := a.isLt; have := b.isLt; dsimp [dimension]; omega⟩

def injectJoint {n : ℕ} (x : JointMean n) (d : Fin (dimension n) → ℝ) : JointMean n :=
  { x with
    imu := ⟨injectPose x.imu.pose (imuError d 0) (imuError d 2),
      x.imu.v+imuError d 1, x.imu.ba+imuError d 3, x.imu.bg+imuError d 4⟩
    clones := fun c => { x.clones c with
      nominal := injectPose (x.clones c).nominal
        (fun a => d (cloneCoordinate c ⟨a.val, by omega⟩))
        (fun a => d (cloneCoordinate c ⟨3+a.val, by omega⟩)) } }

theorem injectPose_unit (x : Pose) (hq : HasUnitNorm x.q) (dp dt : Vector3 ℝ) :
    HasUnitNorm (injectPose x dp dt).q := hq.hamiltonMul (quaternionExp_hasUnitNorm dt)

theorem injectJoint_unit {n : ℕ} (x : JointMean n) (d : Fin (dimension n) → ℝ)
    (hi : HasUnitNorm x.imu.pose.q) (hc : ∀ c, HasUnitNorm (x.clones c).nominal.q) :
    HasUnitNorm (injectJoint x d).imu.pose.q ∧
      ∀ c, HasUnitNorm ((injectJoint x d).clones c).nominal.q :=
  ⟨injectPose_unit _ hi _ _, fun c => injectPose_unit _ (hc c) _ _⟩

theorem injectJoint_frozen {n : ℕ} (x : JointMean n) (d : Fin (dimension n) → ℝ) :
    (injectJoint x d).firstPose = x.firstPose ∧
    (injectJoint x d).firstVelocity = x.firstVelocity ∧
    ∀ c, ((injectJoint x d).clones c).first = (x.clones c).first ∧
      ((injectJoint x d).clones c).id = (x.clones c).id ∧
      ((injectJoint x d).clones c).time = (x.clones c).time :=
  ⟨rfl,rfl,fun _ => ⟨rfl,rfl,rfl⟩⟩

theorem injectPose_zero (x : Pose) : injectPose x 0 0 = x := by
  cases x
  simp [injectPose]

theorem injectJoint_zero {n : ℕ} (x : JointMean n) : injectJoint x 0 = x := by
  have he : ∀ b : Fin 5, imuError (0 : Fin (dimension n) → ℝ) b = 0 := by intro b; rfl
  cases x with
  | mk imu firstPose firstVelocity clones =>
    cases imu
    simp [injectJoint, he, injectPose_zero]
    funext c
    change { clones c with nominal := injectPose (clones c).nominal 0 0 } = clones c
    rw [injectPose_zero]

theorem normalized_unit_unchanged (q : Quaternion ℝ) (hq : HasUnitNorm q) : Quaternion.normalize q = q := by
  have hn : normSquared q = 1 := hq
  ext <;> simp [Quaternion.normalize, hn, scale]

/-- Identity reset retains the entire Joseph covariance, not only its blocks. -/
def identityReset {n : ℕ} (P : Covariance n) : Covariance n := sandwich 1 P

theorem identityReset_eq {n : ℕ} (P : Covariance n) : identityReset P = P := by
  simp [identityReset, sandwich]

theorem corrected_covariance_psd {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m) (K : Matrix (Fin n) (Fin m) ℝ)
    (hp : P.PosSemidef) (hv : V.PosSemidef) :
    (identityReset (josephCovariance P H V K)).PosSemidef := by
  rw [identityReset_eq]
  exact josephCovariance_posSemidef H K hp hv

/-- A single-step perturbation bound for any submultiplicative norm, such as
Euclidean induced operator norm. E and Et denote G-I and its transpose/adjoint.
A correction-angle bound on their norms must be supplied separately. -/
theorem reset_covariance_bound {A : Type*} [NormedRing A] (P E Et : A)
    {epsilon : ℝ} (he : 0 ≤ epsilon) (hE : ‖E‖ ≤ epsilon) (hEt : ‖Et‖ ≤ epsilon) :
    ‖(1+E)*P*(1+Et)-P‖ ≤ (2*epsilon+epsilon^2)*‖P‖ := by
  have hform : (1+E)*P*(1+Et)-P = E*P + P*Et + E*P*Et := by noncomm_ring
  rw [hform]
  have h1 : ‖E*P‖ ≤ epsilon*‖P‖ := (norm_mul_le E P).trans (mul_le_mul_of_nonneg_right hE (norm_nonneg P))
  have h2 : ‖P*Et‖ ≤ ‖P‖*epsilon := (norm_mul_le P Et).trans (mul_le_mul_of_nonneg_left hEt (norm_nonneg P))
  have h3 : ‖E*P*Et‖ ≤ epsilon*‖P‖*epsilon := (norm_mul_le (E*P) Et).trans
    (mul_le_mul h1 hEt (norm_nonneg Et) (mul_nonneg he (norm_nonneg P)))
  calc
    _ ≤ (‖E*P‖+‖P*Et‖)+‖E*P*Et‖ := (norm_add_le _ _).trans
      (add_le_add (norm_add_le _ _) le_rfl)
    _ ≤ (2*epsilon+epsilon^2)*‖P‖ := by nlinarith

end
end FormalGVINS.MSCKF
