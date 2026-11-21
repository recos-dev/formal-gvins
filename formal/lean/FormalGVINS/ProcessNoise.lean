/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.Blocks
import FormalGVINS.Clones
import Mathlib.Data.Matrix.ColumnRowPartitioned
import Mathlib.Tactic

/-! Interval-equivalent noise, not an exact continuous white-noise integral.
The density can be correlated; no isotropy or positive definiteness is imposed. -/
namespace FormalGVINS.MSCKF
noncomputable section
open Matrix SO3 LinearAlgebra

def transitionBlocks (R D A1 A2 B1 B2 J : Matrix3 ℝ) (dp dv : Vector3 ℝ) (h : ℝ) :
    Fin 5 → Fin 5 → Matrix3 ℝ :=
  !![1, h • 1, -hat dp * R, -R*A2, R*B2;
     0, 1, -hat dv * R, -R*A1, R*B1;
     0, 0, D, 0, (-h) • (D*J);
     0, 0, 0, 1, 0;
     0, 0, 0, 0, 1]

def noiseBlocks (R D A1 A2 B1 B2 J : Matrix3 ℝ) (h : ℝ) :
    Fin 5 → Fin 4 → Matrix3 ℝ :=
  !![-R*A2, R*B2, 0, 0;
     -R*A1, R*B1, 0, 0;
     0, (-h) • (D*J), 0, 0;
     0, 0, h • 1, 0;
     0, 0, 0, h • 1]

/-- The top nine rows of Gd copy Psi's two bias columns in the same order. -/
theorem noise_matches_bias_columns (R D A1 A2 B1 B2 J : Matrix3 ℝ)
    (dp dv : Vector3 ℝ) (h : ℝ) (i : Fin 3) (j : Fin 2) :
    noiseBlocks R D A1 A2 B1 B2 J h ⟨i.val, by omega⟩ ⟨j.val, by omega⟩ =
      transitionBlocks R D A1 A2 B1 B2 J dp dv h ⟨i.val, by omega⟩ ⟨3+j.val, by omega⟩ := by
  fin_cases i <;> fin_cases j <;> rfl

def intervalCovariance {n k : ℕ} (G : Matrix (Fin n) (Fin k) ℝ)
    (Qc : Covariance k) (h : ℝ) : Covariance n := sandwich G (h⁻¹ • Qc)

theorem intervalCovariance_psd {n k : ℕ} (G : Matrix (Fin n) (Fin k) ℝ)
    (Qc : Covariance k) (hq : Qc.PosSemidef) {h : ℝ} (hh : 0 < h) :
    (intervalCovariance G Qc h).PosSemidef :=
  sandwich_posSemidef G (hq.smul (inv_nonneg.mpr hh.le))

theorem intervalCovariance_entries {n k : ℕ} (G : Matrix (Fin n) (Fin k) ℝ)
    (Qc : Covariance k) (h : ℝ) (i j : Fin n) :
    intervalCovariance G Qc h i j = ∑ a, ∑ b, G i b * (Qc b a / h) * G j a := by
  simp only [intervalCovariance, sandwich, Matrix.mul_apply, Matrix.transpose_apply,
    Matrix.smul_apply, smul_eq_mul, Finset.sum_mul]
  apply Finset.sum_congr rfl
  intro a _
  apply Finset.sum_congr rfl
  intro b _
  ring

theorem scaled_sandwich (R S : Matrix3 ℝ) (a b c : ℝ) :
    (a • R) * (c • S) * (b • R)ᵀ = (a*c*b) • (R*S*Rᵀ) := by
  simp [Matrix.transpose_smul, smul_smul, mul_comm, mul_left_comm, mul_assoc]

/-- These coefficients distinguish the adopted interval-average disturbance
from exact continuous white acceleration, whose pp coefficient is h^3/3. -/
theorem zero_rate_accelerometer_noise (R Sa : Matrix3 ℝ) {h : ℝ} (hh : h ≠ 0) :
    ((-h^2/2) • R) * (h⁻¹ • Sa) * ((-h^2/2) • R)ᵀ = (h^3/4) • (R*Sa*Rᵀ) ∧
    ((-h^2/2) • R) * (h⁻¹ • Sa) * ((-h) • R)ᵀ = (h^2/2) • (R*Sa*Rᵀ) ∧
    ((-h) • R) * (h⁻¹ • Sa) * ((-h) • R)ᵀ = h • (R*Sa*Rᵀ) := by
  simp only [scaled_sandwich]
  have hp : (-h^2/2)*h⁻¹*(-h^2/2)=h^3/4 := by field_simp; ring
  have hpv : (-h^2/2)*h⁻¹*(-h)=h^2/2 := by field_simp
  have hv : (-h)*h⁻¹*(-h)=h := by field_simp
  exact ⟨congrArg (fun x : ℝ => x • (R*Sa*Rᵀ)) hp,
    congrArg (fun x : ℝ => x • (R*Sa*Rᵀ)) hpv,
    congrArg (fun x : ℝ => x • (R*Sa*Rᵀ)) hv⟩

theorem bias_random_walk_scaling {n : ℕ} (Q : Covariance n) {h : ℝ} (hh : h ≠ 0) :
    (h • (1 : Covariance n)) * (h⁻¹ • Q) * (h • (1 : Covariance n))ᵀ = h • Q := by
  simp [Matrix.transpose_smul, smul_smul, hh]

/-- All four blocks of diag(Psi,I) P diag(Psi,I)^T, including old clones. -/
theorem joint_covariance_blocks {n k : ℕ} (F Pii Q : Covariance n)
    (Pic : Matrix (Fin n) (Fin k) ℝ) (Pci : Matrix (Fin k) (Fin n) ℝ) (Pcc : Covariance k) :
    (fromBlocks F 0 0 (1 : Covariance k)) * (fromBlocks Pii Pic Pci Pcc) *
      (fromBlocks F 0 0 (1 : Covariance k))ᵀ + fromBlocks Q 0 0 (0 : Covariance k) =
    fromBlocks (F*Pii*Fᵀ+Q) (F*Pic) (Pci*Fᵀ) Pcc := by
  simp [fromBlocks_transpose, fromBlocks_multiply, fromBlocks_add]

theorem joint_noise_psd {n k : ℕ} (Q : Covariance n) (hq : Q.PosSemidef) :
    (fromBlocks Q 0 0 (0 : Covariance k)).PosSemidef := by
  have he : fromBlocks Q 0 0 (0 : Covariance k) =
      (fromRows (1 : Covariance n) (0 : Matrix (Fin k) (Fin n) ℝ)) * Q *
        (fromRows (1 : Covariance n) (0 : Matrix (Fin k) (Fin n) ℝ))ᵀ := by
    simp only [fromRows_mul, transpose_fromRows, mul_fromCols,
      Matrix.one_mul, Matrix.zero_mul, transpose_one, transpose_zero, Matrix.mul_one, Matrix.mul_zero]
    ext i j; cases i <;> cases j <;> rfl
  rw [he]
  exact hq.mul_mul_conjTranspose_same
    (fromRows (1 : Covariance n) (0 : Matrix (Fin k) (Fin n) ℝ))

theorem joint_prediction_psd {n k : ℕ} (F : Covariance n)
    (P : Matrix (Fin n ⊕ Fin k) (Fin n ⊕ Fin k) ℝ) (Q : Covariance n)
    (hp : P.PosSemidef) (hq : Q.PosSemidef) :
    ((fromBlocks F 0 0 (1 : Covariance k)) * P *
      (fromBlocks F 0 0 (1 : Covariance k))ᵀ + fromBlocks Q 0 0 (0 : Covariance k)).PosSemidef := by
  have hf := hp.mul_mul_conjTranspose_same (fromBlocks F 0 0 (1 : Covariance k))
  exact hf.add (joint_noise_psd Q hq)

/-- Independent interval disturbances accumulate through the transition. -/
theorem segment_recurrence {n : ℕ} (F1 F2 P Q1 Q2 : Covariance n) :
    sandwich F2 (sandwich F1 P+Q1)+Q2 =
      sandwich (F2*F1) P + (sandwich F2 Q1+Q2) := by
  simp [sandwich, Matrix.mul_add, Matrix.add_mul, transpose_mul, Matrix.mul_assoc, add_assoc]

def scalarAccelerationNoise (h : ℝ) : Covariance 2 := !![h^3/4,h^2/2;h^2/2,h]

/-- A one-axis two-state counterexample: independent segments preserve total
velocity variance but give a different position variance than one interval. -/
theorem split_noise_not_unsplit :
    let F : Covariance 2 := !![1,1;0,1]
    let split := sandwich F (scalarAccelerationNoise 1) + scalarAccelerationNoise 1
    split 0 0 ≠ scalarAccelerationNoise 2 0 0 ∧ split 1 1 = scalarAccelerationNoise 2 1 1 := by
  norm_num [sandwich, scalarAccelerationNoise, Matrix.mul_apply, Matrix.vecMul, dotProduct, Fin.sum_univ_succ]

theorem joint_flatten_psd {n k : ℕ}
    (P : Matrix (Fin n ⊕ Fin k) (Fin n ⊕ Fin k) ℝ) (hp : P.PosSemidef) :
    (P.submatrix finSumFinEquiv.symm finSumFinEquiv.symm).PosSemidef := hp.submatrix _

end
end FormalGVINS.MSCKF
