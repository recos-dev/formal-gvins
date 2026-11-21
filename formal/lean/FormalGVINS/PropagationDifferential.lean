/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.ImuPropagation
import Mathlib.Analysis.Calculus.ParametricIntegral

namespace FormalGVINS.MSCKF
noncomputable section
open Matrix SO3 Quaternion

/-- Perturbing the input rotation vector produces a right-local change, then
acts on a body vector. This fixes the sign of gyroscope-bias sensitivity. -/
theorem rotatedVector_derivative (phi d a : Vector3 ℝ) :
    HasVectorDerivAt
      (fun t => rotationMatrix (quaternionExp (phi + vectorScale t d)) *ᵥ a)
      (-(rotationMatrix (quaternionExp phi) * hat a * rightJacobian phi) *ᵥ d) 0 := by
  let q := quaternionExp phi
  let relative := fun t => hamiltonMul (conjugate q) (quaternionExp (phi + vectorScale t d))
  have hzero : relative 0 = identity := by
    have hz : vectorScale 0 d = 0 := by ext i; simp [vectorScale]
    simp [relative, hz, q, conjugate_hamiltonMul_of_unit (quaternionExp_hasUnitNorm phi)]
  have hr := rotationMatrix_hasDerivAt_identity (rightJacobian phi *ᵥ d)
    hzero (resetAttitude_hasDerivAt phi d)
  have ha := matrixVector_hasDerivAt (dv := (0 : Vector3 ℝ)) hr
    (fun i => hasDerivAt_const 0 (a i))
  have he : ∀ t, rotationMatrix q *ᵥ (rotationMatrix (relative t) *ᵥ a) =
      rotationMatrix (quaternionExp (phi + vectorScale t d)) *ᵥ a := by
    intro t
    rw [Matrix.mulVec_mulVec, ← rotationMatrix_hamiltonMul]
    dsimp [relative]
    rw [← hamiltonMul_assoc, hamiltonMul_conjugate_of_unit (quaternionExp_hasUnitNorm phi),
      identity_hamiltonMul]
  have hd : rotationMatrix q *ᵥ (hat (rightJacobian phi *ᵥ d) *ᵥ a) =
      -(rotationMatrix q * hat a * rightJacobian phi) *ᵥ d := by
    rw [hat_mulVec, cross_antisymm, ← hat_mulVec]
    simp [Matrix.mulVec_neg, Matrix.neg_mulVec, Matrix.mulVec_mulVec, Matrix.mul_assoc]
  have h := ha.const_matrix (rotationMatrix q)
  simpa only [he, Matrix.mulVec_zero, add_zero, hd] using h

/-- Positive B1/B2 gyro-bias kernels follow from omega = measurement - bias. -/
theorem gyroBias_kernel_derivative (w a dbg : Vector3 ℝ) (time : ℝ) :
    HasVectorDerivAt
      (fun t => rotationMatrix (quaternionExp (vectorScale time (w - vectorScale t dbg))) *ᵥ a)
      ((time • (rotationAt w time * hat a * rightJacobian (vectorScale time w))) *ᵥ dbg) 0 := by
  have h := rotatedVector_derivative (vectorScale time w) (vectorScale (-time) dbg) a
  have he : ∀ t, vectorScale time (w-vectorScale t dbg) =
      vectorScale time w + vectorScale t (vectorScale (-time) dbg) := by
    intro t; ext i; simp [vectorScale]; ring
  simp_rw [he]
  convert! h using 1
  rw [rotationAt_eq_exp]
  ext i
  simp [Matrix.mulVec, dotProduct, Fin.sum_univ_succ, vectorScale]
  ring

theorem rightAttitude_action_derivative (R : Matrix3 ℝ) (a d : Vector3 ℝ) :
    HasVectorDerivAt
      (fun t => R *ᵥ (rotationMatrix (quaternionExp (vectorScale t d)) *ᵥ a))
      (-(R*hat a) *ᵥ d) 0 := by
  have h := (rotatedVector_derivative 0 d a).const_matrix R
  simpa using h

/-- The accelerometer-bias block is negative because held force subtracts bias. -/
theorem accelerometerBias_derivative (R A : Matrix3 ℝ) (a d : Vector3 ℝ) :
    HasVectorDerivAt (fun t => R *ᵥ (A *ᵥ (a-vectorScale t d))) (-(R*A) *ᵥ d) 0 := by
  have h : HasVectorDerivAt (fun t => a-vectorScale t d) (-d) 0 := by
    intro i
    simpa [vectorScale] using! (hasDerivAt_const 0 (a i)).sub ((hasDerivAt_id 0).mul_const (d i))
  simpa [Matrix.mulVec_neg, Matrix.neg_mulVec] using (h.const_matrix A).const_matrix R

/-- Change-of-frame identity used to compare FEJ's attitude-sensitive blocks
with the ordinary derivative. The norm factor is essential before imposing unit norm. -/
theorem hat_rotated_mul (q : Quaternion ℝ) (v : Vector3 ℝ) :
    hat (rotationMatrix q *ᵥ v) * rotationMatrix q =
      normSquared q • (rotationMatrix q * hat v) := by
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [hat, rotationMatrix, Matrix.mul_apply, Matrix.mulVec, dotProduct,
      Fin.sum_univ_succ, normSquared] <;> ring

theorem hat_rotated_mul_unit (q : Quaternion ℝ) (hq : HasUnitNorm q) (v : Vector3 ℝ) :
    hat (rotationMatrix q *ᵥ v) * rotationMatrix q = rotationMatrix q * hat v := by
  rw [hat_rotated_mul, hq, one_smul]

/-- Finite quadrature applied to the pointwise derivative has exactly the B1
sum used in C++. This is the derivative of the corresponding quadrature mean,
not a claim that quadrature equals the continuous integral. -/
theorem quadrature_gyroBias_derivative {n : ℕ} (w a dbg : Vector3 ℝ)
    (time weight : Fin n → ℝ) (i : Fin 3) :
    HasDerivAt
      (fun t => ∑ k, weight k *
        (rotationMatrix (quaternionExp (vectorScale (time k) (w-vectorScale t dbg))) *ᵥ a) i)
      (∑ k, weight k *
        (((time k) • (rotationAt w (time k) * hat a *
          rightJacobian (vectorScale (time k) w))) *ᵥ dbg) i) 0 := by
  exact HasDerivAt.fun_sum (u := Finset.univ) (fun k _ => ((gyroBias_kernel_derivative w a dbg (time k)) i).const_mul (weight k))

/-- Differentiation of the continuous B1/B2 target under explicit domination
premises. Weight 1 gives B1; weight h-time gives B2. Unlike quadrature, the
continuous interchange requires these analytic hypotheses. -/
theorem integrated_gyroBias_derivative (w a dbg : Vector3 ℝ) (weight : ℝ → ℝ)
    (i : Fin 3) (μ : MeasureTheory.Measure ℝ) (s : Set ℝ) (bound : ℝ → ℝ)
    (hs : s ∈ nhds (0 : ℝ))
    (hMeas : ∀ᶠ x in nhds (0 : ℝ), MeasureTheory.AEStronglyMeasurable
      (fun t => weight t * (rotationMatrix (quaternionExp (vectorScale t (w-vectorScale x dbg))) *ᵥ a) i) μ)
    (hInt : MeasureTheory.Integrable
      (fun t => weight t * (rotationMatrix (quaternionExp (vectorScale t w)) *ᵥ a) i) μ)
    (hDMeas : MeasureTheory.AEStronglyMeasurable
      (fun t => weight t * ((t • (rotationAt w t * hat a * rightJacobian (vectorScale t w))) *ᵥ dbg) i) μ)
    (hLip : ∀ᵐ t ∂μ, LipschitzOnWith (Real.nnabs (bound t))
      (fun x => weight t * (rotationMatrix (quaternionExp (vectorScale t (w-vectorScale x dbg))) *ᵥ a) i) s)
    (hBound : MeasureTheory.Integrable bound μ) :
    HasDerivAt
      (fun x => ∫ t, weight t * (rotationMatrix (quaternionExp (vectorScale t (w-vectorScale x dbg))) *ᵥ a) i ∂μ)
      (∫ t, weight t * ((t • (rotationAt w t * hat a * rightJacobian (vectorScale t w))) *ᵥ dbg) i ∂μ) 0 := by
  have hz : vectorScale 0 dbg = 0 := by ext j; simp [vectorScale]
  exact (hasDerivAt_integral_of_dominated_loc_of_lip hs hMeas
    (by simpa only [hz, sub_zero] using hInt) hDMeas hLip hBound
    (Filter.Eventually.of_forall (fun t => ((gyroBias_kernel_derivative w a dbg t) i).const_mul (weight t)))).2

end
end FormalGVINS.MSCKF
