/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.RotationDifferential
import Mathlib.MeasureTheory.Integral.IntervalIntegral.FundThmCalculus
import Mathlib.Analysis.SpecialFunctions.Integrals.Basic
import Mathlib.Tactic

/-! Held-body-input propagation in exact real arithmetic. A1/A2 below are
integrals, distinct from the finite Gamma evaluation in production. -/
namespace FormalGVINS.MSCKF
noncomputable section
open Matrix SO3 Quaternion MeasureTheory

def interpolate (a b : ℝ) (u : ℝ) := (1-u)*a+u*b

theorem interpolate_endpoints (a b : ℝ) : interpolate a b 0=a ∧ interpolate a b 1=b := by
  simp [interpolate]

theorem interpolation_fraction {l r t : ℝ} (h : l < r) (hl : l ≤ t) (hr : t ≤ r) :
    0 ≤ (t-l)/(r-l) ∧ (t-l)/(r-l) ≤ 1 := by
  constructor
  · exact div_nonneg (sub_nonneg.mpr hl) (sub_nonneg.mpr h.le)
  · apply (div_le_one (sub_pos.mpr h)).mpr; linarith

theorem interpolate_range {a b u : ℝ} (hu : 0 ≤ u) (hu1 : u ≤ 1) (hab : a ≤ b) :
    a ≤ interpolate a b u ∧ interpolate a b u ≤ b := by
  dsimp [interpolate]; constructor <;> nlinarith

theorem endpoint_average_affine (a b u v : ℝ) :
    (interpolate a b u+interpolate a b v)/2 = interpolate a b ((u+v)/2) := by
  dsimp [interpolate]; ring

theorem average_bias (a b bias : ℝ) : (a-bias)/2+(b-bias)/2 = (a+b)/2-bias := by ring

def rotationAt (w : Vector3 ℝ) (t : ℝ) : Matrix3 ℝ :=
  rotationMatrix (constantRateIncrement w t)

theorem rotationAt_eq_exp (w : Vector3 ℝ) (t : ℝ) :
    rotationAt w t = rotationMatrix (quaternionExp (vectorScale t w)) := by
  rw [rotationAt, constantRateIncrement_eq_exp_signed]

theorem rotationAt_continuous (w : Vector3 ℝ) (i j : Fin 3) :
    Continuous (fun t => rotationAt w t i j) := by
  dsimp only [rotationAt, constantRateIncrement]
  split_ifs
  · exact continuous_const
  · fin_cases i <;> fin_cases j <;>
      simp only [rotationMatrix, fromScalarVector, vectorScale, Matrix.of_apply] <;> fun_prop

def A1 (w : Vector3 ℝ) (h : ℝ) : Matrix3 ℝ := fun i j => ∫ t in 0..h, rotationAt w t i j

def A2 (w : Vector3 ℝ) (h : ℝ) : Matrix3 ℝ :=
  fun i j => h * A1 w h i j - ∫ t in 0..h, t * rotationAt w t i j

theorem A2_weighted (w : Vector3 ℝ) (h : ℝ) (i j : Fin 3) :
    A2 w h i j = ∫ t in 0..h, (h-t)*rotationAt w t i j := by
  have hc := rotationAt_continuous w i j
  dsimp only [A2]
  simp only [sub_mul]
  have hc1 : IntervalIntegrable (fun t => h * rotationAt w t i j) volume 0 h := (continuous_const.mul hc).intervalIntegrable 0 h
  have hc2 : IntervalIntegrable (fun t => t * rotationAt w t i j) volume 0 h := (continuous_id.mul hc).intervalIntegrable 0 h
  rw [intervalIntegral.integral_sub hc1 hc2, intervalIntegral.integral_const_mul]
  rfl

theorem A1_derivative (w : Vector3 ℝ) (h : ℝ) (i j : Fin 3) :
    HasDerivAt (fun t => A1 w t i j) (rotationAt w h i j) h := by
  have hc := rotationAt_continuous w i j
  exact intervalIntegral.integral_hasDerivAt_right (hc.intervalIntegrable 0 h)
    hc.stronglyMeasurable.stronglyMeasurableAtFilter hc.continuousAt

theorem A2_derivative (w : Vector3 ℝ) (h : ℝ) (i j : Fin 3) :
    HasDerivAt (fun t => A2 w t i j) (A1 w h i j) h := by
  have hc : Continuous (fun t : ℝ => t * rotationAt w t i j) := continuous_id.mul (rotationAt_continuous w i j)
  have hi := intervalIntegral.integral_hasDerivAt_right (hc.intervalIntegrable 0 h)
    hc.stronglyMeasurable.stronglyMeasurableAtFilter hc.continuousAt
  exact (((hasDerivAt_id h).mul (A1_derivative w h i j)).sub hi).congr_deriv (by dsimp; ring)

theorem rotationAt_zero_rate (t : ℝ) : rotationAt 0 t = 1 := by
  simp [rotationAt, constantRateIncrement]

theorem A1_zero_rate (h : ℝ) : A1 0 h = h • (1 : Matrix3 ℝ) := by
  ext i j
  simp [A1, rotationAt_zero_rate]

theorem A2_zero_rate (h : ℝ) : A2 0 h = (h^2/2) • (1 : Matrix3 ℝ) := by
  ext i j
  rw [A2_weighted]
  simp only [rotationAt_zero_rate]
  rw [intervalIntegral.integral_mul_const]
  have he : (∫ t in (0:ℝ)..h, h-t) = h^2/2 := by
    rw [intervalIntegral.integral_sub (f := fun _ : ℝ => h) (g := fun t : ℝ => t)
      (continuous_const.intervalIntegrable 0 h) (continuous_id.intervalIntegrable 0 h)]
    simp
    ring
  rw [he]
  rfl

def velocity (v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) (h : ℝ) : Vector3 ℝ :=
  v + R *ᵥ (A1 w h *ᵥ a) + h • g

def position (p v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) (h : ℝ) : Vector3 ℝ :=
  p + h • v + R *ᵥ (A2 w h *ᵥ a) + (h^2/2) • g

theorem velocity_derivative (v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) (h : ℝ) :
    HasVectorDerivAt (velocity v a g R w) (R *ᵥ (rotationAt w h *ᵥ a) + g) h := by
  have ha := matrixVector_hasDerivAt (dv := (0 : Vector3 ℝ)) (A1_derivative w h) (fun i => hasDerivAt_const h (a i))
  have ha' : HasVectorDerivAt (fun t => R *ᵥ (A1 w t *ᵥ a))
      (R *ᵥ (rotationAt w h *ᵥ a)) h := by
    simpa using ha.const_matrix R
  intro i
  exact (((hasDerivAt_const h (v i)).add (ha' i)).add
    ((hasDerivAt_id h).mul_const (g i))).congr_deriv (by simp)

theorem position_derivative (p v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) (h : ℝ) :
    HasVectorDerivAt (position p v a g R w) (velocity v a g R w h) h := by
  have ha := matrixVector_hasDerivAt (dv := (0 : Vector3 ℝ)) (A2_derivative w h) (fun i => hasDerivAt_const h (a i))
  have ha' : HasVectorDerivAt (fun t => R *ᵥ (A2 w t *ᵥ a))
      (R *ᵥ (A1 w h *ᵥ a)) h := by simpa using ha.const_matrix R
  intro i
  exact ((((hasDerivAt_const h (p i)).add ((hasDerivAt_id h).mul_const (v i))).add (ha' i)).add
    ((((hasDerivAt_id h).pow 2).div_const 2).mul_const (g i))).congr_deriv (by
      simp [velocity])

theorem mean_zero_time (p v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) :
    position p v a g R w 0 = p ∧ velocity v a g R w 0 = v := by
  have h1 : A1 w 0 = 0 := by ext i j; simp [A1]
  have h2 : A2 w 0 = 0 := by ext i j; simp [A2]
  simp [position, velocity, h1, h2]

theorem zero_rate_position (p v a g : Vector3 ℝ) (R : Matrix3 ℝ) (h : ℝ) :
    position p v a g R 0 h = p + h • v + (h^2/2) • (R*ᵥa+g) := by
  simp [position, A2_zero_rate, Matrix.smul_mulVec, Matrix.mulVec_smul, smul_add, add_assoc]

theorem zero_rate_velocity (v a g : Vector3 ℝ) (R : Matrix3 ℝ) (h : ℝ) :
    velocity v a g R 0 h = v + h • (R*ᵥa+g) := by
  simp [velocity, A1_zero_rate, Matrix.smul_mulVec, Matrix.mulVec_smul, smul_add, add_assoc]

theorem zero_rate_split (p v a g : Vector3 ℝ) (R : Matrix3 ℝ) (h k : ℝ) :
    position (position p v a g R 0 h) (velocity v a g R 0 h) a g R 0 k =
      position p v a g R 0 (h+k) ∧
    velocity (velocity v a g R 0 h) a g R 0 k = velocity v a g R 0 (h+k) := by
  simp only [zero_rate_position, zero_rate_velocity]
  constructor <;> ext i <;> simp <;> ring

/-- Production recurrence, parameterized by order. This defines its exact-real
polynomial; it is not the infinite rotation integral. -/
def gammaTerm (shift : ℕ) (W : Matrix3 ℝ) : ℕ → Matrix3 ℝ
  | 0 => ((shift.factorial : ℝ)⁻¹) • 1
  | k+1 => ((k+shift+1 : ℕ) : ℝ)⁻¹ • (gammaTerm shift W k * W)

theorem gammaTerm_zero_matrix (shift k : ℕ) : gammaTerm shift 0 (k+1) = 0 := by
  simp [gammaTerm]

def gammaSum (shift order : ℕ) (W : Matrix3 ℝ) : Matrix3 ℝ := ∑ k ∈ Finset.range (order+1), gammaTerm shift W k

theorem gammaSum_zero (shift order : ℕ) :
    gammaSum shift order 0 = ((shift.factorial : ℝ)⁻¹) • (1 : Matrix3 ℝ) := by
  induction order with
  | zero => simp [gammaSum, gammaTerm]
  | succ k ih => rw [gammaSum, Finset.sum_range_succ]; simpa [gammaSum, gammaTerm_zero_matrix] using ih

theorem gamma_zero_matches_integrals (h : ℝ) :
    h • gammaSum 1 10 0 = A1 0 h ∧ h^2 • gammaSum 2 10 0 = A2 0 h := by
  simp [gammaSum_zero, A1_zero_rate, A2_zero_rate, Nat.factorial, smul_smul, div_eq_mul_inv]

theorem constantRate_add (w : Vector3 ℝ) (t u : ℝ) :
    constantRateIncrement w (t+u) = hamiltonMul (constantRateIncrement w t) (constantRateIncrement w u) := by
  by_cases hn : vectorNorm w = 0
  · simp [constantRateIncrement, hn]
  have hs := vectorNorm_squared w
  simp [vectorNormSquared, dotProduct, Fin.sum_univ_succ] at hs
  have ht : vectorNorm w * (t+u)/2 = vectorNorm w*t/2 + vectorNorm w*u/2 := by ring
  ext <;> simp [constantRateIncrement, hn, ht, Real.cos_add, Real.sin_add,
    hamiltonMul, fromScalarVector, vectorScale]
  · field_simp
    nlinarith [congrArg (fun x => x * (Real.sin (vectorNorm w*t/2)*Real.sin (vectorNorm w*u/2))) hs]
  all_goals ring
theorem rotationAt_add (w : Vector3 ℝ) (t u : ℝ) :
    rotationAt w (t+u) = rotationAt w t * rotationAt w u := by
  simp only [rotationAt, constantRate_add, rotationMatrix_hamiltonMul]

private theorem scalar_ode_unique (f g d : ℝ → ℝ)
    (hf : ∀ t, HasDerivAt f (d t) t) (hg : ∀ t, HasDerivAt g (d t) t)
    (hz : f 0 = g 0) : f = g := by
  have hd : ∀ t, HasDerivAt (fun x => f x-g x) 0 t := fun t => by
    simpa using! (hf t).sub (hg t)
  funext t
  have he := is_const_of_deriv_eq_zero (fun t => (hd t).differentiableAt)
    (fun t => (hd t).deriv) t 0
  linarith

theorem velocity_split (v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) (h k : ℝ) :
    velocity (velocity v a g R w h) a g (R*rotationAt w h) w k = velocity v a g R w (h+k) := by
  funext i
  have hd1 : ∀ t, HasDerivAt
      (fun t => velocity (velocity v a g R w h) a g (R*rotationAt w h) w t i)
      ((R *ᵥ (rotationAt w (h+t) *ᵥ a)+g) i) t := by
    intro t
    simpa only [rotationAt_add, Matrix.mulVec_mulVec, Matrix.mul_assoc] using
      (velocity_derivative (velocity v a g R w h) a g (R*rotationAt w h) w t) i
  have hd2 : ∀ t, HasDerivAt (fun t => velocity v a g R w (h+t) i)
      ((R *ᵥ (rotationAt w (h+t) *ᵥ a)+g) i) t := by
    intro t
    simpa using! ((velocity_derivative v a g R w (h+t)) i).comp t
      ((hasDerivAt_const t h).add (hasDerivAt_id t))
  have hz : velocity (velocity v a g R w h) a g (R*rotationAt w h) w 0 i =
      velocity v a g R w (h+0) i := by
    rw [(mean_zero_time 0 (velocity v a g R w h) a g (R*rotationAt w h) w).2]
    simp
  exact congrFun (scalar_ode_unique _ _ _ hd1 hd2 hz) k

theorem position_split (p v a g : Vector3 ℝ) (R : Matrix3 ℝ) (w : Vector3 ℝ) (h k : ℝ) :
    position (position p v a g R w h) (velocity v a g R w h) a g (R*rotationAt w h) w k =
      position p v a g R w (h+k) := by
  funext i
  have hd1 : ∀ t, HasDerivAt
      (fun t => position (position p v a g R w h) (velocity v a g R w h) a g (R*rotationAt w h) w t i)
      (velocity v a g R w (h+t) i) t := by
    intro t
    simpa only [velocity_split] using
      (position_derivative (position p v a g R w h) (velocity v a g R w h) a g (R*rotationAt w h) w t) i
  have hd2 : ∀ t, HasDerivAt (fun t => position p v a g R w (h+t) i)
      (velocity v a g R w (h+t) i) t := by
    intro t
    simpa using! ((position_derivative p v a g R w (h+t)) i).comp t
      ((hasDerivAt_const t h).add (hasDerivAt_id t))
  have hz : position (position p v a g R w h) (velocity v a g R w h) a g (R*rotationAt w h) w 0 i =
      position p v a g R w (h+0) i := by
    rw [(mean_zero_time (position p v a g R w h) (velocity v a g R w h) a g (R*rotationAt w h) w).1]
    simp
  exact congrFun (scalar_ode_unique _ _ _ hd1 hd2 hz) k

end
end FormalGVINS.MSCKF
