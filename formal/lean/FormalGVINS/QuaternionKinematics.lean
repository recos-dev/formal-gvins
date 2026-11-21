/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.

SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.RightJacobian

/-! Exact Hamilton body-rate kinematics, independent of a translation model. -/

namespace FormalGVINS.SO3

noncomputable section
open Quaternion Matrix

/-- Constant body-rate solution, written without `abs dt` so derivatives are
defined also at dt=0. For nonnegative time this equals Exp(dt*omega). -/
def constantRateIncrement (omega : Vector3 ℝ) (dt : ℝ) : Quaternion ℝ :=
  let w := vectorNorm omega
  if w = 0 then identity else
    fromScalarVector (Real.cos (w * dt / 2))
      (vectorScale (Real.sin (w * dt / 2) / w) omega)

theorem constantRateIncrement_eq_exp (omega : Vector3 ℝ) {dt : ℝ} (ht : 0 ≤ dt) :
    constantRateIncrement omega dt = quaternionExp (vectorScale dt omega) := by
  by_cases hw : vectorNorm omega = 0
  · have ho := (vectorNorm_eq_zero_iff omega).mp hw
    subst omega
    simp [constantRateIncrement]
  · by_cases hz : dt = 0
    · subst dt
      have hs : vectorScale 0 omega = 0 := by funext i; simp [vectorScale]
      simp [constantRateIncrement, hw, hs, fromScalarVector, identity]
    · have hn : vectorNorm (vectorScale dt omega) ≠ 0 := by
        rw [vectorNorm_vectorScale_of_nonnegative ht]
        exact mul_ne_zero hz hw
      rw [quaternionExp_of_vectorNorm_ne_zero hn]
      ext <;> simp [constantRateIncrement, hw, fromScalarVector, vectorScale,
        vectorNorm_vectorScale_of_nonnegative ht, mul_comm dt (vectorNorm omega)]
      all_goals field_simp

/-- Componentwise quaternion derivative, using four stored coefficients. -/
def HasQuaternionDerivAt (f : ℝ → Quaternion ℝ) (d : Quaternion ℝ) (t : ℝ) : Prop :=
  HasDerivAt (fun x => (f x).q0) d.q0 t ∧ HasDerivAt (fun x => (f x).q1) d.q1 t ∧
    HasDerivAt (fun x => (f x).q2) d.q2 t ∧ HasDerivAt (fun x => (f x).q3) d.q3 t

/-- The closed form satisfies qdot = (q * [0,omega])/2, not left multiplication. -/
theorem constantRateIncrement_kinematics (omega : Vector3 ℝ) (t : ℝ) :
    HasQuaternionDerivAt (constantRateIncrement omega)
      (scale (1 / 2) (hamiltonMul (constantRateIncrement omega t) (fromScalarVector 0 omega))) t := by
  by_cases hw : vectorNorm omega = 0
  · have ho := (vectorNorm_eq_zero_iff omega).mp hw
    subst omega
    simp only [HasQuaternionDerivAt, constantRateIncrement, vectorNorm_zero,
      hamiltonMul, fromScalarVector, identity, scale, Pi.zero_apply, mul_zero,
      zero_mul, sub_zero, add_zero]
    exact ⟨hasDerivAt_const _ _, hasDerivAt_const _ _, hasDerivAt_const _ _, hasDerivAt_const _ _⟩
  · have hn := vectorNorm_squared omega
    have hn' : vectorNorm omega ^ 2 = omega 0 * omega 0 + omega 1 * omega 1 + omega 2 * omega 2 := by
      simpa [vectorNormSquared, dotProduct, Fin.sum_univ_succ, add_assoc] using hn
    have ha : HasDerivAt (fun x : ℝ => vectorNorm omega * x / 2) (vectorNorm omega / 2) t := by
      simpa only [mul_one, id_eq] using! ((hasDerivAt_id t).const_mul (vectorNorm omega)).div_const 2
    have hc := ha.cos
    have hs (i : Fin 3) := (ha.sin.div_const (vectorNorm omega)).mul_const (omega i)
    simp only [HasQuaternionDerivAt, constantRateIncrement, hw, if_false, fromScalarVector,
      vectorScale, hamiltonMul, scale]
    refine ⟨hc.congr_deriv ?_, (hs 0).congr_deriv ?_, (hs 1).congr_deriv ?_, (hs 2).congr_deriv ?_⟩
    all_goals field_simp
    · rw [hn']
      ring
    all_goals ring

/-- A fixed initial attitude is a linear map on quaternion coefficients. -/
theorem HasQuaternionDerivAt.hamilton_left {f : ℝ → Quaternion ℝ} {d : Quaternion ℝ} {t : ℝ}
    (h : HasQuaternionDerivAt f d t) (q : Quaternion ℝ) :
    HasQuaternionDerivAt (fun x => hamiltonMul q (f x)) (hamiltonMul q d) t := by
  rcases h with ⟨h0, h1, h2, h3⟩
  exact ⟨(((h0.const_mul q.q0).sub (h1.const_mul q.q1)).sub (h2.const_mul q.q2)).sub (h3.const_mul q.q3),
    (((h1.const_mul q.q0).add (h0.const_mul q.q1)).add (h3.const_mul q.q2)).sub (h2.const_mul q.q3),
    (((h2.const_mul q.q0).sub (h3.const_mul q.q1)).add (h0.const_mul q.q2)).add (h1.const_mul q.q3),
    (((h3.const_mul q.q0).add (h2.const_mul q.q1)).sub (h1.const_mul q.q2)).add (h0.const_mul q.q3)⟩

/-- For an arbitrary initial q, the Exp trajectory solves constant body-rate
kinematics. Combine with constantRateIncrement_eq_exp for the prediction map. -/
theorem constantRate_attitude_kinematics (q : Quaternion ℝ) (omega : Vector3 ℝ) (t : ℝ) :
    HasQuaternionDerivAt (fun x => hamiltonMul q (constantRateIncrement omega x))
      (scale (1 / 2) (hamiltonMul (hamiltonMul q (constantRateIncrement omega t))
        (fromScalarVector 0 omega))) t := by
  have h := (constantRateIncrement_kinematics omega t).hamilton_left q
  have heq : hamiltonMul q (scale (1 / 2)
      (hamiltonMul (constantRateIncrement omega t) (fromScalarVector 0 omega))) =
      scale (1 / 2) (hamiltonMul (hamiltonMul q (constantRateIncrement omega t))
        (fromScalarVector 0 omega)) := by
    rw [hamiltonMul_assoc]
    ext <;> simp [hamiltonMul, scale] <;> ring
  rw [heq] at h
  exact h

end
end FormalGVINS.SO3
