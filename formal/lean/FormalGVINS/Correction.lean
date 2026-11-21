/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.

SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.Solve

namespace FormalGVINS.LinearAlgebra
noncomputable section
open Matrix

def innovationCovariance {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m) : Covariance m :=
  H * (P * Hᵀ) + V

theorem innovationCovariance_posDef {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    (innovationCovariance P H V).PosDef := by
  simpa [innovationCovariance, sandwich, Matrix.mul_assoc] using
    Matrix.PosDef.posSemidef_add (sandwich_posSemidef H hP) hV

def kalmanGain {n m : ℕ} (P : Covariance n) (H : Matrix (Fin m) (Fin n) ℝ)
    (V : Covariance m) : Matrix (Fin n) (Fin m) ℝ :=
  rightSolve (P * Hᵀ) (innovationCovariance P H V)

theorem kalmanGain_equation {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    kalmanGain P H V * innovationCovariance P H V = P * Hᵀ :=
  rightSolve_equation _ (innovationCovariance_posDef H hP hV)

def josephCovariance {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ) : Covariance n :=
  sandwich (1 - K * H) P + sandwich K V

/-- Joseph PSD preservation does not require an optimal gain. -/
theorem josephCovariance_posSemidef {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (K : Matrix (Fin n) (Fin m) ℝ) (hP : P.PosSemidef) (hV : V.PosSemidef) :
    (josephCovariance P H V K).PosSemidef :=
  (sandwich_posSemidef _ hP).add (sandwich_posSemidef _ hV)

theorem josephCovariance_expansion {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ) :
    josephCovariance P H V K =
      P - K * H * P - P * Hᵀ * Kᵀ + K * innovationCovariance P H V * Kᵀ := by
  simp only [josephCovariance, sandwich, innovationCovariance, transpose_sub,
    transpose_one, transpose_mul, sub_mul, mul_sub, mul_one, one_mul,
    Matrix.mul_add, Matrix.add_mul, Matrix.mul_assoc]
  abel

/-- The simplified covariance equation follows from the right-solve equation.
It is an exact identity, not a recommendation to replace Joseph in C++. -/
theorem josephCovariance_eq_simplified {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ)
    (hK : K * innovationCovariance P H V = P * Hᵀ) :
    josephCovariance P H V K = (1 - K * H) * P := by
  rw [josephCovariance_expansion, hK, sub_mul, one_mul]
  abel

theorem kalmanGain_joseph_eq_simplified {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    josephCovariance P H V (kalmanGain P H V) = (1 - kalmanGain P H V * H) * P :=
  josephCovariance_eq_simplified P H V _ (kalmanGain_equation H hP hV)

/-- Information reduces covariance in the SAME pre-reset coordinates.
Comparing this directly with a covariance after nonlinear-coordinate reset
would compare different coordinate systems. -/
theorem kalmanGain_covariance_reduction {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    P - josephCovariance P H V (kalmanGain P H V) =
      sandwich (P * Hᵀ) (innovationCovariance P H V)⁻¹ := by
  rw [kalmanGain_joseph_eq_simplified H hP hV]
  simp [sandwich, kalmanGain, rightSolve, transpose_mul, posSemidef_symmetric hP,
    Matrix.sub_mul, Matrix.mul_assoc]

theorem kalmanGain_covariance_reduction_posSemidef {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    (P - josephCovariance P H V (kalmanGain P H V)).PosSemidef := by
  rw [kalmanGain_covariance_reduction H hP hV]
  exact sandwich_posSemidef _ (innovationCovariance_posDef H hP hV).inv.posSemidef

end

end FormalGVINS.LinearAlgebra
