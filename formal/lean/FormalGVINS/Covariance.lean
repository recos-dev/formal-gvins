/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.

SPDX-License-Identifier: Apache-2.0
-/

import Mathlib.LinearAlgebra.Matrix.PosDef
import Mathlib.Algebra.Order.Star.Real
import Mathlib.Tactic.Abel

/-! Exact real covariance sandwiches and right solves. PSD and invertibility
are explicit mathematical premises, not guarantees of C++ admission checks. -/

namespace FormalGVINS.LinearAlgebra

noncomputable section

open Matrix

abbrev Covariance (n : ℕ) := Matrix (Fin n) (Fin n) ℝ

def sandwich {m n : ℕ} (A : Matrix (Fin m) (Fin n) ℝ) (P : Covariance n) :
    Covariance m := A * P * Aᵀ

theorem sandwich_posSemidef {m n : ℕ} (A : Matrix (Fin m) (Fin n) ℝ)
    {P : Covariance n} (hP : P.PosSemidef) : (sandwich A P).PosSemidef := by
  simpa [sandwich] using hP.mul_mul_conjTranspose_same A

theorem posSemidef_symmetric {n : ℕ} {P : Covariance n} (hP : P.PosSemidef) :
    Pᵀ = P := by
  exact hP.isHermitian

theorem sandwich_symmetric {m n : ℕ} (A : Matrix (Fin m) (Fin n) ℝ)
    {P : Covariance n} (hP : Pᵀ = P) : (sandwich A P)ᵀ = sandwich A P := by
  simp [sandwich, Matrix.transpose_mul, hP, Matrix.mul_assoc]

/-- Mathematical characterization of the right solve, not an implementation
of matrix inversion. The C++ backend solves X*S=B without forming an inverse. -/
def rightSolve {n m : ℕ} (B : Matrix (Fin n) (Fin m) ℝ) (S : Covariance m) :=
  B * S⁻¹

theorem rightSolve_equation {n m : ℕ} (B : Matrix (Fin n) (Fin m) ℝ)
    {S : Covariance m} (hS : S.PosDef) : rightSolve B S * S = B := by
  exact Matrix.nonsing_inv_mul_cancel_right S B
    ((Matrix.isUnit_iff_isUnit_det S).mp hS.isUnit)

theorem rightSolve_unique {n m : ℕ} {B X : Matrix (Fin n) (Fin m) ℝ}
    {S : Covariance m} (hS : S.PosDef) (hX : X * S = B) : X = rightSolve B S := by
  have h := congrArg (fun M : Matrix (Fin n) (Fin m) ℝ => M * S⁻¹) hX
  simpa [rightSolve, Matrix.mul_assoc,
    Matrix.mul_nonsing_inv S ((Matrix.isUnit_iff_isUnit_det S).mp hS.isUnit)] using h

/-- The linalg wrapper can implement the right solve by transposing the
right-hand side and invoking an ordinary SPD left solve. -/
theorem rightSolve_transposed_equation {n m : ℕ} (B : Matrix (Fin n) (Fin m) ℝ)
    {S : Covariance m} (hS : S.PosDef) : S * (rightSolve B S)ᵀ = Bᵀ := by
  have h := congrArg Matrix.transpose (rightSolve_equation B hS)
  simpa [Matrix.transpose_mul, posSemidef_symmetric hS.posSemidef] using h


end
end FormalGVINS.LinearAlgebra
