/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.Solve
import Mathlib.Tactic

/-! Exact transformations used by column-pivoted Givens QR. Rank thresholds
are numerical decisions, not statements of exact rank or conditioning. -/
namespace FormalGVINS.LinearAlgebra
noncomputable section
open Matrix

/-- Apply one row rotation without constructing a dense orthogonal matrix. -/
def rotateRows {n m : ℕ} (p q : Fin n) (c s : ℝ) (A : Matrix (Fin n) (Fin m) ℝ) : Matrix (Fin n) (Fin m) ℝ :=
  fun i j => if i = p then c * A p j + s * A q j
    else if i = q then -s * A p j + c * A q j else A i j

theorem rotateRows_inverse {n m : ℕ} (p q : Fin n) (hpq : p ≠ q)
    {c s : ℝ} (hcs : c ^ 2 + s ^ 2 = 1) (A : Matrix (Fin n) (Fin m) ℝ) :
    rotateRows p q c (-s) (rotateRows p q c s A) = A := by
  ext i j
  by_cases hp : i = p
  · subst i; simp [rotateRows, Ne.symm hpq]; nlinarith [congrArg (· * A p j) hcs]
  · by_cases hq : i = q
    · subst i; simp [rotateRows, Ne.symm hpq]; nlinarith [congrArg (· * A q j) hcs]
    · simp [rotateRows, hp, hq]

theorem rotateRows_mul {n m k : ℕ} (p q : Fin n) (c s : ℝ)
    (A : Matrix (Fin n) (Fin m) ℝ) (B : Matrix (Fin m) (Fin k) ℝ) :
    rotateRows p q c s (A * B) = rotateRows p q c s A * B := by
  ext i j
  by_cases hp : i = p
  · subst i; simp [rotateRows, Matrix.mul_apply, add_mul, Finset.sum_add_distrib,
      ← Finset.mul_sum, mul_assoc]
  · by_cases hq : i = q
    · subst i; simp [rotateRows, hp, Matrix.mul_apply, add_mul, Finset.sum_add_distrib,
        ← Finset.mul_sum, mul_assoc]
    · simp [rotateRows, hp, hq, Matrix.mul_apply]

def givens {n : ℕ} (p q : Fin n) (c s : ℝ) : Covariance n :=
  rotateRows p q c s 1

theorem rotateRows_eq_mul {n m : ℕ} (p q : Fin n) (c s : ℝ)
    (A : Matrix (Fin n) (Fin m) ℝ) : rotateRows p q c s A = givens p q c s * A := by
  simpa [givens] using rotateRows_mul p q c s (1 : Covariance n) A

theorem givens_transpose {n : ℕ} (p q : Fin n) (hpq : p ≠ q) (c s : ℝ) :
    (givens p q c s)ᵀ = givens p q c (-s) := by
  ext i j
  by_cases hip : i = p <;> by_cases hiq : i = q <;>
    by_cases hjp : j = p <;> by_cases hjq : j = q <;>
    simp_all [givens, rotateRows, Matrix.one_apply, eq_comm]

theorem givens_orthogonal {n : ℕ} (p q : Fin n) (hpq : p ≠ q)
    {c s : ℝ} (hcs : c^2+s^2=1) :
    (givens p q c s)ᵀ * givens p q c s = 1 ∧
      givens p q c s * (givens p q c s)ᵀ = 1 := by
  have h := rotateRows_inverse p q hpq hcs (1 : Covariance n)
  have hneg : c^2+(-s)^2=1 := by simpa using hcs
  have h' := rotateRows_inverse p q hpq hneg (1 : Covariance n)
  rw [rotateRows_eq_mul] at h h'
  constructor
  · rw [givens_transpose p q hpq]; simpa only [givens] using h
  · rw [givens_transpose p q hpq]; simpa only [givens, neg_neg] using h'

theorem orthogonal_product {n : ℕ} (U V : Covariance n)
    (hu : Uᵀ*U=1) (hv : Vᵀ*V=1) : (V*U)ᵀ*(V*U)=1 := by
  calc
    _ = Uᵀ*(Vᵀ*V)*U := by simp [Matrix.transpose_mul, Matrix.mul_assoc]
    _ = 1 := by rw [hv]; simpa using hu

/-- Squared norm and pairwise inner products are preserved on the affected rows. -/
theorem givens_dot {c s : ℝ} (h : c ^ 2 + s ^ 2 = 1) (a b x y : ℝ) :
    (c*a+s*b)*(c*x+s*y)+(-s*a+c*b)*(-s*x+c*y) = a*x+b*y := by
  nlinarith [congrArg (· * (a*x+b*y)) h]

/-- The scaled coefficients used by the implementation normalize exactly. -/
theorem givens_scaled {a b scale : ℝ} (hs : 0 < scale) (hab : a ≠ 0 ∨ b ≠ 0) :
    let l := Real.sqrt ((a/scale)^2+(b/scale)^2)
    (a/scale/l)^2+(b/scale/l)^2 = 1 ∧
      -(b/scale/l)*a+(a/scale/l)*b = 0 ∧
      (a/scale/l)*a+(b/scale/l)*b = scale*l := by
  dsimp
  set l := Real.sqrt ((a/scale)^2+(b/scale)^2)
  have hx : 0 < (a/scale)^2+(b/scale)^2 := by
    rcases hab with ha | hb <;> positivity
  have hl : l ≠ 0 := ne_of_gt (Real.sqrt_pos.mpr hx)
  have he : l^2 = (a/scale)^2+(b/scale)^2 := Real.sq_sqrt hx.le
  constructor
  · calc
      _ = ((a/scale)^2+(b/scale)^2)/l^2 := by ring
      _ = 1 := by rw [← he, div_self (pow_ne_zero 2 hl)]
  constructor
  · ring
  · calc
      _ = scale * ((a/scale)^2+(b/scale)^2) / l := by
        field_simp [ne_of_gt hs]
      _ = scale*l := by rw [← he]; field_simp

/-- QR's invariant survives every row rotation, even if a later rank test stops. -/
theorem qr_row_invariant {n m k : ℕ} (p q : Fin n) (c s : ℝ)
    (A : Matrix (Fin n) (Fin m) ℝ) (B : Matrix (Fin n) (Fin k) ℝ)
    (U : Covariance n) (R : Matrix (Fin n) (Fin m) ℝ)
    (C : Matrix (Fin n) (Fin k) ℝ) (hR : R = U*A) (hC : C = U*B) :
    rotateRows p q c s R = rotateRows p q c s U * A ∧
    rotateRows p q c s C = rotateRows p q c s U * B := by
  subst R; subst C
  exact ⟨rotateRows_mul p q c s U A, rotateRows_mul p q c s U B⟩

def permuteColumns {n m : ℕ} (A : Matrix (Fin n) (Fin m) ℝ) (p : Equiv.Perm (Fin m)) : Matrix (Fin n) (Fin m) ℝ :=
  fun i j => A i (p j)

theorem pivot_invariant {n m : ℕ} (U : Covariance n) (A : Matrix (Fin n) (Fin m) ℝ)
    (p : Equiv.Perm (Fin m)) : permuteColumns (U*A) p = U * permuteColumns A p := by
  ext i j; rfl

theorem permutation_solution {n m : ℕ} (A : Matrix (Fin n) (Fin m) ℝ)
    (p : Equiv.Perm (Fin m)) (x : Fin m → ℝ) :
    permuteColumns A p *ᵥ (x ∘ p) = A *ᵥ x := by
  funext i
  exact Equiv.sum_comp p (fun j => A i j * x j)

/-- Whitening transforms the covariance to identity, when the Cholesky factor
and triangular inverse satisfy their equations. -/
theorem whitening_covariance {n : ℕ} (L W : Covariance n) (hw : W*L=1) :
    W*(L*Lᵀ)*Wᵀ = 1 := by
  calc
    _ = (W*L)*(W*L)ᵀ := by simp [transpose_mul, Matrix.mul_assoc]
    _ = 1 := by rw [hw]; simp

theorem nullspace_eliminates {n f d : ℕ} (N : Matrix (Fin d) (Fin n) ℝ)
    (Hf : Matrix (Fin n) (Fin f) ℝ) (H : Matrix (Fin n) (Fin d) ℝ)
    (x : Fin d → ℝ) (feature : Fin f → ℝ) (hN : N*Hf=0) :
    N *ᵥ (H*ᵥx + Hf*ᵥfeature) = (N*H)*ᵥx := by
  simp [Matrix.mulVec_add, Matrix.mulVec_mulVec, hN]

/-- The actual trailing rows are the nullspace map once elimination has
zeroed those rows of U*A. The premise is about entries, not N*A=0. -/
theorem trailing_rows_annihilate {n f k : ℕ} (U : Covariance n)
    (A : Matrix (Fin n) (Fin f) ℝ) (rows : Fin k → Fin n)
    (hz : ∀ i j, (U*A) (rows i) j = 0) :
    (U.submatrix rows id) * A = 0 := by
  ext i j
  exact hz i j

theorem trailing_rows_orthonormal {n k : ℕ} (U : Covariance n)
    (hu : U*Uᵀ=1) (rows : Fin k → Fin n) (hi : Function.Injective rows) :
    (U.submatrix rows id) * (U.submatrix rows id)ᵀ = 1 := by
  ext i j
  have he := congrArg (fun A : Covariance n => A (rows i) (rows j)) hu
  simpa [Matrix.mul_apply, Matrix.one_apply, hi.eq_iff] using he

theorem qr_normal_equation {n m : ℕ} (U : Covariance n)
    (A : Matrix (Fin n) (Fin m) ℝ) (b : Fin n → ℝ) (x : Fin m → ℝ)
    (hu : Uᵀ*U=1)
    (hr : (U*A)ᵀ *ᵥ ((U*A)*ᵥx-U*ᵥb)=0) : Aᵀ*ᵥ(A*ᵥx-b)=0 := by
  have he : (U*A)*ᵥx-U*ᵥb = U*ᵥ(A*ᵥx-b) := by
    simp [Matrix.mulVec_sub, Matrix.mulVec_mulVec]
  rw [he, Matrix.transpose_mul, Matrix.mulVec_mulVec, Matrix.mul_assoc, hu,
    Matrix.mul_one] at hr
  exact hr

/-- Projecting whitened noise onto orthonormal rows retains identity covariance. -/
theorem nullspace_noise {n k : ℕ} (N : Matrix (Fin k) (Fin n) ℝ)
    (hN : N*Nᵀ=1) : N*(1 : Covariance n)*Nᵀ = 1 := by simpa using hN

theorem triangular_leastSquares_normal {n m : ℕ}
    (R : Matrix (Fin n) (Fin m) ℝ) (C : Fin n → ℝ) (x : Fin m → ℝ)
    (hz : ∀ i j, m ≤ i.val → R i j = 0)
    (hsolve : ∀ i, i.val < m → (R*ᵥx) i = C i) :
    Rᵀ*ᵥ(R*ᵥx-C)=0 := by
  funext j
  change (∑ i, R i j * ((R*ᵥx) i-C i)) = 0
  apply Finset.sum_eq_zero
  intro i _
  by_cases h : i.val < m
  · simp [hsolve i h]
  · simp [hz i j (Nat.le_of_not_gt h)]

theorem whiten_from_forward_substitution {n : ℕ} (L W : Covariance n)
    (hLower : ∀ i k, i < k → L i k = 0) (hdiag : ∀ i, L i i ≠ 0)
    (hstep : ∀ i j, W i j = forwardSubstitutionStep L 1 W i j) :
    W*(L*Lᵀ)*Wᵀ=1 := by
  exact whitening_covariance L W (mul_eq_one_comm.mp
    (forwardSubstitution_equation hLower hdiag hstep))

/-- Normal equations give a global least-squares minimum; this is independent
of a particular QR implementation and applies to each RHS column. -/
theorem leastSquares_minimum {n m : ℕ} (A : Matrix (Fin n) (Fin m) ℝ)
    (b : Fin n → ℝ) (x : Fin m → ℝ) (hn : Aᵀ *ᵥ (A*ᵥx-b)=0)
    (y : Fin m → ℝ) :
    (A*ᵥx-b) ⬝ᵥ (A*ᵥx-b) ≤ (A*ᵥy-b) ⬝ᵥ (A*ᵥy-b) := by
  let r := A*ᵥx-b
  let d := A*ᵥ(y-x)
  have hz : d ⬝ᵥ r = 0 := by
    dsimp [d, r]
    rw [dotProduct_comm, ← dotProduct_transpose_mulVec, hn]
    simp
  have hd : 0 ≤ d ⬝ᵥ d := by
    exact Finset.sum_nonneg (fun i _ => mul_self_nonneg _)
  have hy : A*ᵥy-b = r+d := by dsimp [r,d]; rw [Matrix.mulVec_sub]; abel
  have hz' : r ⬝ᵥ d = 0 := by rw [dotProduct_comm, hz]
  rw [hy, dotProduct_add, add_dotProduct, add_dotProduct, hz, hz']
  dsimp [r] at *
  linarith

end
end FormalGVINS.LinearAlgebra
