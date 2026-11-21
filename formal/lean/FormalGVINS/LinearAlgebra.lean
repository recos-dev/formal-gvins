/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.

SPDX-License-Identifier: Apache-2.0
-/

import Mathlib.LinearAlgebra.Matrix.PosDef
import Mathlib.Tactic
import Mathlib.Logic.Function.Basic

/-! Mathematical layout and value semantics, without C++ storage or IEEE claims. -/

namespace FormalGVINS.LinearAlgebra

open Matrix
variable {α : Type*} {rows cols inner : ℕ}

def index (r : Fin rows) (c : Fin cols) : Fin (rows * cols) := finProdFinEquiv (r, c)

theorem index_val (r : Fin rows) (c : Fin cols) :
    (index r c).val = r.val * cols + c.val := by
  simp [index, finProdFinEquiv, Nat.mul_comm, Nat.add_comm]

def rowMajor (A : Matrix (Fin rows) (Fin cols) α) : Fin (rows * cols) → α :=
  fun i => A (finProdFinEquiv.symm i).1 (finProdFinEquiv.symm i).2

def fromRowMajor (a : Fin (rows * cols) → α) : Matrix (Fin rows) (Fin cols) α :=
  fun r c => a (index r c)

theorem rowMajor_index (A : Matrix (Fin rows) (Fin cols) α) (r : Fin rows) (c : Fin cols) :
    rowMajor A (index r c) = A r c := by simp [rowMajor, index]

theorem fromRowMajor_rowMajor (A : Matrix (Fin rows) (Fin cols) α) :
    fromRowMajor (rowMajor A) = A := by
  ext r c
  exact rowMajor_index A r c

theorem rowMajor_fromRowMajor (a : Fin (rows * cols) → α) :
    rowMajor (fromRowMajor a) = a := by
  funext i
  change a (finProdFinEquiv (finProdFinEquiv.symm i)) = a i
  rw [Equiv.apply_symm_apply]

def setCoefficient (A : Matrix (Fin rows) (Fin cols) α) (r : Fin rows) (c : Fin cols) (x : α) :=
  Function.update A r (Function.update (A r) c x)

theorem setCoefficient_at (A : Matrix (Fin rows) (Fin cols) α) (r : Fin rows) (c : Fin cols) (x : α) :
    setCoefficient A r c x r c = x := by simp [setCoefficient, Function.update]

theorem setCoefficient_frame (A : Matrix (Fin rows) (Fin cols) α) (r i : Fin rows)
    (c j : Fin cols) (x : α) (h : i ≠ r ∨ j ≠ c) :
    setCoefficient A r c x i j = A i j := by
  rcases h with hr | hc
  · simp [setCoefficient, Function.update, hr]
  · by_cases hr : i = r
    · subst i; simp [setCoefficient, Function.update, hc]
    · simp [setCoefficient, Function.update, hr]

def offsetIndex {size block : ℕ} (start : ℕ) (h : start + block ≤ size) (i : Fin block) : Fin size :=
  ⟨start + i.val, by omega⟩

def extractBlock {br bc : ℕ} (A : Matrix (Fin rows) (Fin cols) α)
    (sr sc : ℕ) (hr : sr + br ≤ rows) (hc : sc + bc ≤ cols) : Matrix (Fin br) (Fin bc) α :=
  fun r c => A (offsetIndex sr hr r) (offsetIndex sc hc c)

theorem extractBlock_entry {br bc : ℕ} (A : Matrix (Fin rows) (Fin cols) α)
    (sr sc : ℕ) (hr : sr + br ≤ rows) (hc : sc + bc ≤ cols) (r : Fin br) (c : Fin bc) :
    extractBlock A sr sc hr hc r c = A (offsetIndex sr hr r) (offsetIndex sc hc c) := rfl

def replaceBlock {br bc : ℕ} (A : Matrix (Fin rows) (Fin cols) α)
    (B : Matrix (Fin br) (Fin bc) α) (sr sc : ℕ) : Matrix (Fin rows) (Fin cols) α :=
  fun r c => if h : sr ≤ r.val ∧ r.val < sr + br ∧ sc ≤ c.val ∧ c.val < sc + bc
    then B ⟨r.val - sr, by omega⟩ ⟨c.val - sc, by omega⟩ else A r c

theorem replaceBlock_at {br bc : ℕ} (A : Matrix (Fin rows) (Fin cols) α)
    (B : Matrix (Fin br) (Fin bc) α) (sr sc : ℕ) (hr : sr + br ≤ rows) (hc : sc + bc ≤ cols)
    (r : Fin br) (c : Fin bc) :
    replaceBlock A B sr sc (offsetIndex sr hr r) (offsetIndex sc hc c) = B r c := by
  simp [replaceBlock, offsetIndex, r.isLt, c.isLt]

theorem replaceBlock_frame {br bc : ℕ} (A : Matrix (Fin rows) (Fin cols) α)
    (B : Matrix (Fin br) (Fin bc) α) (sr sc : ℕ) (r : Fin rows) (c : Fin cols)
    (h : ¬ (sr ≤ r.val ∧ r.val < sr + br ∧ sc ≤ c.val ∧ c.val < sc + bc)) :
    replaceBlock A B sr sc r c = A r c := by simp [replaceBlock, h]

theorem extract_replaceBlock {br bc : ℕ} (A : Matrix (Fin rows) (Fin cols) α)
    (B : Matrix (Fin br) (Fin bc) α) (sr sc : ℕ) (hr : sr + br ≤ rows) (hc : sc + bc ≤ cols) :
    extractBlock (replaceBlock A B sr sc) sr sc hr hc = B := by
  ext r c
  exact replaceBlock_at A B sr sc hr hc r c

theorem transpose_entry (A : Matrix (Fin rows) (Fin cols) α) (r : Fin rows) (c : Fin cols) :
    Aᵀ c r = A r c := rfl

theorem eager_entries (A B : Matrix (Fin rows) (Fin cols) ℝ) (s : ℝ) (r : Fin rows) (c : Fin cols) :
    (A + B) r c = A r c + B r c ∧ (A - B) r c = A r c - B r c ∧
    (-A) r c = -A r c ∧ (s • A) r c = s * A r c := ⟨rfl, rfl, rfl, rfl⟩

theorem product_entry (A : Matrix (Fin rows) (Fin inner) ℝ)
    (B : Matrix (Fin inner) (Fin cols) ℝ) (r : Fin rows) (c : Fin cols) :
    (A * B) r c = ∑ k, A r k * B k c := Matrix.mul_apply

theorem trace_entry_sum (A : Matrix (Fin rows) (Fin rows) ℝ) :
    Matrix.trace A = ∑ i, A i i := rfl

/-- Active extents are type-level proof parameters here, not dynamic allocation. -/
def activeMatrix {mr mc r c : ℕ} (hr : r ≤ mr) (hc : c ≤ mc)
    (storage : Matrix (Fin mr) (Fin mc) α) : Matrix (Fin r) (Fin c) α :=
  fun i j => storage (i.castLE hr) (j.castLE hc)

theorem inactive_storage_irrelevant {mr mc r c : ℕ} (hr : r ≤ mr) (hc : c ≤ mc)
    (a b : Matrix (Fin mr) (Fin mc) α)
    (h : ∀ i : Fin r, ∀ j : Fin c, a (i.castLE hr) (j.castLE hc) = b (i.castLE hr) (j.castLE hc)) :
    activeMatrix hr hc a = activeMatrix hr hc b := by ext i j; exact h i j

/-- The subtraction-form admission check cannot wrap in natural arithmetic. -/
theorem block_admission {start count capacity : ℕ} :
    (start ≤ capacity ∧ count ≤ capacity-start) ↔ start+count ≤ capacity := by omega

theorem active_product_entries {mr mk mc r k c : ℕ}
    (hr : r ≤ mr) (hk : k ≤ mk) (hc : c ≤ mc)
    (a : Matrix (Fin mr) (Fin mk) ℝ) (b : Matrix (Fin mk) (Fin mc) ℝ)
    (i : Fin r) (j : Fin c) :
    (activeMatrix hr hk a * activeMatrix hk hc b) i j =
      ∑ t : Fin k, a (i.castLE hr) (t.castLE hk) * b (t.castLE hk) (j.castLE hc) := rfl

end FormalGVINS.LinearAlgebra
