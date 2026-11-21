/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.
SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.LinearAlgebra
import FormalGVINS.Covariance

/-! Exact joint coordinates and deterministic clone covariance.
No feature, calibration or gravity coordinates are appended to this state. -/
namespace FormalGVINS.MSCKF
noncomputable section
open Matrix LinearAlgebra

def dimension (n : ℕ) := 15 + 6*n

def cloneCoordinate {n : ℕ} (clone : Fin n) (axis : Fin 6) : Fin (dimension n) :=
  ⟨15+6*clone.val+axis.val, by dsimp [dimension]; omega⟩

theorem active_dimension_bound {n capacity : ℕ} (h : n ≤ capacity) :
    dimension n ≤ dimension capacity := by dsimp [dimension]; omega

theorem cloneCoordinate_injective {n : ℕ} (c d : Fin n) (a b : Fin 6)
    (h : cloneCoordinate c a = cloneCoordinate d b) : c = d ∧ a = b := by
  have he := congrArg Fin.val h
  dsimp [cloneCoordinate] at he
  constructor <;> apply Fin.ext <;> omega

/-- Local pose order is (position, right-local attitude), skipping velocity. -/
def poseIndex (i : Fin 6) : Fin 15 :=
  ⟨if i.val < 3 then i.val else i.val+3, by split_ifs <;> omega⟩

theorem poseIndex_entries : List.ofFn (fun i => (poseIndex i).val) = [0,1,2,6,7,8] := by decide

def augmentIndex (n : ℕ) (i : Fin (dimension (n+1))) : Fin (dimension n) :=
  if h : i.val < dimension n then ⟨i.val,h⟩
  else ⟨(poseIndex ⟨i.val-dimension n, by dsimp [dimension] at *; omega⟩).val,
    by have := (poseIndex ⟨i.val-dimension n, by dsimp [dimension] at *; omega⟩).isLt
       dsimp [dimension]; omega⟩

theorem augmentIndex_old (n : ℕ) (i : Fin (dimension n)) :
    augmentIndex n (i.castLE (by dsimp [dimension]; omega)) = i := by
  simp [augmentIndex, i.isLt]

theorem augmentIndex_new (n : ℕ) (a : Fin 6) :
    (augmentIndex n ⟨dimension n+a.val, by dsimp [dimension]; omega⟩).val = (poseIndex a).val := by
  simp [augmentIndex]

def removeIndex {n : ℕ} (removed : Fin (n+1)) (i : Fin (dimension n)) : Fin (dimension (n+1)) :=
  ⟨if i.val < 15+6*removed.val then i.val else i.val+6,
    by split_ifs <;> dsimp [dimension] at * <;> omega⟩

theorem removeIndex_injective {n : ℕ} (removed : Fin (n+1)) :
    Function.Injective (removeIndex removed) := by
  intro i j h
  have he := congrArg Fin.val h
  dsimp [removeIndex] at he
  apply Fin.ext
  split_ifs at he <;> omega

theorem removeIndex_preserves_imu {n : ℕ} (removed : Fin (n+1)) (i : Fin 15) :
    (removeIndex removed ⟨i.val, by dsimp [dimension]; omega⟩).val = i.val := by
  simp [removeIndex, show i.val < 15+6*removed.val by omega]

theorem removeIndex_skips_block {n : ℕ} (removed : Fin (n+1)) (i : Fin (dimension n)) :
    (removeIndex removed i).val < 15+6*removed.val ∨
      15+6*(removed.val+1) ≤ (removeIndex removed i).val := by
  dsimp [removeIndex]; split_ifs <;> omega

def selection {m n : ℕ} (f : Fin m → Fin n) : Matrix (Fin m) (Fin n) ℝ :=
  fun i j => if j=f i then 1 else 0

theorem selection_mul {m n k : ℕ} (f : Fin m → Fin n) (A : Matrix (Fin n) (Fin k) ℝ) :
    selection f * A = A.submatrix f id := by
  ext i j
  simp [selection, Matrix.mul_apply]

theorem selection_covariance {m n : ℕ} (f : Fin m → Fin n) (P : Covariance n) :
    selection f * P * (selection f)ᵀ = P.submatrix f f := by
  rw [selection_mul]
  ext i j
  simp [selection, Matrix.mul_apply]

def augmentCovariance (n : ℕ) (P : Covariance (dimension n)) :=
  P.submatrix (augmentIndex n) (augmentIndex n)

def removeCovariance {n : ℕ} (r : Fin (n+1)) (P : Covariance (dimension (n+1))) :=
  P.submatrix (removeIndex r) (removeIndex r)

theorem augment_psd (n : ℕ) (P : Covariance (dimension n)) (hp : P.PosSemidef) :
    (augmentCovariance n P).PosSemidef := hp.submatrix _

theorem remove_psd {n : ℕ} (r : Fin (n+1)) (P : Covariance (dimension (n+1)))
    (hp : P.PosSemidef) : (removeCovariance r P).PosSemidef := hp.submatrix _

theorem augment_old_block (n : ℕ) (P : Covariance (dimension n))
    (i j : Fin (dimension n)) :
    augmentCovariance n P (i.castLE (by dsimp [dimension]; omega))
      (j.castLE (by dsimp [dimension]; omega)) = P i j := by
  simp [augmentCovariance, augmentIndex_old]

theorem augment_remove_last (n : ℕ) (P : Covariance (dimension n)) :
    removeCovariance ⟨n, by omega⟩ (augmentCovariance n P) = P := by
  ext i j
  have hi : i.val < 15+6*n := i.isLt
  have hj : j.val < 15+6*n := j.isLt
  change P (augmentIndex n (removeIndex ⟨n, by omega⟩ i))
    (augmentIndex n (removeIndex ⟨n, by omega⟩ j)) = P i j
  simp [removeIndex, augmentIndex, dimension, hi, hj]
  rfl

/-- Both covariance axes use the same map, including all cross-covariances. -/
theorem augment_duplicate_row (n : ℕ) (P : Covariance (dimension n)) (a : Fin 6)
    (j : Fin (dimension (n+1))) :
    augmentCovariance n P ⟨dimension n+a.val, by dsimp [dimension]; omega⟩ j =
      P ⟨(poseIndex a).val, by have := (poseIndex a).isLt; dsimp [dimension]; omega⟩
        (augmentIndex n j) := by
  simp [augmentCovariance, augmentIndex]

/-- C++ copies the upper triangle before mirroring. This equals P on the exact
symmetric domain, but is not a PSD-repair operation for asymmetric inputs. -/
def canonicalUpper {n : ℕ} (P : Covariance n) : Covariance n :=
  fun i j => if i ≤ j then P i j else P j i

theorem canonicalUpper_of_symmetric {n : ℕ} (P : Covariance n) (hp : Pᵀ=P) :
    canonicalUpper P = P := by
  ext i j
  dsimp [canonicalUpper]
  split_ifs
  · rfl
  · exact congrArg (fun A : Covariance n => A i j) hp

/-- A clone duplicates current and frozen poses separately; no correction
can retroactively rewrite the value in this immutable mathematical snapshot. -/
structure Snapshot (Pose : Type*) where
  nominal : Pose
  first : Pose
  id : ℕ
  time : ℕ

def cloneSnapshot {Pose : Type*} (nominal first : Pose) (id time : ℕ) : Snapshot Pose :=
  ⟨nominal, first, id, time⟩

theorem cloneSnapshot_fields {Pose : Type*} (nominal first : Pose) (id time : ℕ) :
    (cloneSnapshot nominal first id time).nominal = nominal ∧
    (cloneSnapshot nominal first id time).first = first ∧
    (cloneSnapshot nominal first id time).id = id ∧
    (cloneSnapshot nominal first id time).time = time := ⟨rfl,rfl,rfl,rfl⟩

end
end FormalGVINS.MSCKF
