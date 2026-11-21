/-
formal-gvins is licensed under the Apache License, Version 2.0.
See LICENSE for usage and redistribution terms.

SPDX-License-Identifier: Apache-2.0
-/

import FormalGVINS.PropagationDifferential
namespace FormalGVINS.SO3
noncomputable section
open Matrix Quaternion

theorem HasQuaternionDerivAt.conjugated {f : ℝ → Quaternion ℝ} {d : Quaternion ℝ} {t : ℝ}
    (hf : HasQuaternionDerivAt f d t) :
    HasQuaternionDerivAt (fun x => conjugate (f x)) (conjugate d) t :=
  ⟨hf.1, hf.2.1.neg, hf.2.2.1.neg, hf.2.2.2.neg⟩

theorem rightJacobian_neg_frame (phi : Vector3 ℝ) :
    rightJacobian (-phi) = rotationMatrix (quaternionExp phi) * rightJacobian phi := by
  let q := quaternionExp phi
  have hq : HasUnitNorm q := quaternionExp_hasUnitNorm phi
  apply Matrix.ext_iff_mulVec.mpr
  intro v
  have h := (((resetAttitude_hasDerivAt phi (-v)).conjugated).hamilton_left q).hamilton_right (conjugate q)
  have he : ∀ t, hamiltonMul
      (hamiltonMul q (conjugate (hamiltonMul (conjugate q)
        (quaternionExp (phi + vectorScale t (-v)))))) (conjugate q) =
      hamiltonMul (conjugate (quaternionExp (-phi))) (quaternionExp (-phi + vectorScale t v)) := by
    intro t
    have hv : -(phi+vectorScale t (-v)) = -phi+vectorScale t v := by ext i; simp [vectorScale]; ring
    simp only [conjugate_hamiltonMul, conjugate_conjugate, hamiltonMul_assoc,
      hamiltonMul_conjugate_of_unit hq, hamiltonMul_identity, ← quaternionExp_neg, hv]
    simp [q]
  have hpure (u : Vector3 ℝ) :
      hamiltonMul q (hamiltonMul (fromScalarVector 0 u) (conjugate q)) =
        fromScalarVector 0 (rotationMatrix q *ᵥ u) := by
    simpa [conjugate_conjugate, inverseRotate, rotationMatrix_conjugate] using
      quaternionConjugation_pure (conjugate q) u
  have hd : hamiltonMul
      (hamiltonMul q (conjugate (fromScalarVector 0 (vectorScale (1/2) (rightJacobian phi *ᵥ (-v))))))
      (conjugate q) = fromScalarVector 0 (vectorScale (1/2) (rotationMatrix q *ᵥ (rightJacobian phi *ᵥ v))) := by
    have hv : conjugate (fromScalarVector 0 (vectorScale (1/2) (rightJacobian phi *ᵥ (-v)))) =
        fromScalarVector 0 (vectorScale (1/2) (rightJacobian phi *ᵥ v)) := by
      ext <;> simp [conjugate, fromScalarVector, vectorScale, Matrix.mulVec_neg]
    rw [hv, hamiltonMul_assoc, hpure]
    congr 1
    change rotationMatrix q *ᵥ ((1/2:ℝ) • (rightJacobian phi *ᵥ v)) = _
    exact Matrix.mulVec_smul _ _ _
  have hfinal : HasQuaternionDerivAt
      (fun t => hamiltonMul (conjugate (quaternionExp (-phi))) (quaternionExp (-phi+vectorScale t v)))
      (fromScalarVector 0 (vectorScale (1/2) (rotationMatrix q *ᵥ (rightJacobian phi *ᵥ v)))) 0 := by
    convert! h using 1
    · funext t; exact (he t).symm
    · exact hd.symm
  have h' := resetAttitude_hasDerivAt (-phi) v
  have h1 := h'.2.1.unique hfinal.2.1
  have h2 := h'.2.2.1.unique hfinal.2.2.1
  have h3 := h'.2.2.2.unique hfinal.2.2.2
  simp only [fromScalarVector, vectorScale] at h1 h2 h3
  rw [← Matrix.mulVec_mulVec]
  funext i
  fin_cases i <;> dsimp [q] at * <;> linarith

theorem rightJacobian_transported (phi : Vector3 ℝ) :
    (rotationMatrix (quaternionExp phi))ᵀ * rightJacobian (-phi) = rightJacobian phi := by
  rw [rightJacobian_neg_frame, ← Matrix.mul_assoc,
    rotationMatrix_transpose_mul_of_unit (quaternionExp_hasUnitNorm phi), Matrix.one_mul]
end
end FormalGVINS.SO3
