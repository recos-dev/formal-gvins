# M-CLONE: Pose cloning and covariance selection

**ID:** `M-CLONE`

**Status:** Draft

**Applies to:** Augmentation and retirement of IMU-pose clones.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [M-FEJ](consistency.md), [V-ASSOCIATION](../vision/track-association.md), [F-BOUNDED](../math/linalg/bounded-matrices.md).

## Basis

[Augmentation](../../architecture/msckf-formulation.md#clone-augmentation) and
[removal](../../architecture/msckf-formulation.md#clone-removal-and-execution-order)
operate on the same joint covariance and pose representation.

## Required behavior

After propagation to an admitted modeled image time, append a copy of the current
IMU pose with stable identity. Let Jc select current position/local attitude from
the old D-dimensional error. Augmentation shall target

$$
P_{aug}=\begin{bmatrix}P&PJ_c^\mathsf T\\J_cP&J_cPJ_c^\mathsf T\end{bmatrix}.
$$

Copy the matching first estimate and attitude-coordinate map into the clone.
Add no independent clone noise. Exact covariance may become singular; rejecting
it merely for not being SPD would violate this model. A synchronized stereo pair
may share a clone only under its reviewed acquisition/time model.

Removal shall retain nominal poses, identities and FEJ caches in the selected
order and target `P_keep=E*P*E^T`, where E selects the same retained error indices
on both axes. Do not use a covariance Schur complement or discard surviving
cross-blocks. Pending observations shall be consumed or explicitly retired under
the lifecycle policy before their required clone disappears.

Invalid identity/time/selection, excess capacity or numerical failure shall not
publish partial state/covariance/cache layout. Selection indices for clones shall
be unique even though the reusable matrix gather permits repeated indices.

## Acceptance and verification

Establish augmentation/selection correspondence and preservation of exact PSD.
Exercise first/full-capacity clone, middle-clone removal, all retained cross-blocks,
stereo identity, invalid/duplicate selections and failure preservation. Verify
retirement cannot rebind observations to reused storage. Numerical symmetry/PSD
acceptance and capacity/selection policies require identified profiles.
