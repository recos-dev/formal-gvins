# V-LOCALIZE: Subpixel feature localization

**ID:** `V-LOCALIZE`

**Status:** Draft

**Applies to:** The raw-image ORB frontend, independently in each camera.

**Parent:** [V-ORB](orb-frontend.md).

**Uses:** [V-OBSERVATION](observation-admission.md), [V-EXECUTION](frontend-execution.md).

## Basis

ORB descriptors establish appearance similarity, not subpixel measurement accuracy.
The selected development baseline refines each detected corner in its own image
before description, following the structure of [OpenVINS's grid detector][ov-grid].
[OpenCV cornerSubPix][cv-subpix] defines an iterative gradient-based corner method;
its void-returning API does not certify convergence for each point.
This is not cross-image optical flow or a rectified-stereo disparity search.

## Required behavior

For each candidate, localization shall use a bounded two-dimensional corner
refinement in the original image domain. The profile shall bind pyramid level,
patch/gradient support, interpolation, iteration cap, stopping tolerance,
maximum displacement and observable localization-quality checks. Published
coordinates and total-displacement limits shall use original-image pixels.
An iteration stopping tolerance may use octave pixels when the profile states
that unit and its scale conversion. It shall not be presented as a bound on
localization error in either coordinate system.

Admission shall require finite coordinates, valid image/mask support for the
entire computation, an accepted corner-quality measure and displacement within
the profile bound. The profile shall define the measure and its threshold;
gradient strength/conditioning or an explicit update-residual check are possible
bindings, not a requirement to reconstruct all library internals.
If the backend reports a failed point, reject it. If it exposes no per-point
termination reason, report that limitation rather than labeling the point
converged or inferring exhaustion from unchanged coordinates. Acceptance then
depends on the stated observable checks and qualified localization error.
No padding outside admitted support or fallback to an unqualified coarse point
shall produce an accepted observation. A bounded call is not a convergence proof.

The frontend shall compute the descriptor using the accepted location and its
declared scale/orientation recipe. It shall preserve pairing if descriptor
computation removes or reorders points. Any further location change shall require
consistent descriptor regeneration before association. Raw coordinates shall
remain the estimator measurement; normalized bearings are derived metadata.

Post-localization selection shall enforce V-ORB's count, spatial and duplicate
rules. Rejecting one candidate shall not move another observation, transfer its
identity, or modify the source image. Localization quality shall not automatically
be interpreted as a calibrated pixel covariance.

## Acceptance and verification

Use known subpixel corners with varied orientation, contrast and blur; measure
bias and error against the known positions. Exercise edges without corners,
flat/saturated patches, masks, image borders, pyramid scaling, excessive movement,
weak corners, iteration-limit behavior and descriptor reordering. Require bounded
work and no admitted output on failed observable checks. If termination status
is unavailable, test the capped method without claiming to detect every
nonconvergent point. Qualify each CPU/CUDA path
and its measurement-noise model separately; storing floating-point coordinates
or passing synthetic corners alone does not establish real-image accuracy.
Stratify localization-error/noise qualification by pyramid level and image region;
an octave's coordinates converted to raw pixels do not automatically have the
same uncertainty as base-level corners.

## Profile binding

The [frontend profile](../../profiles/orb-frontend.md#raw-image-qualification)
owns numerical bounds and qualification. The production localizer has explicit
trial bounds and synthetic-corner tests.
Real-camera localization-noise qualification and deployment acceptance remain open.

[cv-subpix]: https://docs.opencv.org/4.13.0/dd/d1a/group__imgproc__feature.html
[ov-grid]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/Grider_FAST.h
