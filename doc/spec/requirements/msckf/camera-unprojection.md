# M-UNPROJECT: Calibrated pixel-to-bearing conversion

**ID:** `M-UNPROJECT`

**Status:** Draft

**Applies to:** Fixed pinhole/radial-tangential calibration, for frontend geometry and triangulation.

**Parent:** [M-VISUAL](visual-geometry.md).

**Uses:** [V-FRAMES](../common/frames.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

The [camera formulation](../../architecture/msckf-formulation.md#visual-observation-and-jacobians)
defines `D_c`, the normalized-coordinate to raw-pixel map. Inverting coordinates
pointwise preserves the selected raw-image observation model; it does not require
resampling an image or treating bearings as independent measurements.

## Required behavior

For admitted raw pixel `z`, calibration and an explicit numerical domain, the
operation shall compute `xi=[x_n,y_n]` and the bearing `d=[x_n,y_n,1]` satisfying

$$
\|D_c(\xi)-z\|_\infty\le\epsilon_{\mathrm{inverse,px}}.
$$

The caller shall bind camera/image/calibration identity and a qualified region in
which the selected inverse is unique. Qualification shall address distortion-map
folding and inverse sensitivity; finite coefficients or a small round-trip
residual alone do not establish uniqueness. The operation shall check finite
inputs, valid intrinsics, normalized-domain membership and the numerical conditions
required by its solver. The image admission owner shall check raw pixel bounds. Unsupported models shall not be silently treated as pinhole.

The profile shall fix initialization, iteration/step limits, domain constraints,
conditioning threshold and forward residual tolerance. No unbounded iteration,
silent clamping to a valid pixel/radius, or returned last iterate on failure is
permitted. A zero-distortion path shall obey the same coordinate and acceptance
semantics. Failure shall preserve output and shall not yield a usable bearing.

Derived bearings shall retain their raw observation and calibration binding.
Raw pixels and their derived bearings shall not be fused as independent measurements.
Unit rays used for angular tests shall be explicitly normalized; `d` above is
not a unit vector. If pixel noise is transformed for geometric screening, use
the inverse-map Jacobian and declare that first-order approximation. It shall
not change the raw covariance used for the MSCKF residual.

## Acceptance and verification

Check zero-distortion limits, nonzero radial/tangential coefficients, off-axis and
boundary points, invalid intrinsics, folded/nonunique domains, ill-conditioned
inverses and bounded nonconvergence. Use independently generated normalized points
and a high-accuracy reference inverse; check both forward residual and bearing
error in the qualified domain. Coordinate/calibration pairing and failure
atomicity are separate obligations. Floating-point round trips alone do not
prove a uniform inverse error bound.

The [frontend profile](../../profiles/orb-frontend.md#camera-inverse) binds the
implemented Newton solver, including units and bounded failure behavior. Its
numerical residual tolerance is not a camera measurement-noise bound. The
[local VINS profile](../../profiles/local-vins.md#visual-constraints-and-updates)
must qualify its camera domain, solver settings and error limits before estimator
acceptance; using the inverse in a frontend does not discharge those obligations.
