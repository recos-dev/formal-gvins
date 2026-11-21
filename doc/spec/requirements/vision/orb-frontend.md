# V-ORB: ORB feature extraction and association

**ID:** `V-ORB`

**Status:** Draft

**Applies to:** The raw-image ORB frontend, with optional calibrated stereo; no SLAM map or relocalization implementation is implied.

**Uses:** [V-FRAMES](../common/frames.md), [V-TRACK](feature-tracks.md), [V-EXECUTION](frontend-execution.md), [M-UNPROJECT](../msckf/camera-unprojection.md).

## Basis

The selected baseline uses OpenCV ORB keypoints and binary descriptors. The
[ORB](https://docs.opencv.org/4.13.0/db/d95/classcv_1_1ORB.html) and
[matcher](https://docs.opencv.org/4.13.0/d3/da1/classcv_1_1BFMatcher.html)
references define library semantics; version 4.13.0 is also the current pinned
development dependency, not deployment qualification. Descriptor tracking with a ratio test and
geometric rejection is also demonstrated by
[OpenVINS TrackDescriptor](https://docs.openvins.com/classov__core_1_1TrackDescriptor.html).
This requirement does not import its source, thresholds or quaternion convention.
The [development profile](../../profiles/orb-frontend.md) binds the implemented
raw-image pipeline and its current qualification limits.

## Required behavior

### Extraction and output

The frontend shall use ORB detection and description on an admitted image. Its
configuration shall identify the OpenCV implementation/version, preprocessing,
image dimensions, pyramid settings, detector settings and feature budget.
Invalid image/configuration input shall not create observations or alter existing
track associations.

For the local VINS profile, extraction shall operate on original distorted images
without a full-image undistortion remap. Published coordinates shall remain raw
pixels. Pointwise undistorted/normalized coordinates used for geometry shall be
derived with the same calibration and retain their source feature identity;
they shall not overwrite the raw observation or count as another measurement.

The first descriptor profile shall use `WTA_K = 2` and Hamming distance. Other
descriptor encodings shall not be compared as if they belonged to this profile.
Extraction shall preserve the association between each returned keypoint and its
descriptor, including after filtering or reordering. The tracking/publication
boundary shall admit only finite keypoint coordinates inside the image and
selection mask. Raw extractor candidates are not admitted observations.

Descriptor compatibility shall include encoding, pattern/version and preprocessing
recipe. A caller supplying descriptor blocks shall establish that binding; byte
shape alone is insufficient. The current adapter checks rows/type/width, not
descriptor provenance. Mixing incompatible recipes requires separate validation
and shall not be treated as equivalent execution of one descriptor profile.

The tracking boundary's published count shall not exceed the configured feature
capacity. Selection shall apply declared spatial quotas so a dense textured area
cannot consume the entire allocation; empty regions need not produce artificial
features. Feature selection shall also apply a configured minimum pixel separation across grid
boundaries and pyramid levels, with deterministic response/index tie-breaking.
Multiple detections of one image location shall not create independent estimator
observations merely because they have different scales. Candidate extraction and
[V-LOCALIZE](feature-localization.md) shall have their own work/count limits.
Final publication selection shall follow association: within the configured
spatial/count limits, retain verified continuations before new detections, then
prefer longer surviving tracks with stable-ID tie-breaking. Rank remaining new
features by the declared response/index policy. A new high-response corner shall
not replace an admissible continuing track merely because its response is larger.
Track length shall never waive matching or geometry checks.

Descriptor storage shall be owned or have a lifetime covering every consumer.
A retained keyframe shall own the descriptors and coordinate/calibration metadata needed
after the live image buffers are reused.

An image with no usable features shall produce an explicit empty observation set,
not fabricated correspondences or a reuse of the last image's measurements.
ORB responses and descriptor distances shall not be treated as calibrated
pixel-noise variance or match probabilities.

### Descriptor association

For each candidate temporal or stereo association, compare compatible descriptors
and apply a configured nearest-neighbor ratio threshold $0<\rho<1$.
Let $d_1\le d_2$ be the two nearest candidate distances. The ratio test shall
accept only when two candidates exist and $d_1<\rho d_2$. Ambiguous equal-distance
matches, including two zero distances, shall not pass this test.

Apply the test in both directions and retain only reciprocal best matches.
An absolute Hamming limit or a scale-compatibility restriction may be enabled by
the profile after qualifying false matches and track loss. If enabled, an integer
`d_max` in `[0,256]` shall use `d_1 <= d_max`. Neither is a universal additional
gate: OpenVINS's descriptor path does not use an absolute-distance threshold,
and ORB-SLAM3 uses thresholds specific to its matching task. Valid image support
is required independently. A candidate search region
shall retain enough alternatives to perform the ratio test; narrowing it to one
candidate shall not bypass ambiguity rejection. Greedy uniqueness is not a
substitute for reciprocal acceptance.

### Temporal geometry

Accepted associations shall additionally pass geometric verification under a
declared camera/motion model. A bounded robust-estimation procedure shall screen
outliers; failure to obtain an admissible model shall not be interpreted as all
matches being inliers. A descriptor match alone shall not create a confirmed
continuation of a track.

Use valid pointwise normalized coordinates from M-UNPROJECT. An admitted essential
model shall represent `E=[t_21]_x*R_21`, with nonzero translation direction and a
proper rotation. Normalize finite nonzero E to unit Frobenius norm before
testing its denominator, so that the numerical floor is not scale-dependent.
For bearings `d_1,d_2`, define

$$
s_E^2=\frac{(d_2^\mathsf T E d_1)^2}
{\|(E d_1)_{1:2}\|^2+\|(E^\mathsf T d_2)_{1:2}\|^2}.
$$

Admit an epipolar inlier only when `s_E <= tau_E`; an invalid or numerically
vanishing denominator rejects that candidate. This Sampson distance is in
normalized-image units, not raw pixels. Fit/score conventions, denominator floor,
sample degeneracy, minimum support and iteration bounds belong to the profile;
a threshold from a different image/noise domain shall not be copied as tau_E.

Static/pure-rotation tracking shall be qualified independently from finite-depth
triangulation. Insufficient triangulation parallax shall not itself retire an
otherwise verified temporal track. The profile shall declare its temporal model
and any alternative-model selection/failure policy. This contract does not mandate
a rotation-first, homography or IMU-aided fallback. If a rotation-only verifier
is selected, its support, angular-spread and angular-residual limits need separate
qualification; rotation agreement shall not be interpreted as known zero
translation or usable depth. Failure of every configured verification path shall
publish no confirmed continuations, never an accept-all fallback.

For the current supplementary rotation screen, selection requires the profile's
minimum count/support fraction. It may only remove E inliers, never rescue a
failed E fit. Its residual and rank checks shall be tested with known distractors
in each declared motion/scene regime.

Within a camera/frame pair, published associations shall be one-to-one. Conflicting
candidates shall be rejected or resolved by the declared selection policy before
track IDs are assigned. The frontend shall preserve identity according to
[V-TRACK](feature-tracks.md), not infer identity from descriptor equality.

### Estimator boundary

The estimator-facing observations shall contain identities, acquisition time,
coordinates and their calibration/noise-model binding. OpenCV containers and
descriptor-search state shall remain outside the numerical core. Cross-camera
association shall meet V-STEREO when enabled. Landmark retrieval, PnP and loop
constraints need separate contracts; local matching does not establish them.

## Decomposition

| Child | Allocated obligation |
|---|---|
| [V-LOCALIZE](feature-localization.md) | Bounded raw-image subpixel localization and descriptor pairing. |
| [V-STEREO](stereo-association.md) | Calibrated cross-camera association and identity handling, only when stereo is enabled. |

## Acceptance and verification

Check keypoint/descriptor pairing after selection, count/mask bounds, empty and
invalid inputs, descriptor incompatibility, insufficient neighbors, distance ties,
reciprocal rejection, optional distance/scale gates, one-to-one conflicts and geometry failure.
Verify that rejected matches do not inherit track IDs and that retained
descriptors survive live-buffer reuse.
For the raw-image profile, verify raw/bearing pairing, calibration identity,
inverse failures and geometric-threshold units. Existing remapped-image tests
and benchmarks do not establish acceptance of this changed input domain.
Exercise static scenes, pure rotation, translating planar scenes, low parallax,
repeated textures, moving objects and model-selection boundaries. Measure track
loss as well as false continuations; conservative rejection can prevent startup.
Check spacing across cell/scale boundaries and include each applicable child's
acceptance in the composed pipeline.
Test a continuing low-response track competing with a new high-response corner,
and verify that capacity selection preserves valid history without preserving
failed associations. Measure history lengths reaching triangulation eligibility.

Evaluate track survival, spatial coverage, false associations with labeled data,
and estimator error on identified sequences. EuRoC body-pose truth is not dense
feature correspondence/depth truth. Measure extraction/matching latency and
memory on the selected target. Bounded published counts do not prove a bound on OpenCV's internal
allocation or worst-case execution time; passing geometry tests does not prove
that two observations correspond to the same physical point.

## Open decisions

Feature/grid budgets, detector and pyramid settings, spacing, scale compatibility,
ratio/absolute thresholds, model support/inlier thresholds, robust-estimation limits and
target latency/memory limits require a reviewed frontend profile. Camera choice
and coordinate calibration follow V-FRAMES. No numerical defaults or flight
performance claims are established by this requirement.

The [development profile](../../profiles/orb-frontend.md) identifies current
choices and acceptance gaps; it does not approve those values for deployment.
