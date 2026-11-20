# V-ORB: ORB feature extraction and temporal association

**ID:** `V-ORB`

**Status:** Draft

**Applies to:** The first image-frontend profile; no SLAM map or relocalization implementation is implied.

**Uses:** [V-FRAMES](../common/frames.md), [V-TRACK](feature-tracks.md), [V-EXECUTION](frontend-execution.md).

## Basis

The selected baseline uses OpenCV ORB keypoints and binary descriptors. The
[ORB](https://docs.opencv.org/4.13.0/db/d95/classcv_1_1ORB.html) and
[matcher](https://docs.opencv.org/4.13.0/d3/da1/classcv_1_1BFMatcher.html)
references define library semantics; version 4.13.0 is a documentation reference,
not a deployment-version decision. Descriptor tracking with a ratio test and
geometric rejection is also demonstrated by
[OpenVINS TrackDescriptor](https://docs.openvins.com/classov__core_1_1TrackDescriptor.html).
This requirement does not import its source, thresholds or quaternion convention.

## Required behavior

### Extraction and output

The frontend shall use ORB detection and description on an admitted image. Its
configuration shall identify the OpenCV implementation/version, preprocessing,
image dimensions, pyramid settings, detector settings and feature budget.
Invalid image/configuration input shall not create observations or alter existing
track associations.

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
cannot consume the entire allocation; empty regions need not produce artificial features. Descriptor
storage shall be owned or have a lifetime covering every consumer. A retained
keyframe shall own the descriptors and coordinate/calibration metadata needed
after the live image buffers are reused.

An image with no usable features shall produce an explicit empty observation set,
not fabricated correspondences or a reuse of the last image's measurements.
ORB responses and descriptor distances shall not be treated as calibrated
pixel-noise variance or match probabilities.

### Temporal association

For each candidate temporal association, the frontend shall compare compatible
descriptors and apply a configured nearest-neighbor ratio threshold $0<\rho<1$.
Let $d_1\le d_2$ be the two nearest candidate distances. The ratio test shall
accept only when two candidates exist and $d_1<\rho d_2$. Ambiguous equal-distance
matches, including two zero distances, shall not pass this test.

Accepted associations shall additionally pass geometric verification under a
declared camera/motion model. A bounded robust-estimation procedure shall reject
outliers; failure to obtain an admissible model shall not be interpreted as all
matches being inliers. A descriptor match alone shall not create a confirmed
continuation of a track.

Within a camera/frame pair, published associations shall be one-to-one. Conflicting
candidates shall be rejected or resolved by the declared selection policy before
track IDs are assigned. The frontend shall preserve identity according to
[V-TRACK](feature-tracks.md), not infer identity from descriptor equality.

### Estimator boundary

The estimator-facing observations shall contain identities, acquisition time,
coordinates and their calibration/noise-model binding. OpenCV containers and
descriptor-search state shall remain outside the numerical core. Cross-camera
association, landmark retrieval, PnP and loop constraints require their own
acceptance contracts; temporal matching does not establish them.

## Acceptance and verification

Check keypoint/descriptor pairing after selection, count/mask bounds, empty and
invalid inputs, descriptor incompatibility, insufficient neighbors, distance ties,
one-to-one conflicts and geometry-estimation failure. Verify that rejected matches
do not inherit track IDs and that retained descriptors survive live-buffer reuse.

Evaluate track survival, spatial coverage, false associations and estimator error
on identified sequences. Measure extraction/matching latency and memory on the
selected target. Bounded published counts do not prove a bound on OpenCV's internal
allocation or worst-case execution time; passing geometry tests does not prove
that two observations correspond to the same physical point.

## Open decisions

Feature/grid budgets, detector and pyramid settings, ratio threshold, geometric
model, inlier thresholds, robust-estimation iteration limit, conflict policy and
target latency/memory limits require a reviewed frontend profile. Camera choice
and coordinate calibration follow V-FRAMES. No numerical defaults or flight
performance claims are established by this requirement.

The [development profile](../../profiles/orb-frontend.md) identifies current
choices and acceptance gaps; it does not approve those values for deployment.
