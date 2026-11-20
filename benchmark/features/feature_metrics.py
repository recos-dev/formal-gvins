"""Independent host-side feature scoring; no detector or execution backend.

Source points have already been projected into the target pixel domain by the
reference geometry. Eligibility comes from that reference's common visibility
region, never a frontend's match confidence or estimated RANSAC mask.
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    eligible: bool = True


@dataclass(frozen=True)
class Match:
    source: int
    target: int


@dataclass(frozen=True)
class PairMetrics:
    source_points: int
    target_points: int
    eligible_source_points: int
    eligible_target_points: int
    geometric_pairs: int
    accepted_pairs: int
    correct_pairs: int
    evaluated_pairs: int
    repeatability: float | None
    precision: float | None
    recall: float | None
    rmse_all_px: float | None
    rmse_correct_px: float | None


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        valid = math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(f"{name} must be a finite number")


def _validate_points(points):
    for point in points:
        _finite(point.x, "point x")
        _finite(point.y, "point y")
        if type(point.eligible) is not bool:
            raise ValueError("point eligibility must be a reference boolean")


def _maximum_pair_count(neighbors, target_count):
    """Maximum bipartite matching using iterative augmenting paths.

    A greedy nearest-target assignment can undercount the recall denominator.
    Input index order resolves search ties; the maximum count is order invariant.
    """
    source_match = [-1] * len(neighbors)
    target_match = [-1] * target_count
    count = 0
    for root in range(len(neighbors)):
        parent = [-1] * target_count
        queue = deque([root])
        endpoint = -1
        while queue and endpoint == -1:
            source = queue.popleft()
            for target in neighbors[source]:
                if parent[target] != -1:
                    continue
                parent[target] = source
                if target_match[target] == -1:
                    endpoint = target
                    break
                queue.append(target_match[target])
        if endpoint == -1:
            continue
        while endpoint != -1:
            source = parent[endpoint]
            previous = source_match[source]
            source_match[source] = endpoint
            target_match[endpoint] = source
            endpoint = previous
        count += 1
    return count


def _rmse(errors):
    if not errors:
        return None
    scale = max(errors)
    if scale == 0:
        return 0.0
    # Scale before squaring so finite pixel errors do not overflow or underflow.
    mean_square = math.fsum((error / scale) ** 2 for error in errors) / len(errors)
    return scale * math.sqrt(mean_square)


def _ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def score_pair(
    projected_source: Sequence[Point],
    target: Sequence[Point],
    matches: Sequence[Match],
    tolerance_px: float,
) -> PairMetrics:
    """Score one pair against independent, finite reference projections.

    The nonnegative tolerance includes equality. Matches must be one-to-one and
    refer to existing indices. Matches to reference-ineligible points count as
    false positives but have no scored transfer error. Zero denominators and
    empty error sets return None, not zero. Invalid data raises ValueError.
    This returns measurements, never an acceptance or regression PASS verdict.
    """
    _finite(tolerance_px, "tolerance_px")
    if tolerance_px < 0:
        raise ValueError("tolerance_px must be nonnegative")
    _validate_points(projected_source)
    _validate_points(target)

    predicted = {}
    used_target = set()
    for match in matches:
        if (
            type(match.source) is not int or type(match.target) is not int
            or not 0 <= match.source < len(projected_source)
            or not 0 <= match.target < len(target)
        ):
            raise ValueError("match index is invalid")
        if match.source in predicted or match.target in used_target:
            raise ValueError("matches must be one-to-one")
        predicted[match.source] = match.target
        used_target.add(match.target)

    neighbors = [[] for _ in projected_source]
    errors = []
    for source_index, source in enumerate(projected_source):
        if not source.eligible:
            continue
        matched_target = predicted.get(source_index)
        for target_index, destination in enumerate(target):
            if not destination.eligible:
                continue
            try:
                distance = math.hypot(destination.x - source.x, destination.y - source.y)
            except OverflowError as error:
                raise ValueError("pixel transfer distance is not representable") from error
            if not math.isfinite(distance):
                raise ValueError("pixel transfer distance is not representable")
            if distance <= tolerance_px:
                neighbors[source_index].append(target_index)
            if matched_target == target_index:
                errors.append(distance)

    geometric_pairs = _maximum_pair_count(neighbors, len(target))
    correct_errors = [error for error in errors if error <= tolerance_px]
    eligible_source = sum(point.eligible for point in projected_source)
    eligible_target = sum(point.eligible for point in target)
    return PairMetrics(
        source_points=len(projected_source),
        target_points=len(target),
        eligible_source_points=eligible_source,
        eligible_target_points=eligible_target,
        geometric_pairs=geometric_pairs,
        accepted_pairs=len(matches),
        correct_pairs=len(correct_errors),
        evaluated_pairs=len(errors),
        repeatability=_ratio(geometric_pairs, min(eligible_source, eligible_target)),
        precision=_ratio(len(correct_errors), len(matches)),
        recall=_ratio(len(correct_errors), geometric_pairs),
        rmse_all_px=_rmse(errors),
        rmse_correct_px=_rmse(correct_errors),
    )
