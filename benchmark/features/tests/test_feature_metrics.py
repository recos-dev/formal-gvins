"""Independent answers for the shared feature scorer; no images or GPU required."""

from dataclasses import asdict
import itertools
import json
import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import feature_metrics as metrics
from feature_metrics import Match, Point, score_pair


FIXTURES = Path(__file__).parent / "fixtures" / "feature_pairs.json"


class PairMetricTests(unittest.TestCase):
    def test_known_answers(self):
        fixture = json.loads(FIXTURES.read_text(encoding="utf-8"))
        self.assertEqual(fixture["protocol"], "projected-pairs-v1")
        cases = fixture["cases"]
        self.assertTrue(cases)
        self.assertEqual(len({case["id"] for case in cases}), len(cases))
        for case in cases:
            with self.subTest(case=case["id"]):
                source = [Point(*point) for point in case["projected_source"]]
                target = [Point(*point) for point in case["target"]]
                matches = [Match(*match) for match in case["matches"]]
                result = score_pair(source, target, matches, case["tolerance_px"])
                expected = dict(case["expected"], source_points=len(source), target_points=len(target))
                actual = asdict(result)
                self.assertEqual(actual.keys(), expected.keys())
                for field, value in expected.items():
                    with self.subTest(field=field):
                        if value is None:
                            self.assertIsNone(actual[field])
                        elif field.endswith("_points") or field.endswith("_pairs"):
                            self.assertEqual(actual[field], value)
                        else:
                            self.assertTrue(math.isclose(actual[field], value, rel_tol=1e-14, abs_tol=0))

    def test_threshold_is_inclusive(self):
        for tolerance, correct in [(math.nextafter(5.0, 0.0), 0), (5.0, 1),
                                   (math.nextafter(5.0, math.inf), 1)]:
            with self.subTest(tolerance=tolerance):
                result = score_pair([Point(0, 0)], [Point(3, 4)], [Match(0, 0)], tolerance)
                self.assertEqual(result.geometric_pairs, correct)
                self.assertEqual(result.correct_pairs, correct)
                self.assertEqual(result.rmse_all_px, 5.0)

    def test_empty_domains_have_no_fabricated_scores(self):
        for source, target in [([], []), ([Point(0, 0)], []),
                               ([Point(0, 0, False)], [Point(0, 0, False)])]:
            with self.subTest(source=source, target=target):
                result = score_pair(source, target, [], 1)
                self.assertEqual(result.geometric_pairs, 0)
                self.assertIsNone(result.repeatability)
                self.assertIsNone(result.precision)
                self.assertIsNone(result.recall)
                self.assertIsNone(result.rmse_all_px)
                self.assertIsNone(result.rmse_correct_px)

    def test_repeatability_uses_smaller_eligible_population(self):
        result = score_pair(
            [Point(0, 0), Point(10, 0), Point(20, 0)], [Point(0, 0), Point(100, 0)], [], 1
        )
        self.assertEqual(result.geometric_pairs, 1)
        self.assertEqual(result.repeatability, 0.5)

    def test_recall_is_detection_conditioned_and_not_precision(self):
        result = score_pair(
            [Point(0, 0), Point(10, 0), Point(20, 0)],
            [Point(0, 0), Point(10, 0), Point(20, 0)], [Match(1, 1)], 0
        )
        self.assertEqual(result.precision, 1)
        self.assertEqual(result.recall, 1 / 3)
        self.assertEqual(result.repeatability, 1)

    def test_reference_ineligibility_on_either_side_remains_false_positive(self):
        for source_eligible, target_eligible in [(False, True), (True, False), (False, False)]:
            with self.subTest(source=source_eligible, target=target_eligible):
                result = score_pair(
                    [Point(0, 0, source_eligible)], [Point(0, 0, target_eligible)], [Match(0, 0)], 0
                )
                self.assertEqual(result.accepted_pairs, 1)
                self.assertEqual(result.correct_pairs, 0)
                self.assertEqual(result.precision, 0)
                self.assertEqual(result.evaluated_pairs, 0)
                self.assertIsNone(result.rmse_all_px)

    def test_permuted_indices_and_match_order_preserve_scores(self):
        source = [Point(0, 0), Point(-0.5, 0), Point(20, 0)]
        target = [Point(0, 0), Point(0.5, 0), Point(30, 0)]
        matches = [Match(0, 1), Match(1, 0), Match(2, 2)]
        expected = score_pair(source, target, matches, 0.6)
        for source_order in itertools.permutations(range(3)):
            for target_order in itertools.permutations(range(3)):
                remapped = [Match(source_order.index(m.source), target_order.index(m.target))
                            for m in reversed(matches)]
                result = score_pair([source[i] for i in source_order],
                                    [target[i] for i in target_order], remapped, 0.6)
                self.assertEqual(result, expected)

    def test_duplicate_or_conflicting_matches_are_rejected(self):
        points = [Point(0, 0), Point(1, 0)]
        for matches in [[Match(0, 0), Match(0, 0)], [Match(0, 0), Match(0, 1)],
                        [Match(0, 0), Match(1, 0)]]:
            with self.subTest(matches=matches), self.assertRaisesRegex(ValueError, "one-to-one"):
                score_pair(points, points, matches, 1)

    def test_invalid_match_indices_are_rejected(self):
        for index in [-1, 1, True, 0.0, "0"]:
            for match in [Match(index, 0), Match(0, index)]:
                with self.subTest(match=match), self.assertRaisesRegex(ValueError, "index"):
                    score_pair([Point(0, 0)], [Point(0, 0)], [match], 1)
        with self.assertRaisesRegex(ValueError, "index"):
            score_pair([], [], [Match(0, 0)], 1)

    def test_invalid_tolerances_are_rejected(self):
        for tolerance in [-1, math.nan, math.inf, -math.inf, True, "1", 10**400]:
            with self.subTest(tolerance=tolerance), self.assertRaises(ValueError):
                score_pair([], [], [], tolerance)

    def test_invalid_coordinates_are_rejected_even_when_ineligible(self):
        for value in [math.nan, math.inf, -math.inf, True, "0", 10**400]:
            for point in [Point(value, 0), Point(0, value), Point(value, 0, False)]:
                with self.subTest(point=point):
                    with self.assertRaises(ValueError):
                        score_pair([point], [], [], 1)
                    with self.assertRaises(ValueError):
                        score_pair([], [point], [], 1)

    def test_eligibility_must_be_boolean(self):
        for eligibility in [0, 1, None, "false"]:
            with self.subTest(eligibility=eligibility), self.assertRaisesRegex(ValueError, "boolean"):
                score_pair([Point(0, 0, eligibility)], [], [], 1)

    def test_rmse_preserves_large_and_small_finite_errors(self):
        for distance in [1e200, 1e-200]:
            with self.subTest(distance=distance):
                result = score_pair([Point(0, 0), Point(0, 0)],
                                    [Point(0, distance), Point(0, distance / 2)],
                                    [Match(0, 0), Match(1, 1)], distance)
                # Mean square of d and d/2 is (5/8)*d^2; never square d here.
                expected = distance * math.sqrt(5 / 8)
                self.assertTrue(math.isclose(result.rmse_all_px, expected, rel_tol=1e-14))
                self.assertTrue(math.isclose(result.rmse_correct_px, expected, rel_tol=1e-14))

    def test_unrepresentable_distance_is_rejected(self):
        for coordinate in [1e308, 10**308]:
            with self.subTest(coordinate=coordinate), self.assertRaisesRegex(ValueError, "representable"):
                score_pair([Point(-coordinate, 0)], [Point(coordinate, 0)], [], 1)

    def test_inputs_are_not_modified_on_success_or_failure(self):
        source, target = [Point(0, 0)], [Point(1, 0)]
        matches = [Match(0, 0)]
        original = (source[:], target[:], matches[:])
        score_pair(source, target, matches, 1)
        self.assertEqual((source, target, matches), original)
        with self.assertRaises(ValueError):
            score_pair(source, target, matches, -1)
        self.assertEqual((source, target, matches), original)

    def test_unavailable_metrics_serialize_as_null_not_nan(self):
        result = score_pair([], [], [], 1)
        encoded = json.dumps(asdict(result), allow_nan=False)
        self.assertIsNone(json.loads(encoded)["precision"])


class AssignmentTests(unittest.TestCase):
    def test_all_three_by_three_graphs_against_exhaustive_assignments(self):
        # Independent oracle: enumerate all assignments, including unmatched (-1).
        assignments = [a for a in itertools.product(range(-1, 3), repeat=3)
                       if len([t for t in a if t != -1]) == len({t for t in a if t != -1})]
        for edges in range(1 << 9):
            neighbors = [[t for t in range(3) if edges & (1 << (s * 3 + t))] for s in range(3)]
            expected = max(sum(t != -1 for t in a) for a in assignments
                           if all(t == -1 or t in neighbors[s] for s, t in enumerate(a)))
            with self.subTest(edges=edges):
                self.assertEqual(metrics._maximum_pair_count(neighbors, 3), expected)


if __name__ == "__main__":
    unittest.main()
