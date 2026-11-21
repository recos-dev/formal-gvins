/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/selection.hpp>
#include "test_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    SelectionOptions options{3, 2, 1, 8};
    std::vector<SelectionCandidate> input{
        {49, 20, 1, 10, 3}, {51, 20, 1000, 0, 0}, {20, 20, 1, 20, 2}, {30, 20, 2000, 0, 0}, {80, 20, 1, 0, 0}};
    SelectionResult selected;
    test.expect(select_features(input, 100, 100, options, selected) == VisionStatus::success &&
                    selected.indices == std::vector<std::size_t>({0, 2, 4}),
                "selection", "old tracks win over stronger new detections; spacing crosses cell boundary");
    test.expect(selected.stats.quota_rejected == 1 && selected.stats.spacing_rejected == 1, "selection",
                "rejection accounting");
    options = {2, 1, 1, 8};
    input = {{40, 40, 1, 20, 3}, {40, 40, 2, 10, 3}, {48, 40, 1, 0, 0}, {60, 40, 1, 0, 0}};
    test.expect(select_features(input, 100, 100, options, selected) == VisionStatus::success &&
                    selected.indices == std::vector<std::size_t>({1, 2}),
                "selection", "stable ID breaks equal age; distance equality is admitted; new response ties use index");
    input[0].age = 4;
    test.expect(select_features(input, 100, 100, options, selected) == VisionStatus::success &&
                    selected.indices[0] == 0,
                "selection", "age precedes ID and detector response");
    auto saved = selected.indices;
    input[0].track_id = 10;
    test.expect(select_features(input, 100, 100, options, selected) == VisionStatus::invalid_input &&
                    selected.indices == saved,
                "selection", "duplicate existing identity fails atomically");
    input = {{0, 0, 1, 0, 0}, {100, 0, 1, 0, 0}, {50, 50, std::numeric_limits<double>::quiet_NaN(), 0, 0}};
    test.expect(select_features(input, 100, 100, options, selected) == VisionStatus::success &&
                    selected.indices == std::vector<std::size_t>({0}) && selected.stats.invalid_rejected == 2,
                "selection", "half-open bounds and nonfinite response before sorting");
    saved = selected.indices;
    options.minimum_distance = 0;
    test.expect(select_features(input, 100, 100, options, selected) == VisionStatus::invalid_input &&
                    selected.indices == saved,
                "selection", "invalid spacing preserves output");
    options = {400, 8, 6, 8};
    input.clear();
    for (int y = 0; y < 480; y += 7)
    {
        for (int x = 0; x < 752; x += 7)
        {
            input.push_back({double(x), double(y), double(x + y), 0, 0});
        }
    }
    test.expect(select_features(input, 752, 480, options, selected) == VisionStatus::success &&
                    selected.indices.size() <= 400,
                "selection", "dense candidate bound");
    test.expect(selected.indices.size() + selected.stats.quota_rejected + selected.stats.spacing_rejected ==
                    input.size(),
                "selection", "every eligible candidate accounted for");
    return test.failures() == 0 ? 0 : 1;
}
