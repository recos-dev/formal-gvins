/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/camera/radtan.hpp>
#include <formal_gvins/scalar/backend/standard.hpp>
#include "test_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins;
    using Math = scalar::StandardMath<double>;
    test::Context test;
    camera::InverseDomain<double> domain;
    camera::Radtan<double> lens{458.654, 457.296, 367.215, 248.375, -.28340811, .07395907, .00019359, 1.76187114e-5};
    camera::Bearing<double> result;
    for (int model = 0; model < 2; ++model)
    {
        auto const calibrated = model == 0 ? lens : camera::Radtan<double>{400, 450, 320, 240};
        for (int i = -10; i <= 10; ++i)
        {
            for (int j = -10; j <= 10; ++j)
            {
                // Independent higher-precision forward expression, not the production helper.
                long double const x = i * .08L, y = j * .06L;
                long double const r2 = x * x + y * y;
                long double const s = 1 + calibrated.k1 * r2 + calibrated.k2 * r2 * r2;
                double const u = static_cast<double>(
                    calibrated.fx * (x * s + 2 * calibrated.p1 * x * y + calibrated.p2 * (3 * x * x + y * y)) +
                    calibrated.cx);
                double const v = static_cast<double>(
                    calibrated.fy * (y * s + calibrated.p1 * (x * x + 3 * y * y) + 2 * calibrated.p2 * x * y) +
                    calibrated.cy);
                test.expect(succeeded(camera::unproject<Math>(u, v, calibrated, domain, result)), "inverse",
                            "qualified point succeeds");
                test.expect(std::abs(result.x - x) < 1e-10 && std::abs(result.y - y) < 1e-10, "inverse",
                            "known bearing recovered");
            }
        }
    }
    // EuRoC raw image domain, including its corners, must be admitted without clipping.
    for (int u = 0; u < 752; u += 7)
    {
        for (int v = 0; v < 480; v += 7)
        {
            test.expect(succeeded(camera::unproject<Math>(u, v, lens, domain, result)), "inverse", "raw image grid");
        }
    }
    auto const saved = result;
    auto rejects = [&](camera::Radtan<double> const & c, camera::InverseDomain<double> const & d, double u, double v)
    {
        test.expect(!succeeded(camera::unproject<Math>(u, v, c, d, result)) && result.x == saved.x &&
                        result.y == saved.y,
                    "inverse", "failure preserves output");
    };
    auto invalid = lens;
    invalid.fx = 0;
    rejects(invalid, domain, 300, 200);
    invalid = lens;
    invalid.k1 = -1;
    invalid.k2 = 0;
    rejects(invalid, domain, 300, 200); // Folded domain, even near principal point.
    invalid = lens;
    invalid.p1 = 1;
    rejects(invalid, domain, 300, 200);
    auto limited = domain;
    limited.iterations = 1;
    rejects(lens, limited, 20, 20);
    limited = domain;
    limited.maximum_step = 1e-8;
    rejects(lens, limited, 20, 20);
    limited = domain;
    limited.radius = .2;
    rejects(lens, limited, 20, 20);
    limited = domain;
    limited.iterations = 33;
    rejects(lens, limited, 300, 200);
    rejects(lens, domain, std::numeric_limits<double>::quiet_NaN(), 200);
    rejects(lens, domain, std::numeric_limits<double>::max(), 200);
    invalid = lens;
    invalid.k1 = std::numeric_limits<double>::infinity();
    rejects(invalid, domain, 300, 200);
    return test.failures() == 0 ? 0 : 1;
}
