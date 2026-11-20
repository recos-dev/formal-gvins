/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <formal_gvins/msckf/state.hpp>

namespace formal_gvins::test
{
template <typename L> bool equal_pose(msckf::Pose<L> const & a, msckf::Pose<L> const & b)
{
    return linalg::max_abs(a.p_WB - b.p_WB) == 0 && so3::same_coefficients(a.q_WB, b.q_WB);
}
template <typename L, std::size_t N> bool equal_state(msckf::State<L, N> const & a, msckf::State<L, N> const & b)
{
    if (a.clone_count != b.clone_count || a.time_ns != b.time_ns || a.P.rows() != b.P.rows() ||
        a.P.columns() != b.P.columns() || !equal_pose(a.imu.pose, b.imu.pose) ||
        !equal_pose(a.first.pose, b.first.pose) || linalg::max_abs(a.imu.v_WB - b.imu.v_WB) != 0 ||
        linalg::max_abs(a.first.v_WB - b.first.v_WB) != 0 || linalg::max_abs(a.imu.b_a - b.imu.b_a) != 0 ||
        linalg::max_abs(a.imu.b_g - b.imu.b_g) != 0)
    {
        return false;
    }
    for (std::size_t i = 0; i < a.clone_count; ++i)
    {
        if (!equal_pose(a.clones[i].pose, b.clones[i].pose) || !equal_pose(a.clones[i].first, b.clones[i].first) ||
            a.clones[i].id != b.clones[i].id || a.clones[i].time_ns != b.clones[i].time_ns)
        {
            return false;
        }
    }
    for (std::size_t i = 0; i < a.P.rows(); ++i)
    {
        for (std::size_t j = 0; j < a.P.columns(); ++j)
        {
            if (a.P(i, j) != b.P(i, j))
            {
                return false;
            }
        }
    }
    return true;
}
} /* end namespace formal_gvins::test */
