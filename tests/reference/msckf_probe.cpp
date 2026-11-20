/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/linalg/backend/eigen.hpp>
#include <formal_gvins/msckf/prediction.hpp>
#include <iomanip>
#include <iostream>

using L = formal_gvins::linalg::EigenBackend<double>;
using Vector = formal_gvins::linalg::Vector<L, 3>;
void read(Vector & value)
{
    for (std::size_t i = 0; i < 3; ++i)
    {
        std::cin >> value(i);
    }
}
template <typename M> void print(M const & value)
{
    for (std::size_t r = 0; r < M::row_count; ++r)
    {
        for (std::size_t c = 0; c < M::column_count; ++c)
        {
            std::cout << value(r, c) << ' ';
        }
    }
}
int main()
{
    using namespace formal_gvins;
    Eigen::internal::set_is_malloc_allowed(false);
    msckf::ImuState<L> state;
    msckf::ImuInterval<L> interval;
    Vector angle, gravity;
    std::cin >> interval.end_ns;
    read(angle);
    read(state.pose.p_WB);
    read(state.v_WB);
    read(interval.a);
    read(interval.omega);
    read(gravity);
    if (!std::cin || !succeeded(so3::exp(angle, .5, state.pose.q_WB)))
    {
        return 2;
    }
    msckf::ImuState<L> output;
    if (!succeeded(msckf::predict_nominal(state, interval, gravity, msckf::Tolerances<double>{1e-12, 1e-12}, output)))
    {
        return 3;
    }
    std::cout << std::setprecision(17);
    print(output.pose.p_WB);
    print(output.v_WB);
    print(so3::to_rotation_matrix(output.pose.q_WB));
    std::cout << '\n';
}
