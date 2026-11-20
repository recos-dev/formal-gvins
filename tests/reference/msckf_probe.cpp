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
int main(int argc, char **)
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
    if (argc > 1)
    {
        msckf::State<L, 2> joint;
        joint.imu = state;
        Vector first_angle;
        read(first_angle);
        read(joint.first.pose.p_WB);
        read(joint.first.v_WB);
        if (!succeeded(so3::exp(first_angle, .5, joint.first.pose.q_WB)))
        {
            return 4;
        }
        linalg::Matrix<L, 12, 12> Qc;
        for (std::size_t i = 0; i < 12; ++i)
        {
            for (std::size_t j = 0; j < 12; ++j)
            {
                std::cin >> Qc(i, j);
            }
        }
        if (!std::cin)
        {
            return 5;
        }
        joint.clone_count = 2;
        joint.time_ns = 2;
        for (std::size_t i = 0; i < 2; ++i)
        {
            joint.clones[i].time_ns = i;
            joint.clones[i].id = i;
        }
        (void)joint.P.resize(27, 27);
        for (std::size_t i = 0; i < 27; ++i)
        {
            for (std::size_t j = 0; j < 27; ++j)
            {
                joint.P(i, j) = (i == j ? .1 : 0) + .0001 * (i + 1) * (j + 1);
            }
        }
        interval.start_ns = 2;
        interval.end_ns += 2;
        msckf::PredictionWorkspace<L, 2> work;
        if (!succeeded(msckf::predict(joint, interval, gravity, Qc, msckf::Tolerances<double>{1e-12, 1e-12}, work)))
        {
            return 6;
        }
        print(work.Psi);
        print(work.Qd);
        for (std::size_t i = 0; i < 27; ++i)
        {
            for (std::size_t j = 0; j < 27; ++j)
            {
                std::cout << joint.P(i, j) << ' ';
            }
        }
    }
    std::cout << '\n';
}
