#pragma once
// TODO: Convert to struct?
struct Params
{
    const int dof;
    const int dimensions;
    const double a_max;
    const double v_max;
};

constexpr Params param_1dof = {
    1,   // dof
    2,   // dimensions
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_2dof = {
    2,   // dof
    4,   // dimensions
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_3dof = {
    3,   // dof
    6,   // dimensions
    1.0,  // a_max
    10   // v_max
};
constexpr Params param = param_2dof;
// constexpr Params param = param_1dof;
