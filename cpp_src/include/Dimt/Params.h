#pragma once
// TODO: Convert to struct?
struct Params
{
    const int dof;
    const int dimensions;
    const double s_max;
    const double a_max;
    const double v_max;
};

constexpr Params param_1dof = {
    1,   // dof
    2,   // dimensions
    5.0,  // s_max
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_2dof = {
    2,   // dof
    4,   // dimensions
    5.0,  // s_max
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_3dof = {
    3,   // dof
    6,   // dimensions
    5.0,  // s_max
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_4dof = {
    4,   // dof
    8,   // dimensions
    5.0,  // s_max
    0.2,  // a_max
    4  // v_max
};
constexpr Params param_6dof = {
    6, // dof
    12, // dimensions
    5.0,  // s_max
    1.0, // a_max
    10 // v_max
};
constexpr Params param = param_4dof;
// constexpr Params param = param_1dof;
