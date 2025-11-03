#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <algorithm>

#include "keyframes.h"

// return linearly interpolated Cvec3 over t1, t2, a
Cvec3 get_t_lerp(const Cvec3 &t0, const Cvec3 &t1, double a)
{
    return t0 * (1 - a) + t1 * a;
}

Quat get_q_slerp(Quat q0, Quat q1, double a)
{
    // cn
    if (dot(q0, q1) < 0)
    {
        q1 = q1 * -1;
    }
    Quat q = q1 * inv(q0);

    return (q ^ a) * q0;
}

Frame get_frame_interpolation(const Frame &f0, const Frame &f1, double a)
{
    std::vector<RigTForm> new_rbts;
    std::vector<RigTForm> f0_rbts = f0.get_rbts();
    std::vector<RigTForm> f1_rbts = f1.get_rbts();
    for (int i = 0; i < f0_rbts.size(); ++i)
    {
        Cvec3 t = get_t_lerp(f0_rbts[i].getTranslation(), f1_rbts[i].getTranslation(), a);
        Quat q = get_q_slerp(f0_rbts[i].getRotation(), f1_rbts[i].getRotation(), a);
        new_rbts.push_back(RigTForm(t, q));
    }

    return Frame(new_rbts);
}

#endif