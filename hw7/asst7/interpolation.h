#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <algorithm>
#include <cmath>

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

Frame get_frame_linear_interpolation(const Frame &f0, const Frame &f1, double a)
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

// return CR interpolated frame within f1 and f2
Frame get_frame_CR_interpolation(const Frame &f0, const Frame &f1, const Frame &f2, const Frame &f3, double a)
{
    std::vector<RigTForm> new_rbts;
    std::vector<RigTForm> f0_rbts = f0.get_rbts();
    std::vector<RigTForm> f1_rbts = f1.get_rbts();
    std::vector<RigTForm> f2_rbts = f2.get_rbts();
    std::vector<RigTForm> f3_rbts = f3.get_rbts();

    std::vector<RigTForm> fd_rbts, fe_rbts; // intermediate frames are determined by f1, fd, fe, f2 with cubic Bezier funciton

    for (int i = 0; i < f0_rbts.size(); ++i)
    {

        Cvec3 t_d = (f2_rbts[i].getTranslation() - f0_rbts[i].getTranslation()) * (1.0 / 6.0) + f1_rbts[i].getTranslation();
        Quat q_d;
        if (dot(f2_rbts[i].getRotation(), inv(f0_rbts[i].getRotation())) < 0)
        {
            q_d = ((f2_rbts[i].getRotation() * -1 * inv(f0_rbts[i].getRotation())) ^ (1.0 / 6.0)) * f1_rbts[i].getRotation();
        }
        else
        {
            q_d = ((f2_rbts[i].getRotation() * inv(f0_rbts[i].getRotation())) ^ (1.0 / 6.0)) * f1_rbts[i].getRotation();
        }

        Cvec3 t_e = (f3_rbts[i].getTranslation() - f1_rbts[i].getTranslation()) * (-1.0 / 6.0) + f2_rbts[i].getTranslation();
        Quat q_e;
        if (dot(f3_rbts[i].getRotation(), inv(f1_rbts[i].getRotation())) < 0)
        {
            q_e = ((f3_rbts[i].getRotation() * -1 * inv(f1_rbts[i].getRotation())) ^ (-1.0 / 6.0)) * f2_rbts[i].getRotation();
        }
        else
        {
            q_e = ((f3_rbts[i].getRotation() * inv(f1_rbts[i].getRotation())) ^ (-1.0 / 6.0)) * f2_rbts[i].getRotation();
        }

        fd_rbts.push_back(RigTForm(t_d, q_d));
        fe_rbts.push_back(RigTForm(t_e, q_e));
    }

    for (int i = 0; i < f0_rbts.size(); ++i)
    {
        Cvec3 t = f1_rbts[i].getTranslation() * pow(1 - a, 3) + fd_rbts[i].getTranslation() * 3 * a * pow(1 - a, 2) + fe_rbts[i].getTranslation() * 3 * pow(a, 2) * (1 - a) + f2_rbts[i].getTranslation() * pow(a, 3);
        Quat q10 = get_q_slerp(f1_rbts[i].getRotation(), fd_rbts[i].getRotation(), a);
        Quat q11 = get_q_slerp(fd_rbts[i].getRotation(), fe_rbts[i].getRotation(), a);
        Quat q12 = get_q_slerp(fe_rbts[i].getRotation(), f2_rbts[i].getRotation(), a);
        Quat q20 = get_q_slerp(q10, q11, a);
        Quat q21 = get_q_slerp(q11, q12, a);
        Quat q = get_q_slerp(q20, q21, a);
        new_rbts.push_back(RigTForm(t, q));
    }

    return Frame(new_rbts);
}

#endif