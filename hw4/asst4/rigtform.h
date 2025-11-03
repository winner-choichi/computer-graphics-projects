#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm
{
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0)
  {
    assert(norm2(Quat(1, 0, 0, 0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3 &t, const Quat &r)
  {
    t_ = t;
    r_ = r;
  }

  explicit RigTForm(const Cvec3 &t)
  {
    t_ = t;
    r_ = Quat(1, 0, 0, 0);
  }

  explicit RigTForm(const Quat &r)
  {
    t_ = Cvec3(0, 0, 0);
    r_ = r;
  }

  Cvec3 getTranslation() const
  {
    return t_;
  }

  Quat getRotation() const
  {
    return r_;
  }

  RigTForm &setTranslation(const Cvec3 &t)
  {
    t_ = t;
    return *this;
  }

  RigTForm &setRotation(const Quat &r)
  {
    r_ = r;
    return *this;
  }

  Cvec4 operator*(const Cvec4 &a) const
  {
    return Cvec4(0, t_[0], t_[1], t_[3]) + r_ * a; // asst3, 쿼터니언 곱으로 회전하고 t_더해서 translation
  }

  RigTForm operator*(const RigTForm &a) const
  {
    const Cvec4 rt = r_ * Cvec4(a.getTranslation(), 0); // asst3, r은 둘이 곱한거, t는 t1에 r1t2더한거
    RigTForm ret;
    ret.setTranslation(t_ + Cvec3(rt[0], rt[1], rt[2]));
    ret.setRotation(r_ * a.getRotation());
    return ret;
  }
};

inline RigTForm inv(const RigTForm &tform)
{
  const Cvec4 rt = inv(tform.getRotation()) * Cvec4(tform.getTranslation(), 0) * -1; // asst3, r은 그냥 inv, t는 -inv(r)곱한거
  RigTForm ret;
  ret.setRotation(inv(tform.getRotation()));
  ret.setTranslation(Cvec3(rt[0], rt[1], rt[2]));
  return ret;
}

inline RigTForm transFact(const RigTForm &tform)
{
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm &tform)
{
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm &tform)
{
  Matrix4 m = quatToMatrix(tform.getRotation()); // asst3, r 행렬에 t만 추가한거
  Cvec3 t = tform.getTranslation();
  m(0, 3) = t[0];
  m(1, 3) = t[1];
  m(2, 3) = t[2];

  return m;
}

#endif
