// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: ExoMomentum
 *
 * Args:
 *     q: Matrix61
 *     dq: Matrix61
 *
 * Outputs:
 *     res: Matrix21
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> Exomomentum(const Eigen::Matrix<Scalar, 6, 1>& q,
                                        const Eigen::Matrix<Scalar, 6, 1>& dq) {
  // Total ops: 180

  // Input arrays

  // Intermediate terms (39)
  const Scalar _tmp0 = Scalar(0.017453292519943295) * q(0, 0);
  const Scalar _tmp1 = _tmp0 + Scalar(1.5707963267948966);
  const Scalar _tmp2 = std::sin(_tmp1);
  const Scalar _tmp3 = std::cos(_tmp1);
  const Scalar _tmp4 = Scalar(1.7453292519943295) * dq(0, 0);
  const Scalar _tmp5 = _tmp4 + Scalar(1.7453292519943295) * dq(1, 0);
  const Scalar _tmp6 = -_tmp5;
  const Scalar _tmp7 = _tmp0 + Scalar(0.017453292519943295) * q(1, 0);
  const Scalar _tmp8 = std::sin(_tmp7);
  const Scalar _tmp9 = std::cos(_tmp7);
  const Scalar _tmp10 = _tmp5 + Scalar(1.7453292519943295) * dq(2, 0);
  const Scalar _tmp11 = -_tmp10;
  const Scalar _tmp12 = _tmp7 + Scalar(0.017453292519943295) * q(2, 0);
  const Scalar _tmp13 = std::sin(_tmp12);
  const Scalar _tmp14 = std::cos(_tmp12);
  const Scalar _tmp15 = _tmp10 - Scalar(1.7453292519943295) * dq(3, 0);
  const Scalar _tmp16 = _tmp15 - Scalar(1.7453292519943295) * dq(4, 0);
  const Scalar _tmp17 = -_tmp16;
  const Scalar _tmp18 = Scalar(0.017453292519943295) * q(3, 0);
  const Scalar _tmp19 = _tmp12 - _tmp18 - Scalar(0.017453292519943295) * q(4, 0);
  const Scalar _tmp20 = std::sin(_tmp19);
  const Scalar _tmp21 = std::cos(_tmp19);
  const Scalar _tmp22 = _tmp18 + Scalar(3.1415926535897931);
  const Scalar _tmp23 = std::sin(_tmp22);
  const Scalar _tmp24 = Scalar(6.123233995736766e-17) * _tmp13 + Scalar(1.0) * _tmp14;
  const Scalar _tmp25 = std::cos(_tmp22);
  const Scalar _tmp26 = Scalar(0.22739999999999999) * _tmp24;
  const Scalar _tmp27 = Scalar(1.0) * _tmp13 - Scalar(6.123233995736766e-17) * _tmp14;
  const Scalar _tmp28 = _tmp23 * _tmp27;
  const Scalar _tmp29 = Scalar(0.0025999999999999999) * _tmp25;
  const Scalar _tmp30 = _tmp16 - Scalar(1.7453292519943295) * dq(5, 0);
  const Scalar _tmp31 = Scalar(6.123233995736766e-17) * _tmp20 - Scalar(1.0) * _tmp21;
  const Scalar _tmp32 = Scalar(0.017453292519943295) * q(5, 0) + Scalar(1.5707963267948966);
  const Scalar _tmp33 = std::sin(_tmp32);
  const Scalar _tmp34 = Scalar(0.064199999999999993) * _tmp33;
  const Scalar _tmp35 = std::cos(_tmp32);
  const Scalar _tmp36 = Scalar(1.0) * _tmp20 + Scalar(6.123233995736766e-17) * _tmp21;
  const Scalar _tmp37 = Scalar(0.063) * _tmp33;
  const Scalar _tmp38 = Scalar(0.064199999999999993) * _tmp35;

  // Output terms (1)
  Eigen::Matrix<Scalar, 2, 1> _res;

  _res(0, 0) =
      Scalar(3.0242665000000004) * _tmp11 *
          (Scalar(2.6819764901327034e-17) * _tmp13 + Scalar(0.438) * _tmp14) +
      Scalar(10.205830000000001) * _tmp11 *
          (Scalar(0.2858) * _tmp13 + Scalar(0.30530000000000002) * _tmp14) -
      Scalar(1.2) * _tmp15 *
          (-Scalar(0.0025999999999999999) * _tmp23 * _tmp24 + _tmp25 * _tmp26 + _tmp27 * _tmp29 +
           Scalar(0.22739999999999999) * _tmp28) +
      Scalar(0.82426650000000001) * _tmp17 *
          (Scalar(2.6819764901327034e-17) * _tmp20 - Scalar(0.438) * _tmp21) +
      _tmp17 * (Scalar(0.039200000000000013) * _tmp20 - Scalar(0.1968) * _tmp21) -
      Scalar(0.82426650000000001) * _tmp30 *
          (-_tmp31 * _tmp34 + Scalar(0.063) * _tmp31 * _tmp35 - _tmp36 * _tmp37 - _tmp36 * _tmp38) -
      _tmp4 * (Scalar(0.25569999999999998) * _tmp2 - Scalar(0.023599999999999999) * _tmp3) +
      Scalar(1.2) * _tmp6 *
          (-Scalar(0.0025999999999999869) * _tmp8 + Scalar(0.21049999999999999) * _tmp9) +
      Scalar(13.2300965) * _tmp6 *
          (Scalar(2.7585169150794132e-17) * _tmp8 + Scalar(0.45050000000000001) * _tmp9);
  _res(1, 0) =
      Scalar(3.0242665000000004) * _tmp10 *
          (-Scalar(0.438) * _tmp13 + Scalar(2.6819764901327034e-17) * _tmp14) +
      Scalar(10.205830000000001) * _tmp10 *
          (-Scalar(0.30530000000000002) * _tmp13 + Scalar(0.2858) * _tmp14) +
      Scalar(1.2) * _tmp15 *
          (_tmp23 * _tmp26 + _tmp24 * _tmp29 - Scalar(0.22739999999999999) * _tmp25 * _tmp27 +
           Scalar(0.0025999999999999999) * _tmp28) +
      _tmp16 * (Scalar(0.1968) * _tmp20 + Scalar(0.039200000000000013) * _tmp21) +
      Scalar(0.82426650000000001) * _tmp16 *
          (Scalar(0.438) * _tmp20 + Scalar(2.6819764901327034e-17) * _tmp21) +
      Scalar(0.82426650000000001) * _tmp30 *
          (_tmp31 * _tmp37 + _tmp31 * _tmp38 - _tmp34 * _tmp36 + Scalar(0.063) * _tmp35 * _tmp36) +
      _tmp4 * (Scalar(0.023599999999999999) * _tmp2 + Scalar(0.25569999999999998) * _tmp3) +
      Scalar(13.2300965) * _tmp5 *
          (-Scalar(0.45050000000000001) * _tmp8 + Scalar(2.7585169150794132e-17) * _tmp9) +
      Scalar(1.2) * _tmp5 *
          (-Scalar(0.21049999999999999) * _tmp8 - Scalar(0.0025999999999999869) * _tmp9);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym