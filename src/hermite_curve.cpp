#include "hermite_curve.hpp"

#include <algorithm> // std::transform
#include <iterator>

namespace modelling {

//
// public interface
//

HermiteCurve::HermiteCurve(HermiteCurve::control_points controlPoints)
    : m_cps(controlPoints) {}

// evaluate curve at u
vec3f HermiteCurve::operator()(float u) const {
  return evaluateCubicHermite(m_cps, u);
}

HermiteCurve::control_points const &HermiteCurve::controlPoints() const {
  return m_cps;
}

HermiteCurve::control_points &HermiteCurve::controlPoints() { return m_cps; }

//
// free functions interface
//
float segmentFrom(float u, size_t segmentCount) {
  // TODO (Students): Currently the curve is clamped to the edges.
  // One way to allow wrapping around the curve is to modify the segment 
  // detection to implicitly indicate this. Not the only way, but is one 
  // of the more elegant ways to do it
  if (u > 1.f) {
    u = 1.f;
  }
  if (u < 0.f) {
    u = 0.f;
  }

  return u * segmentCount;
}

vec3f evaluateCubicHermite(vec3f const &positionA, vec3f const &tangentA,
                           vec3f const &positionB, vec3f const &tangentB,
                           float u) {
  //
  // (Students): lerp for now, but replace with Hermite, or other...
  // Note that 0 <= u <= 1 indicating the portion travelled along the segement between points A and B.
  //
  float u3 = u * u * u, u2 = u * u;
  return positionA * (2 * u3 - 3 * u2 + 1) +
  		 tangentA * (u3 - 2 * u2 + u)  +
		 positionB * (-2 * u3 + 3 * u2) +
		 tangentB * (u3 - u2);
}

vec3f evaluateCubicHermite(HermiteCurve::ControlPoint const &cpA,
                           HermiteCurve::ControlPoint const &cpB, float u) {
  return evaluateCubicHermite(cpA.position, cpA.tangent, cpB.position,
                              cpB.tangent, u);
}

vec3f evaluateCubicHermite(HermiteCurve::control_points const &cps, float u) {
  if (u <= 0.f || u >= 1.f)
    return cps.front().position;

  auto segment = segmentFrom(u, cps.size());
  size_t index = floor(segment);

  auto cpA = cps.at(index);
  auto cpB = nextValueOrWrap(index, cps);

  u = segment - floor(segment); // move it to 0-1 in segment;

  return evaluateCubicHermite(cpA, cpB, u);
}

std::vector<vec3f> sample(HermiteCurve const &curve, int sampleCount) {
  std::vector<vec3f> samples;
  samples.reserve(sampleCount);

  float delta_u = 1.f / (sampleCount - 1);

  for (int i = 0; i < sampleCount; ++i) {
    auto u = i * delta_u;
    samples.push_back(curve(u));
  }

  return samples;
}

float arcLength(HermiteCurve const &curve, float delta_u) {
  assert(delta_u > 0.f);
  float l = 0.f;
  for (float u = 0.f; u < 1.f; u += delta_u) {
    l += length(curve(u + delta_u) - curve(u));
  }
  return l;
}

HermiteCurve::control_points buildControlPoints(std::vector<vec3f> points) {
  HermiteCurve::control_points cps;
  cps.reserve(points.size());

  // set up points
  std::transform(std::begin(points), std::end(points),
                 std::back_insert_iterator(cps), [](vec3f const &point) {
                   HermiteCurve::ControlPoint cp;
                   cp.position = point;
                   return cp;
                 });
  // setup
  cps = calculateCatmullRomTangents(cps);

  return cps;
}

HermiteCurve::control_points
calculateCatmullRomTangents(HermiteCurve::control_points cps) {
  return calculateCanonicalTangents(cps, 0.f);
}

HermiteCurve::control_points
calculateCanonicalTangents(HermiteCurve::control_points cps, float c) {
  for (size_t index = 0; index < cps.size(); ++index) {
    auto &p = cps[index];
    auto const &a = previousValueOrWrap(index, cps);
    auto const &b = nextValueOrWrap(index, cps);

    p.tangent = ((1.f - c) * 0.5f) * (b.position - a.position);
  }

  return cps;
}

} // namespace modelling
