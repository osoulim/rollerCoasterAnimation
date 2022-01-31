#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace modelling {

template <typename T> using ControlPoints = std::vector<T>;

using vec3f = glm::vec3;

class HermiteCurve {

public: // types
  struct ControlPoint {
    glm::vec3 position;
    glm::vec3 tangent;
  };

  using control_points = ControlPoints<ControlPoint>;
  using control_point_t = control_points::value_type;

public: // interface
  HermiteCurve() = default;
  explicit HermiteCurve(control_points controlPoints);

  vec3f operator()(float u) const;

  control_points const &controlPoints() const;
  control_points &controlPoints();

private: // functions
private: // member variables
  control_points m_cps;
};

// free function interface
float segmentFrom(float u, size_t segmentCount);

vec3f evaluateCubicHermite(vec3f const &positionA, vec3f const &tangentA,
                           vec3f const &positionB, vec3f const &tangentB,
                           float u);

vec3f evaluateCubicHermite(HermiteCurve::ControlPoint const &cpA,
                           HermiteCurve::ControlPoint const &cpB, float u);

vec3f evaluateCubicHermite(HermiteCurve::control_points const &cps, float u);

float arcLength(HermiteCurve const &curve, float delta_u);

std::vector<vec3f> sample(HermiteCurve const &curve, int sampleCount);

HermiteCurve::control_points buildControlPoints(std::vector<vec3f> points);

HermiteCurve::control_points
calculateCatmullRomTangents(HermiteCurve::control_points cps);

HermiteCurve::control_points
calculateCanonicalTangents(HermiteCurve::control_points cps, float c);

// helper functions
template <typename T>
T const &nextValueOrWrap(size_t index, std::vector<T> const &values) {
  if (index >= values.size() - 1)
    return values.front();
  return values[index + 1];
}

template <typename T>
T const &previousValueOrWrap(size_t index, std::vector<T> const &values) {
  if (index == 0)
    return values.back();
  return values[index - 1];
}

} // namespace modelling
