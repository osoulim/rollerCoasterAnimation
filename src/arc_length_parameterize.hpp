/**
  Abrtaction around Arc-Length Table built by arc-length parameterization
  function below:
          calculateArcLengthTable(...)

          table[0] = u_0
          table[1] = u_1
          ...
          table[N] = u_N

  Each entry in table is a parameter into a curve:
          C(u): u -> vec3f
  However, each subsequent value in the table is such that:
          distance(C(u_i+1), C(u_i)) aprrox. delta S
  **/

#pragma once

#include "hermite_curve.hpp"
#include <glm/glm.hpp>
#include <vector>
namespace modelling {

using vec3f = glm::vec3;

class ArcLengthTable {
public: // types
  using table_t = std::vector<float>;
  using iterator = table_t::iterator;
  using const_iterator = table_t::const_iterator;

public: // interface
  ArcLengthTable() = default;
  explicit ArcLengthTable(float deltaS);

  // accessors
  float nearestValueTo(float s) const;
  float nextValueTo(float s) const;
  float operator()(float s) const;

  size_t size() const;
  float deltaS() const;
  float length() const;

  // mutators
  void addNext(float t);
  void reserve_memory(size_t n);

  // iterator helpers
  iterator begin();
  iterator end();
  const_iterator begin() const;
  const_iterator end() const;

private: // functions
  size_t indexAt(float s) const;

private: // member variables
  table_t m_values;
  float m_delta_s = 1.f;
};

// free function interface
ArcLengthTable calculateArcLengthTable(HermiteCurve const &curve, float delta_s,
                                       float delta_u);

} // namespace modelling
