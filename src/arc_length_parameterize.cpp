#include "arc_length_parameterize.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include <iostream>

namespace modelling {

//
// public interface
//

ArcLengthTable::ArcLengthTable(float deltaS) : m_delta_s(deltaS) {

}

float ArcLengthTable::nearestValueTo(float s) const {
  auto index = indexAt(s);
  return m_values[index];
}

float ArcLengthTable::operator()(float s) const { return nearestValueTo(s); }

void ArcLengthTable::addNext(float u) { m_values.push_back(u); }

void ArcLengthTable::reserve_memory(size_t n) { m_values.reserve(n); }

float ArcLengthTable::deltaS() const { return m_delta_s; }

size_t ArcLengthTable::size() const { return m_values.size(); }

float ArcLengthTable::length() const { return size() * deltaS(); }

ArcLengthTable::iterator ArcLengthTable::begin() {
  return std::begin(m_values);
}

ArcLengthTable::iterator ArcLengthTable::end() { return std::end(m_values); }

ArcLengthTable::const_iterator ArcLengthTable::begin() const {
  return std::begin(m_values);
}

ArcLengthTable::const_iterator ArcLengthTable::end() const {
  return std::end(m_values);
}

//
// private functions
//

size_t ArcLengthTable::indexAt(float s) const {
  return static_cast<size_t>(std::floor(s / m_delta_s));
}

//
// free function interface
//

ArcLengthTable calculateArcLengthTable(HermiteCurve const &curve, float delta_s,
                                       float delta_u) {

  //
  // TODO (Students): calculate the arc-length parameterization...
  //
  assert(delta_u > 0.f);
	ArcLengthTable table(delta_s);
	float arc_length = modelling::arcLength(curve, delta_u);
	table.addNext(0);
	float delta_s_acc = 0.0, u = 0.0;
	while (u <= 1) {
		delta_s_acc += length(curve(u + delta_u) - curve(u));
		u += delta_u;
		if (delta_s_acc >= delta_s) {
			table.addNext(u);
			delta_s_acc = 0;
		}
	}
	// TODO remove this tof!
	while (table.size() <= arc_length / delta_s) {
		table.addNext(u);
	}
  return table;
}

} // namespace modelling
