#pragma once

#include "hermite_curve.hpp"
#include "arc_length_parameterize.hpp"


namespace utils {
	modelling::vec3f getInterpolatedPoint(
			modelling::HermiteCurve const &curve,
			modelling::ArcLengthTable const &arcLengthTable,
			float delta_s, float s) {
		auto curve_p = curve(arcLengthTable.nearestValueTo(s));
		auto curve_q = curve(arcLengthTable.nextValueTo(s));
		float index = std::floor(s / delta_s);
		return curve_p + ((s - index * delta_s) / delta_s) * (curve_q - curve_p);
	}
} //end of namespace utils