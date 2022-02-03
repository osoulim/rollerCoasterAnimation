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

	modelling::vec3f getMaxPoint(modelling::HermiteCurve const &curve,
								 modelling::ArcLengthTable const &arcLengthTable) {
		auto maxPoint = curve(0);
		for(auto &u: arcLengthTable) {
			auto point = curve(u);
			if (point.y > maxPoint.y) {
				maxPoint = point;
			}
		}
		return maxPoint;
	}

	modelling::vec3f getMinPoint(modelling::HermiteCurve const &curve,
								 modelling::ArcLengthTable const &arcLengthTable) {
		auto minPoint = curve(0);
		for(auto &u: arcLengthTable) {
			auto point = curve(u);
			if (point.y < minPoint.y) {
				minPoint = point;
			}
		}
		return minPoint;
	}

	float getDeltaSpeed(modelling::vec3f point, modelling::vec3f lastPoint, float scaleFactor) {
		float delta_h = lastPoint.y - point.y;
		if (delta_h >= 0) {
			return std::sqrt(delta_h * scaleFactor);
		}
		return -std::sqrt(-delta_h * scaleFactor);

	}

	float getEnoughSpeed(modelling::vec3f point, modelling::vec3f maxPoint, float scaleFactor) {
		return getDeltaSpeed(point, maxPoint, scaleFactor);
	}

} //end of namespace utils