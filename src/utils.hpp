#pragma once

#include "hermite_curve.hpp"
#include "arc_length_parameterize.hpp"


namespace utils {
	modelling::vec3f
	getInterpolatedPoint(const modelling::HermiteCurve &curve, const modelling::ArcLengthTable &arcLengthTable,
						 float delta_s, float s) {
		auto curve_p = curve(arcLengthTable.nearestValueTo(s));
		auto curve_q = curve(arcLengthTable.nextValueTo(s));
		float index = std::floor(s / delta_s);
		return curve_p + ((s - index * delta_s) / delta_s) * (curve_q - curve_p);
	}

	modelling::vec3f getNormalOfPoint(modelling::HermiteCurve const &curve,
									  modelling::ArcLengthTable const &arcLengthTable,
									  float speed,
									  float arcLength,
									  float delta_s, float s) {
		float lastPointS = s - delta_s * 20, nextPointS = s + delta_s * 20;
		if (lastPointS < 0) {
			lastPointS += arcLength;
		}
		if (nextPointS > arcLength) {
			nextPointS -= arcLength;
		}
		auto lastPoint = getInterpolatedPoint(curve, arcLengthTable, delta_s, lastPointS);
		auto point = getInterpolatedPoint(curve, arcLengthTable, delta_s, s);
		auto nextPoint = getInterpolatedPoint(curve, arcLengthTable, delta_s, nextPointS);
		auto N = speed * speed * (nextPoint - point * 2.f + lastPoint) / (delta_s * delta_s) - modelling::vec3f {0.f, -10.0f , 0.f};
		return N;
//		return -glm::normalize(N);
	}

	modelling::vec3f getTangentOfPoint(modelling::HermiteCurve const &curve,
									   modelling::ArcLengthTable const &arcLengthTable,
									   float arcLength,
									   float delta_s, float s) {
		float nextPointS = s + delta_s;
		if (nextPointS > arcLength) {
			nextPointS -= arcLength;
		}
		auto point = getInterpolatedPoint(curve, arcLengthTable, delta_s, s);
		auto nextPoint = getInterpolatedPoint(curve, arcLengthTable, delta_s, nextPointS);
		return glm::normalize(nextPoint - point);
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

	float getDeltaSpeed(modelling::vec3f point, modelling::vec3f lastPoint) {
		float delta_h = lastPoint.y - point.y;
		if (delta_h >= 0) {
			return std::sqrt(2 * 10 * delta_h);
		}
		return -std::sqrt(2 * 10 * -delta_h);

	}

	float getEnoughSpeed(modelling::vec3f point, modelling::vec3f maxPoint) {
		return getDeltaSpeed(point, maxPoint);
	}



} //end of namespace utils