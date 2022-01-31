#pragma once

#include <optional>
#include <string>

#include "hermite_curve.hpp"

namespace modelling {

void saveHermiteCurveToFile(modelling::HermiteCurve const &curve,
                            std::string const &filePath);

void saveHermiteCurveTo_OBJ_File(modelling::HermiteCurve const& curve,
    std::string const& filePath);

std::optional<HermiteCurve>
readHermiteCurveFromFile(std::string const &filePath);

std::optional<HermiteCurve>
readHermiteCurveFrom_OBJ_File(std::string const &filePath);

} // namespace modelling
