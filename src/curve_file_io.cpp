#include "curve_file_io.hpp"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

namespace modelling {

// HELPER FUNCTIONS because GLM does not provide them (should be in another
// place, but are acceptable in .cpp (local to this translation unit)
std::istream &operator>>(std::istream &in, glm::vec3 &vec) {
  in >> vec.x >> vec.y >> vec.z;
  return in;
}

std::ostream &operator<<(std::ostream &out, glm::vec3 const &vec) {
  out << vec.x << ' ' << vec.y << ' ' << vec.z;
  return out;
}

std::istream &operator>>(std::istream &in, HermiteCurve::control_point_t &cp) {
  in >> cp.position >> cp.tangent;
  return in;
}

std::ostream &operator<<(std::ostream &out,
                         HermiteCurve::control_point_t const &cp) {
  out << cp.position << ' ' << cp.tangent;
  return out;
}

void saveHermiteCurveToFile(const HermiteCurve &curve,
                            const std::string &filePath) {
  std::ofstream file(filePath);
  for (auto const &cp : curve.controlPoints()) {
    file << cp << '\n';
  }
  file.close();
}

void saveHermiteCurveTo_OBJ_File(const HermiteCurve& curve,
    const std::string& filePath) {
    std::ofstream file(filePath);
    for (auto const& cp : curve.controlPoints()) {
        file << cp.position << '\n';
    }
    file.close();
}

std::optional<HermiteCurve>
readHermiteCurveFromFile(std::string const &filePath) {
  using std::istreambuf_iterator;
  using std::string;
  using std::stringstream;

  HermiteCurve::control_points cps;
  std::ifstream file(filePath);

  if (!file) {
    std::cerr << "Unable to open file " << filePath << '\n';
    return std::nullopt;
  }

  string line;
  size_t index;
  stringstream ss(std::ios_base::in);
  size_t lineNum = 0;

  while (getline(file, line)) {
    ++lineNum;

    // remove comments
    index = line.find_first_of("#");
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    // removes leading/tailing junk
    line.erase(0, line.find_first_not_of(" \t\r\n\v\f"));
    index = line.find_last_not_of(" \t\r\n\v\f") + 1;
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    if (line.empty()) {
      continue; // empty or commented out line
    }

    ss.str(line);
    ss.clear();

    HermiteCurve::control_point_t cp;
    ss >> cp;

    if (!ss) {
      std::cerr << "Error read file: " + line +
                       " (line: " + std::to_string(lineNum) + ")";
    } else {
      cps.push_back(cp);
    }
  }
  file.close();

  return HermiteCurve(cps);
}

std::optional<HermiteCurve>
readHermiteCurveFrom_OBJ_File(std::string const &filePath) {
  using std::istreambuf_iterator;
  using std::string;
  using std::stringstream;

  HermiteCurve::control_points cps;
  std::ifstream objFile(filePath);

  if (!objFile) {
    std::cerr << "Unable to open file " << filePath << '\n';
    return std::nullopt;
  }

  string line;
  size_t index;
  stringstream ss(std::ios_base::in);
  size_t lineNum = 0;

  while (getline(objFile, line)) {
    ++lineNum;

    // remove comments
    index = line.find_first_of("#");
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    // removes leading/tailing junk
    line.erase(0, line.find_first_not_of(" \t\r\n\v\f"));
    index = line.find_last_not_of(" \t\r\n\v\f") + 1;
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    if (line.empty()) {
      continue; // empty or commented out line
    }

    if (line.front() != 'v') // only accept vertices (control points)
      continue;

    ss.str(line);
    ss.clear();

    char isV; // stupid but meh
    ss >> isV;

    HermiteCurve::control_point_t cp;
    ss >> cp.position;

    if (!ss) {
      std::cerr << "Error read file: " + line +
                       " (line: " + std::to_string(lineNum) + ")";
    } else {
      cps.push_back(cp);
    }
  }
  objFile.close();

  // set tangets
  cps = modelling::calculateCatmullRomTangents(cps);

  return HermiteCurve(cps);
}

} // namespace modelling
