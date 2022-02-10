#include "givio.h"
#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>
//#include <glm/gtx/string_cast.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "panel.h"
#include "picking_controls.h"
#include "turntable_controls.h"

#include "arc_length_parameterize.hpp"
#include "curve_file_io.hpp"
#include "hermite_curve.hpp"
#include "utils.hpp"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

// helper functions (maybe put into another .h/.cpp?)

PolyLine<PrimitiveType::LINE_LOOP>
controlPointsGeometry(modelling::HermiteCurve const &curve) {
	PolyLine<PrimitiveType::LINE_LOOP> geometry;

	for (auto const &cp : curve.controlPoints()) {
		geometry.push_back(Point(cp.position));
	}

	return geometry;
}

modelling::HermiteCurve initialCurve() {
	std::vector<vec3f> points = {
			{0.f, 0.f, 1.f}, {1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}, {-1.f, 0.f, 0.f}};

	// scale them
	for (auto &p : points) {
		p *= 10.f;
	}

	auto controlPoints = modelling::buildControlPoints(points);
	return modelling::HermiteCurve(controlPoints);
}

PolyLine<PrimitiveType::LINE_LOOP>
sampleTrack(modelling::HermiteCurve const &curve, size_t samplesCount) {
	PolyLine<PrimitiveType::LINE_LOOP> geometry;

	auto samples = modelling::sample(curve, samplesCount);

	for (auto const &p : samples) {
		geometry.push_back(Point(p));
	}

	return geometry;
}

//
// program entry point
//
int main(void) {
	//
	// initialize OpenGL and window
	//
	namespace givio = giv::io; // perhaps better than giv::io
	givio::GLFWContext glContext;
	glContext.glMajorVesion(4)
			.glMinorVesion(0)
			.glForwardComaptability(true)
			.glCoreProfile()
			.glAntiAliasingSamples(4)
			.matchPrimaryMonitorVideoMode();

	std::cout << givio::glfwVersionString() << '\n';

	//
	// setup window (OpenGL context)
	//
	auto window =
			glContext.makeImGuiWindow(givio::Properties()
											  .size(givio::dimensions{1000, 1000})
											  .title("Curve surfing...")
											  .glslVersionString("#version 330 core"));

	auto view = View(TurnTable(), Perspective());
	// Preset Bindings
	TurnTableControls controls(window, view.camera);

	//
	// setup simulation
	//

	//  Custom Bind keys
	// For modifying the panel go to panel.h and panel.cpp in the libs directory
	window.keyboardCommands() |
	givio::Key(GLFW_KEY_V, [&](auto) { view.camera.reset(); }) |
	givio::Key(GLFW_KEY_P, [&](auto event) {
		if (event.action == GLFW_PRESS) {
			panel::showPanel = !panel::showPanel;
		}
	});

	// initial curve --
//	auto curve = initialCurve();
	//To load the arc length parameterized curve (only worth part marks):
  	auto curve = modelling::readHermiteCurveFrom_OBJ_File("./models/roller_coaster_1.obj").value();

	// control points
	auto cp_geometry = controlPointsGeometry(curve);
	auto cp_style = GL_Line(Width(15.), Colour(0.5, 1.0, 0.0));
	auto cp_render = createRenderable(cp_geometry, cp_style);

	// instanced monkey --
	// In the place for a cart
	auto sue_geometry = Mesh(Filename("./models/cart.obj"));
	auto sue_style = Phong(Colour(1.f, 1.f, 1.f), LightPosition(100.f, 100.f, 100.f));
	auto sue_renders = createInstancedRenderable(sue_geometry, sue_style);

	auto rail_geometry = Mesh(Filename("./models/block.obj"));
	auto rail_renders = createInstancedRenderable(rail_geometry, sue_style);

	auto earth_geometry = Mesh(Filename("./models/earth.obj"));
	auto earth_renders = createInstancedRenderable(earth_geometry, sue_style);


	auto track_geometry = sampleTrack(curve, 500);
	auto track_style = GL_Line(Width(15.), Colour(0.2, 0.7, 1.0));
	auto track_render = createRenderable(track_geometry, track_style);

	float s = 0.f, delta_t = 1.f / 50.f, delta_u = 0.00001f, speed = 0.0f;
	float arc_length = modelling::arcLength(curve, delta_u);
	float delta_s = arc_length / 200;
	modelling::ArcLengthTable arcLengthTable = modelling::calculateArcLengthTable(curve, delta_s, delta_u);
	auto maxPoint = utils::getMaxPoint(curve, arcLengthTable) + vec3{0.f, 5.f, 0.f};
//	std::cout<<arc_length<<" "<<arcLengthTable.size()<<std::endl;
	std::vector<glm::mat4> rails;

	auto applyPanel = [&]() {
		if (panel::rereadControlPoints) {
			// load points
			auto loaded = modelling::readHermiteCurveFrom_OBJ_File(
					panel::controlPointsFilePath);
			if (loaded) {
				curve = loaded.value();

				// reload cps to GPU
				cp_geometry = controlPointsGeometry(curve);
				updateRenderable(cp_geometry, cp_style, cp_render);

				// reload curve to GPU
				track_geometry = sampleTrack(curve, 500);
				updateRenderable(track_geometry, track_style, track_render);

				// reset
				s = 0.f, delta_t = 1.f / 50.f, delta_u = 0.00001f, speed = 0.0f;
				arc_length = modelling::arcLength(curve, delta_u);
				delta_s = arc_length / 200;
				arcLengthTable = modelling::calculateArcLengthTable(curve, delta_s, delta_u);
				maxPoint = utils::getMaxPoint(curve, arcLengthTable) + vec3{0.f, 5.f, 0.f};

				rails.clear();
				for (float rail_s = 0; rail_s < arc_length; rail_s += delta_s/2) {
					auto rail_point = utils::getInterpolatedPoint(curve, arcLengthTable, delta_s, rail_s);
					auto m = utils::calculateMatrixOfPoint(curve, arcLengthTable, maxPoint, rail_point, arc_length, delta_s, rail_s);
					rails.emplace_back(scale(m, vec3{1 / 3.f}));
				}
			}

		}

		if (panel::resetView) {
			view.camera.reset();
		}
	};


	for (float rail_s = 0; rail_s < arc_length; rail_s += delta_s / 2) {
		auto rail_point = utils::getInterpolatedPoint(curve, arcLengthTable, delta_s, rail_s);
		auto m = utils::calculateMatrixOfPoint(curve, arcLengthTable, maxPoint, rail_point, arc_length, delta_s, rail_s);
		rails.emplace_back(scale(m, vec3{1/3.f}));
	}


	mainloop(std::move(window), [&](float) {
		applyPanel();

		for (auto const& rail_mat: rails) {
			addInstance(rail_renders,  rail_mat);
		}

		addInstance(earth_renders, glm::translate(mat4{1.f}, vec3{0.f, -20.f, 0.f}));

		auto lastPoint = utils::getInterpolatedPoint(curve, arcLengthTable, delta_s, s);
		if (panel::play) {
			s += speed * delta_t;
			if (s >= arc_length) {
				s -= arc_length;
			}
		}

		for (int i = 0; i < 3; i++) {
			float point_s = s - i * delta_s;
			auto point = utils::getInterpolatedPoint(curve, arcLengthTable, delta_s, point_s);
			auto m = utils::calculateMatrixOfPoint(curve, arcLengthTable, maxPoint, point, arc_length, delta_s, point_s, true);
			addInstance(sue_renders, m);
		}

		auto point = utils::getInterpolatedPoint(curve, arcLengthTable, delta_s, s);
		if (panel::play) {
			if (s >= arc_length * 0.75 && speed > 1e-4) {
				float dist = arc_length - s;
				speed -= (speed * speed) / (2 * dist) * delta_t;
//			} else if (speed < utils::getEnoughSpeed(point, maxPoint, delta_t)) {
//				speed += acceleration;
			} else {
				speed = utils::getEnoughSpeed(point, maxPoint);
			}
//			std::cout<<speed<<std::endl;
		}
//		speed += utils::getDeltaSpeed(point, lastPoint, delta_t);

		auto color = panel::clear_color;
		glClearColor(color.x, color.y, color.z, color.z);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		view.projection.updateAspectRatio(window.width(), window.height());
		view.camera.translate(point);
		draw(cp_render, view);

		draw(sue_renders, view);

		draw(track_render, view);

		draw(rail_renders, view);
		draw(earth_renders, view);

		view.camera.translate(-point);

	});

	return EXIT_SUCCESS;
}
