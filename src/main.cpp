#include "givio.h"
#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "panel.h"
#include "picking_controls.h"
#include "turntable_controls.h"

#include "arc_length_parameterize.hpp"
#include "curve_file_io.hpp"
#include "hermite_curve.hpp"

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
//  auto curve = initialCurve();
  //To load the arc length parameterized curve (only worth part marks):
  auto curve = modelling::readHermiteCurveFrom_OBJ_File("./models/roller_coaster_3.obj").value();
  
  // control points
  auto cp_geometry = controlPointsGeometry(curve);
  auto cp_style = GL_Line(Width(15.), Colour(0.5, 1.0, 0.0));
  auto cp_render = createRenderable(cp_geometry, cp_style);

  // instanced monkey --
            // In the place for a cart
  auto sue_geometry = Mesh(Filename("./models/monkey.obj"));
  auto sue_style =
      Phong(Colour(1.f, 1.f, 0.f), LightPosition(100.f, 100.f, 100.f));
  auto sue_renders = createInstancedRenderable(sue_geometry, sue_style);

  // geometry for curve
  auto track_geometry = sampleTrack(curve, 500);
  auto track_style = GL_Line(Width(15.), Colour(0.2, 0.7, 1.0));
  auto track_render = createRenderable(track_geometry, track_style);

//  size_t controlPointIndex = 0;
  float s = 0.f;
  float delta_s = 0.2f, delta_u = 0.00001f;
  float arc_length = modelling::arcLength(curve, delta_u);
  modelling::ArcLengthTable arcLengthTable = modelling::calculateArcLengthTable(curve, delta_s, delta_u);
  std::cout<<arc_length<<" "<<arcLengthTable.size()<<std::endl;

  //
  // panel update
  //
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
		s = 0.f;
	    delta_s = 0.2f;
		delta_u = 0.00001f;
	  	arc_length = modelling::arcLength(curve, delta_u);
		arcLengthTable = modelling::calculateArcLengthTable(curve, delta_s, delta_u);
      }
    }

    if (panel::resetView) {
      view.camera.reset();
    }
  };

  //
  // main loop
  // To load a model place it in the "models" directory, build, then type "./models/[name].obj" and press load.
  // If the above does not work, manually copy the model into the models folder in the output directory, then try and reload the track
  // Backspace isnt enabled when the panel is over the window, please move the panel off the window to backspace.
  mainloop(std::move(window), [&](float) {
    //
    // updates from panel
    //
    applyPanel();
	//
    // simulation
    //
    if (panel::play) {
      // increment and wrap
//      ++controlPointIndex;
//      if (controlPointIndex >= curve.controlPoints().size())
//        controlPointIndex = 0;

      s += delta_s;
      if (s >= arc_length)
        s = 0.f;
    }

    auto p = vec3f{0.f, 0.f, 0.f};
    auto M = translate(mat4f{1.f}, p);
    addInstance(sue_renders, M);

//    auto cp = curve.controlPoints()[controlPointIndex];
//    M = glm::translate(mat4f{1.f}, cp.position);
//    addInstance(sue_renders, M);


    // Currently sampling directly out of the curve.
    //For full marks, the ArcLengthTable (or an equivalent) 
    //needs be completed and used for proper traversal of the curve.
//	std::cout<<s<<std::endl;
    auto curve_p = curve(arcLengthTable.nearestValueTo(s));
    M = scale(translate(mat4f{1.f}, curve_p), vec3f{0.75});
    addInstance(sue_renders, M);

    //
    // render
    //
    auto color = panel::clear_color;
    glClearColor(color.x, color.y, color.z, color.z);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    view.projection.updateAspectRatio(window.width(), window.height());
   
    draw(cp_render, view);

    draw(sue_renders, view);

    draw(track_render, view);

  });

  return EXIT_SUCCESS;
}
