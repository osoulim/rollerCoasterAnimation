#include "panel.h"

#include <array>

namespace panel {

// default values
bool showPanel = true;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

// visualization
int curveSamples = 200;

// loading
bool rereadControlPoints = false;
std::string controlPointsFilePath = "./roller_coaster.obj";

// animation
bool play = false;

// reset
bool resetView = false;

void updateMenu() {
  using namespace ImGui;

  giv::io::ImGuiBeginFrame();

  if (showPanel && Begin("panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
    if (BeginMenuBar()) {
      if (BeginMenu("File")) {
        if (MenuItem("Close", "(P)")) {
          showPanel = false;
        }
        // add more if you would like...
        ImGui::EndMenu();
      }
      EndMenuBar();
    }

    Spacing();
    if (CollapsingHeader("Background Color")) { // Clear
      ColorEdit3("Clear color", (float *)&clear_color);
    }

    Spacing();
    if (CollapsingHeader("Visualization")) {
      SliderInt("Samples", &curveSamples, 5, 1000);
    }

    Spacing();
    if (CollapsingHeader("Loading control points")) {
      // local string buffer
      static std::array<char, 64> buffer;

      InputText("(OBJ) file", buffer.data(), buffer.size());
      rereadControlPoints = Button("Load");
      if (rereadControlPoints) {
        controlPointsFilePath = buffer.data();
      }
    }

    Spacing();
    Separator();
    if (Button("Play/Pause")) {
      play = !play;
    }

    Spacing();
    Separator();
    resetView = Button("Reset view");

    Spacing();
    Separator();
    Text("Application average %.3f ms/frame (%.1f FPS)",
         1000.0f / GetIO().Framerate, GetIO().Framerate);

    End();
  }
  giv::io::ImGuiEndFrame();

  
}

} // namespace panel
