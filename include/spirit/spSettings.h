#ifndef SP_SETTINGS_H__
#define SP_SETTINGS_H__

// This header should only include stdlib headers
#include <stdio.h>
#include <memory>
#include <iostream>
#include <spirit/Types/spTypes.h>

#define SPIRITGUI_PARAM_FILE     "../../../config_files/PangolinGuiParams.cfg"
#define SPIRITGUI_WINDOW_NAME    "Spirit GUI "
#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480
#define UI_PANEL_WIDTH 0//200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT
#define UI_WAYPOINT_BOX_DIM 0.3   // in centimeters

class spSettings {
public:
  spSettings();
  ~spSettings();
  void SetGuiType( const spGuiType& gui_type);
  void SetPhysicsEngineType(const spPhyEngineType& phy_type);
  spGuiType GetGuiType();
  spPhyEngineType GetPhysicsEngineType();
  void SetGuiParamFilePath(std::string& file_path);
  std::string GetGuiParamFilePath();

private:
  spGuiType gui_type_;
  spPhyEngineType phy_type_;
  std::string gui_param_file_path_;
  int num_threads_;
};

#endif  // SP_SETTINGS_H__
