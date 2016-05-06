#ifndef SETTINGS_H__
#define SETTINGS_H__

// This header should only include stdlib headers
#include <stdio.h>
#include <memory>
#include <iostream>

#define SPIRITGUI_PARAM_FILE     "../config_files/PangolinGuiParams.cfg"
#define SPIRITGUI_WINDOW_NAME    "Spirit GUI"

namespace spirit {
// Available gui types
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};

class Settings {
public:
  Settings();
  ~Settings();
  void SetGuiType( const spGuiType& gui_type);
  spGuiType GetGuiType();
  bool HasGui( void );
  void SetGuiParamFilePath(std::string& file_path);
  std::string GetGuiParamFilePath();

private:
  spGuiType gui_type_;
  std::string gui_param_file_path_;
};
}
#endif  // SETTINGS_H__
