#ifndef GUI_H__
#define GUI_H__

#include <spirit/Gui/spCommonGui.h>
#include <spirit/Gui/spPangolinScenegraphGui.h>
#include <spirit/spGeneralTools.h>

class Gui {
public:
  Gui();
  ~Gui();
  void Create(const spGuiType gui_type);
  bool ShouldQuit();
  void Refresh();
  void CheckKeyboardAction();
private:
   std::shared_ptr<spCommonGUI> gui_;
};

#endif  // GUI_H__
