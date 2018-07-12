#ifndef GUI_H__
#define GUI_H__

#include <spirit/Gui/spCommonGui.h>
#include <spirit/Gui/spPangolinScenegraphGui.h>
#include <spirit/Objects.h>
#include <spirit/Gui/spOpenSceneGraphGui.h>

class Gui {
public:
  Gui();
  ~Gui();
  void Create(const spGuiType gui_type);
  bool ShouldQuit();
  void Refresh();
  void CheckKeyboardAction();
  void AddObject(spCommonObject& obj);
  void Iterate(Objects& spobjects);
  void RemoveObject(spCommonObject &obj);
private:
  std::shared_ptr<spCommonGUI> gui_;
};

#endif  // GUI_H__
