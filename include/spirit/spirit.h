#ifndef SPIRIT_H__
#define SPIRIT_H__

#include <spirit/Gui.h>
#include <spirit/Physics.h>
#include <spirit/spSettings.h>
#include <spirit/spGeneralTools.h>
#include <spirit/Objects/spBox.h>

class spirit{
public:
  spirit(spSettings& user_settings);
  ~spirit();
  void Create();
  bool ShouldRun();
  void IterateWorld();
  void CheckKeyboardAction();
  void ScenarioWorldBoxFall();
private:
  Gui gui_;
  spSettings user_settings_;
  Physics physics_;
  void IterateGraphics();
  void IteratePhysics();

  // for test purposes
  spBox groundbox;
  spBox box;

};

#endif  //SPIRIT_H__
