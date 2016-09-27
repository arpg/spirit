#ifndef SPIRIT_H__
#define SPIRIT_H__

#include <spirit/Gui.h>
#include <spirit/Physics.h>
#include <spirit/spSettings.h>
#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>

class spirit{
public:
  spirit(spSettings& user_settings);
  ~spirit();
  void Create();
  bool ShouldRun();
  void IterateWorld();
  void CheckKeyboardAction();
  void ScenarioWorldBoxFall();
  void ScenarioWorldCarFall();
private:
  Gui gui_;
  spSettings user_settings_;
  Physics physics_;

  Objects objects_;

  int obj_gnd_index;
  int obj_box_index;
  int obj_car_index;
  int obj_waypoint_index;
};

#endif  //SPIRIT_H__
