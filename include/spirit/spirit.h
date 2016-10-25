#ifndef SPIRIT_H__
#define SPIRIT_H__

#include <spirit/Gui.h>
#include <spirit/Physics.h>
#include <spirit/spSettings.h>
#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>
#include <spirit/Planners/spBezierPlanner.h>
#include <functional>

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
  std::vector<Physics> physics_vec_;
  std::vector<Objects> objects_vec_;
  void runthread(int ind);
  Objects objects_;

  std::thread threads_[42];

  int obj_gnd_index;
  int obj_box_index;
  int obj_car_index;
  int obj_waypoint_index1;
  int obj_waypoint_index2;
  int obj_curve_index;

};

#endif  //SPIRIT_H__
