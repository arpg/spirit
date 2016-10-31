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
//  void LocalPlanTest(spWaypoint& w1, spWaypoint& w2);
private:
  void InitCarPool(int num_cars);
  std::vector<Physics> pool_physics_vec_;
  std::vector<Objects> pool_objects_vec_;
  std::vector<std::thread> pool_threads_vec_;

  Gui gui_;
  spSettings user_settings_;
  Physics physics_;
  Objects objects_;
  int obj_gnd_index;
  int obj_box_index;
  int obj_car_index;
  int obj_waypoint_index1;
  int obj_waypoint_index2;
  int obj_curve_index;

};

#endif  //SPIRIT_H__
