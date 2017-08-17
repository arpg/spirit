#ifndef SPIRIT_H__
#define SPIRIT_H__

#include <spirit/Gui.h>
#include <spirit/Controllers/spPID.h>
#include <spirit/spSettings.h>
#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>
#include <functional>
#include <spirit/Planners/spLocalPlanner.h>
#include <spirit/Controllers/spMPC.h>
#include <iomanip>

class spirit{
public:
  spirit(spSettings& user_settings);
  ~spirit();
  bool ShouldRun();
  void IterateWorld();
  void CheckKeyboardAction();
  void ScenarioWorldBoxFall();
  void DummyTests();
  void SenarioCostSurf();
  void ScenarioPIDController();
  void CalcLocalPlannerJacobian();
  void SenarioCeresTest();
  void SenarioTrajectoryTest();
  void SenarioStateInitialization();
  void SenarioControllerTest();
  void multithreadtest();
  void zibil();

private:
  void InitCarPool(int num_cars);
  std::vector<Objects> pool_objects_vec_;
  std::vector<std::thread> pool_threads_vec_;
  Gui gui_;
  spSettings user_settings_;
  Objects objects_;
  spObjectHandle obj_gnd_index;
  spObjectHandle obj_box_index;
  spObjectHandle obj_car_index;
  spObjectHandle obj_cars_index[9];
  spObjectHandle obj_waypoints_index[9];
  spObjectHandle obj_waypoint_index0;
  spObjectHandle obj_waypoint_index1;
  spObjectHandle obj_waypoint_index2;
  spObjectHandle obj_linestrip_index;
  spVehicleConstructionInfo car_param;
};

#endif  //SPIRIT_H__
