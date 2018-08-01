#ifndef SPIRIT_H__
#define SPIRIT_H__

#include <spirit/Gui.h>
#include <spirit/Controllers/spPID.h>
#include <spirit/Controllers/spMPC.h>
#include <spirit/spSettings.h>
#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>
#include <functional>
#include <spirit/Planners/spLocalPlanner.h>
#include <spirit/Controllers/spMPC.h>
#include <spirit/Calibration/CandidateWindow.h>
#include <spirit/Calibration/PriorityQueue.h>
#include <spirit/Calibration/EntropyTable.h>
#include <iomanip>
#include <spirit/CarSimFunctor.h>
#include <spirit/CarSimFunctorRK4.h>
#include <spirit/BikeSimFunctorRK4.h>
#include <spirit/CarODE.h>
#include <spirit/Planners/spTrajectory.h>
#include <spirit/Planners/VehicleCeresCostFunc.h>
#include <spirit/Planners/spCirclePlanner.h>
#include <fstream>
#include <spirit/RK4.h>
#include <osgDB/ReadFile>


class spirit {
public:
  spirit(const spSettings& user_settings);
  ~spirit();
  bool ShouldRun();
  void CheckKeyboardAction();
  void ScenarioWorldBoxFall();
  void IterateWorld();
  std::shared_ptr<Objects> objects_;
  Gui gui_;
  spVehicleConstructionInfo car_param;

private:
  spSettings user_settings_;
  spObjectHandle obj_gnd_index;
  spObjectHandle obj_box_index;

};

#endif  //SPIRIT_H__
