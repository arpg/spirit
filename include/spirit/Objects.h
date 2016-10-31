#ifndef OBJECTS_H__
#define OBJECTS_H__

#include <spirit/Types/spTypes.h>
#include <vector>
#include <spirit/Objects/spCommonObject.h>
#include <spirit/Objects/spBox.h>
#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spAWSDCar.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>

class Objects {
public:
  Objects();
  ~Objects();
  int CreateBox(const spPose& pose, const spBoxSize& size, const double mass,const spColor& color);
  int CreateWaypoint(const spPose& pose, const spColor& color);
  int CreateVehicle(const spVehicleConstructionInfo& vehicle_info);
  int CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color);
  void RemoveObj(int obj_index);
  int GetNumOfObjects();
  spCommonObject& GetObject(int obj_index);

private:
  std::vector<std::shared_ptr<spCommonObject>> objects_;
};

#endif  //  OBJECTS_H__
