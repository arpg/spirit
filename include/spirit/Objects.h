#ifndef OBJECTS_H__
#define OBJECTS_H__

#include <spirit/spGeneralTools.h>
#include <vector>
#include <spirit/Objects/spCommonObject.h>
#include <spirit/Objects/spBox.h>

class Objects {
public:
  Objects();
  ~Objects();
  int CreateBox(const spPose& pose, const spBoxSize& size, const double mass,const spColor& color);
  void EditObjMass(int obj_index, const double des_mass);
  void EditObjPose(int obj_index, const spPose& des_pose);
  void RemoveObj(int obj_index);
  int GetNumOfObjects();
  spCommonObject& GetObject(int obj_index);

private:
  std::vector<std::shared_ptr<spCommonObject>> objects_;
};

#endif  //  OBJECTS_H__
