#ifndef COMMON_OBJ_
#define COMMON_OBJ_

#include <string>
#include <Eigen/Eigen>
#include <SceneGraph/SceneGraph.h>
#include <CarPlanner/BulletCarModel.h>

class SpiritCommonObj {
 public:
  SpiritCommonObj()
      : dMin_(DBL_MAX, DBL_MAX, DBL_MAX), dMax_(DBL_MIN, DBL_MIN, DBL_MIN) {}
  ~SpiritCommonObj() {}
  virtual int AddObj(Eigen::Vector6d T_w_a) = 0;
  virtual int NumOfObjs() = 0;
  virtual int DelObj(int objnum) = 0;
  btVector3 dMin_;
  btVector3 dMax_;
};

#endif  // COMMON_OBJ_
