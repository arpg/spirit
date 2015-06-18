#ifndef SPIRIT_CAR_H_
#define SPIRIT_CAR_H_

#include <CarPlanner/LocalPlanner.h>
#include <spirit/objects/CommonObj.h>

class SpiritCars : public SpiritCommonObj{
 public:
  SpiritCars(btCollisionShape* col_shape);
  ~SpiritCars();

  int AddObj(Eigen::Vector6d T_w_a);
  int NumOfObjs();
  int DelObj(int objnum);

 private:
  std::vector<std::unique_ptr<BulletCarModel>> vec_;
  btCollisionShape* collision_shape_;
  CarParameterMap default_parametes_;
};

#endif    //SPIRIT_CAR_H_
