#include <spirit/objects/SpiritCars.h>

SpiritCars::SpiritCars(btCollisionShape* col_shape)
    : collision_shape_(col_shape) {}
SpiritCars::~SpiritCars() { delete collision_shape_; }

int SpiritCars::AddObj(Eigen::Vector6d T_w_a) {
  if (collision_shape_ == NULL) {
    std::cout << "collision shape has not been set" << std::endl;
    return -1;
  }
  std::unique_ptr<BulletCarModel> new_car(new BulletCarModel);
  // TODO(sina) : what is num_of_worlds for?
  int num_of_worlds = 1;
  new_car->Init(collision_shape_, dMin_, dMax_, default_parametes_,num_of_worlds);
  vec_.push_back(std::move(new_car));
  return vec_.size();
}

int SpiritCars::NumOfObjs() { return vec_.size(); }

int SpiritCars::DelObj(int objnum) {}
