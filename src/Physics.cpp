#include <spirit/Physics.h>

Physics::Physics(){

}

Physics::~Physics(){

}

void Physics::Create(const spPhyEngineType phy_type) {
  switch(phy_type) {
    case spPhyEngineType::PHY_NONE:
      std::cout << "Initializing spirit without PhysicsEngine" << std::endl;
      break;
    case spPhyEngineType::PHY_BULLET:
      phyworld_.reset();
      phyworld_ = std::make_shared<spBulletWorld>();
      break;
    default:
      std::cerr << "Wrong type of PhysicsEngine has been selected" << std::endl;
  }
  phyworld_->InitEmptyDynamicsWorld();
}

int Physics::AddBox(spBox &box){
  return phyworld_->AddBox(box);
}

int Physics::AddSphere(spSphere &sphere) {
  return phyworld_->AddSphere(sphere);
}

int Physics::AddCar(spCarParamseters &car_params) {
  return phyworld_->AddCar(car_params);
}
