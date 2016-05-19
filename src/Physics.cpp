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

void Physics::AddBox(spBox &box){
  phyworld_->AddBox(box);
}

void Physics::AddSphere(spSphere &sphere) {
  phyworld_->AddSphere(sphere);
}

void Physics::AddCar(spCarParamseters &car_params) {
  phyworld_->AddCar(car_params);
}
