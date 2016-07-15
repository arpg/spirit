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

void Physics::AddObject(spCommonObject &obj) {
  phyworld_->AddNewPhyObject(obj);
}

void Physics::Iterate(Objects& objects) {
  // simulate in 60Hz
//  phyworld_->UpdatePhyObjectsFromSpirit(objects);
  phyworld_->StepPhySimulation(1.f/100);
  phyworld_->UpdateSpiritObjectsFromPhy(objects);
}
