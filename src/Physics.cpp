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
      std::cerr << "Selected PhysicsEngine is not supported by spirit." << std::endl;
  }
  phyworld_->InitEmptyDynamicsWorld();
}

void Physics::AddObject(spCommonObject &obj) {
  phyworld_->AddNewPhyObject(obj);
}

void Physics::Iterate(Objects& objects) {
  // step 100ms (0.1s)
  phyworld_->UpdatePhyObjectsFromSpirit(objects);
  phyworld_->StepPhySimulation(1);
  phyworld_->UpdateSpiritObjectsFromPhy(objects);
}
