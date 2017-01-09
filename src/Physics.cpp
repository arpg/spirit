#include <spirit/Physics.h>

Physics::Physics(){

}

Physics::~Physics(){

}

void Physics::Create(const spPhyEngineType& phy_type) {
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
  if(!phyworld_) {
    SPERROREXIT("phyworld_ object has not been created yet.");
  }
  phyworld_->AddNewPhyObject(obj);
}

void Physics::Iterate(Objects& objects, double sim_sec) {
#warning "UpdatePhyObjectsFromSpirit takes about 100us and needs to be optimized"
  phyworld_->UpdatePhyObjectsFromSpirit(objects);
  phyworld_->ClampObjectsToSurfaces(objects);
  phyworld_->UpdatePhyObjectsFromSpirit(objects);
//  spTimestamp t = spGeneralTools::Tick();
  phyworld_->StepPhySimulation(sim_sec);
//  std::cout << "time was " << spGeneralTools::Tock_us(t) << std::endl;
  phyworld_->UpdateSpiritObjectsFromPhy(objects);
}
