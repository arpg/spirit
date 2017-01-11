#ifndef SP_BULLETWORLD_H__
#define SP_BULLETWORLD_H__




class spBulletWorld: public spPhysicsWorld {


public:
  spBulletWorld();
  ~spBulletWorld();

  bool InitEmptyDynamicsWorld();
  void AddNewPhyObject(spCommonObject& sp_obj);
  void UpdatePhyObjectsFromSpirit(Objects& spobj);
  void StepPhySimulation(double step_time);
  void UpdateSpiritObjectsFromPhy(Objects& spobjects);

private:
  void UpdateBulletBoxObject(spBox& source_obj, btRigidBody* dest_obj);
  void UpdateBulletVehicleObject(spVehicle& source_obj, btRigidBody* dest_obj);
  btRigidBody* CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape);
  btTransform& spPose2btTransform(const spPose& pose, double btworld_scale);
  spPose& btTransform2spPose(const btTransform& tr, double btworld_scale_inv);
  btRigidBody* CreateBulletVehicleObject(spVehicle& source_obj);
  btRigidBody* CreateBulletBoxObject(spBox& source_obj);
  void ClampObjectsToSurfaces(Objects &spobj);

};

#endif // SP_BULLETWORLD_H__
