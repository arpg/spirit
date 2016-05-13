#ifndef SP_PHYSICSRIGIDBODY_H__
#define SP_PHYSICSRIGIDBODY_H__

enum spRigidBodyType{BOX,SPHERE,};

class spPhysicsRigidBody {
public:
  spPhysicsRigidBody(spRigidBodyType rigidbody_type);
  virtual void SetRigidBodyParams(...) = 0;
  virtual void GetRigidBodyPointer(...) = 0;
};

#endif // SP_PHYSICSRIGIDBODY_H__
