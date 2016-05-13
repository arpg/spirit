#ifndef SP_BULLETRIGIDBODY_H__
#define SP_BULLETRIGIDBODY_H__

#include <spirit/Physics/spPhysicsRigidBody.h>

class spBulletRigiBody: public spPhysicsRigidBody {
public:
  spBulletRigiBody();
  ~spBulletRigiBody();

  bool CreateEmptyWorld();
  int AddShape(const Physhape shape, const std::vector<double>& dimensions, const double weight,const std::vector& pose);

};

#endif // SP_BULLETRIGIDBODY_H__
