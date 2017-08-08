#ifndef SP_BOX_H__
#define SP_BOX_H__

#include <spirit/Objects/spCommonObject.h>

class spBox : public spCommonObject {
 public:
  spBox(const spPose& pose, const spBoxSize& size, double mass, const spColor& color, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_);
  ~spBox();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();
  void SetMass(double mass);
  double GetMass();
  void SetDimensions(const spBoxSize& dims);
  spBoxSize GetDimensions();
  void SetFriction(double fric_coeff);
  double GetFriction();
  void SetRollingFriction(double fric_coeff);
  double GetRollingFriction();
  void ClampToSurface();

 private:
  spColor color_;
  // pose_ shouldn't be used directly, call GetPose() if position of object is required
  spPose pose_;
  std::shared_ptr<btCollisionShape> shape_;
};

#endif  //  SP_BOX_H__
