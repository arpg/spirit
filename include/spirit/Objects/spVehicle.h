#ifndef SP_VEHICLE_H__
#define SP_VEHICLE_H__

#include <spirit/Objects/spCommonObject.h>
#include <vector>
#include <spirit/Objects/spWheel.h>

// 1 - Geometric center of the chassis box will be the center of vehicle object and
// any local transformation will be with respect to coordinate system in this
// center.
// 2 - every transformation is in global coordinates unless it has name "Local"
// in its fucntion name.
class spVehicle : public spCommonObject {
 public:
  spVehicle(const spVehicleConstructionInfo& vehicle_info, btDiscreteDynamicsWorld* dyn_world, btAlignedObjectArray<btCollisionShape*>& col_shapes);
  ~spVehicle();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();
  int GetNumberOfWheels();
  spWheel* GetWheel(int index);
  double GetChassisMass();
  void SetChassisMass(double mass);
  const spPose& GetWheelOrigin(int index);
  const spBoxSize& GetChassisSize();
  const spPose& GetLocalCOG();
  void SetVelocity(const spVelocity& chassis_vel);
  const spStateVec& GetStateVecor();  // returns [x,y,z,q1,q2,q3,q4,x_d,y_d,z_d,p_d,q_d,r_d]
  void SetClampToSurfaceFlag();
  const spLinVel& GetLinVel();
  void SetLinVel(const spLinVel& vel);
  const spRotVel& GetRotVel();
  void SetRotVel(const spRotVel& vel);

 private:
  void MoveWheelsToAnchors(const spPose& chasis_pose);
  std::vector<std::shared_ptr<spWheel>> wheel_;
  spPose pose_;        // this pose will represent geometric center of the car
  spStateVec statevec_;  // this should be updated from phy engine only
  spPose cog_local_;  // center of gravity
  spColor color_;
  spRotVel rot_vel;
  spLinVel lin_vel;
  spBoxSize chassis_size_;
  void SetWheelOrigin(int index, const spPose& pose);
  void SetWheelAnchor(int index, const spTranslation& tr);
};

#endif  //  SP_VEHICLE_H__
