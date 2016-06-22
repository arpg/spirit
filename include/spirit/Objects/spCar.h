#ifndef SP_CAR_H__
#define SP_CAR_H__

#include <spirit/Objects/spCommonObject.h>
#include <vector>
#include <spirit/Objects/spWheel.h>

// 1 - Geometric center of the chassis box will be the center of car object and
// any local transformation will be with respect to coordinate system in this
// center.
// 2 - every transformation is in global coordinates unless it has name "Local"
// in its fucntion name.
class spCar : public spCommonObject {
 public:
  spCar(int num_wheels);
  ~spCar();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);

  const spPose& GetChassisPose();

  int GetNumberOfWheels();
  spWheel* GetWheel(int index);

  double GetRollInfluence();
  void SetRollInfluence(double roll_inf);

  double GetChassisMass();
  void SetChassisMass(double mass);

  const spTranslation& GetWheelOrigin(int index);

  const spBoxSize& GetChassisSize();
  void SetChassisSize(const spBoxSize& dim);

  const spTranslation& GetLocalCOG();
  void SetLocalCOG(const spTranslation& tr);

 protected:
  void SetWheelOrigin(int index, const spTranslation& tr);

 private:
  std::vector<std::shared_ptr<spWheel>> wheel_;
  spPose pose_;        // this pose will represent geometric center of the car
  spTranslation cog_;  // center of gravity
  spColor color_;
  spBoxSize chassis_size_;
  spTranslation cog_local_;
  // Car parameters
  double chassis_mass_;  // in kg
  double roll_influence_;
};

#endif  //  SP_BOX_H__
