#ifndef SP_COMMONOBJECT_H__
#define SP_COMMONOBJECT_H__

#include <spirit/Types/spTypes.h>

/// Any spirit object should only have instances of spirit classes. like
/// spphysics, spgui
class spCommonObject {
 public:
  virtual void SetPose(const spPose& pose) = 0;
  virtual const spPose& GetPose() = 0;
  virtual void SetColor(const spColor& color) = 0;
  virtual const spColor& GetColor() = 0;

  bool IsDynamic();

  spCommonObject();
  ~spCommonObject();
  void SetPhyIndex(int index);
  void SetGuiIndex(int index);
  int GetGuiIndex();
  int GetPhyIndex();
  void Create();
  void Delete();
  bool HasChangedGui();
  bool HasChangedPhy();
  spObjectType GetObjecType();
  void SetPhyUpdated();
  void SetGuiUpdated();
  bool IsGuiModifiable();
  bool NeedsClampToSurface();
  void SetClamped();
  void SetRolligFriction(double roll_fric);
  void SetFriction(double fric);
  double GetRollingFriction();
  double GetFriction();


 protected:
  int index_phy_;
  int index_gui_;
  bool obj_guichanged_;
  bool obj_phychanged_;
  bool obj_clamptosurface_;
  spObjectType object_type_;
  bool modifiable_gui_;
  double mass_;
  double rolling_friction;
  double friction;
};

#endif  //  SP_COMMONOBJECT_H__
