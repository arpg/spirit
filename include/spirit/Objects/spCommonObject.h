#ifndef SP_COMMONOBJECT_H__
#define SP_COMMONOBJECT_H__

#include <spirit/spGeneralTools.h>

/// Any spirit object should only have instances of spirit classes. like
/// spphysics, spgui
class spCommonObject {
 public:
  virtual void SetPose(const spPose& pose) = 0;
  virtual const spPose& GetPose() = 0;
  virtual void SetColor(const spColor& color) = 0;
  virtual void SetMass(double mass) = 0;
  virtual double GetMass() = 0;

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

 protected:
  int index_phy_;
  int index_gui_;
  bool obj_guichanged_;
  bool obj_phychanged_;
  spObjectType object_type_;
};

#endif  //  SP_COMMONOBJECT_H__
