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
  void SetGuiIndex(int index);
  int GetGuiIndex();
  void Create();
  void Delete();
  bool HasChangedGui();
  bool HasChangedPhy();
  spObjectType GetObjecType();
  void SetGuiUpdated();
  bool IsGuiModifiable();
  bool NeedsClampToSurface();
  void SetClamped();
  btRigidBody* GetRigidbody();

 protected:
  int index_gui_;
  bool obj_guichanged_;
  bool obj_clamptosurface_;
  spObjectType object_type_;
  bool modifiable_gui_;
  double mass_;
  btRigidBody* rigid_body_;
//  btAlignedObjectArray<btCollisionShape*>*	collisionShapes_;
  btRigidBody* CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape);
  btTransform& spPose2btTransform(const spPose& pose);
  spPose& btTransform2spPose(const btTransform& tr);
};

#endif  //  SP_COMMONOBJECT_H__
