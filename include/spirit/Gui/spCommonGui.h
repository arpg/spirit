#ifndef SP_COMMONGUI_H__
#define SP_COMMONGUI_H__

#include <spirit/Objects.h>

/// This is a abstract class which defines generic graphical interface for
/// spirit. One can add a different graphical interface with implementing this
/// abstract class.
class spCommonGUI {
 public:
  virtual void InitGui() = 0;
  virtual bool ShouldQuit() = 0;
  virtual void Refresh() = 0;
  virtual void CheckKeyboardAction() = 0;
  virtual void AddBox(spBox& box) = 0;
  virtual void AddVehicle(spVehicle& vehicle) = 0;
  virtual void UpdateGuiObjects(Objects& spobjects) = 0;
};

#endif  // SP_COMMONGUI_H__
