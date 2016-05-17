#ifndef WORLD_H__
#define WORLD_H__

#include <spirit/Gui.h>
#include <spirit/Physics.h>
#include <spirit/Settings.h>
#include <spirit/GeneralTools.h>

namespace spirit {

class World{
public:
  World(Settings& user_settings);
  ~World();
  void Create();
  bool ShouldRun();
  void IterateWorld();
  void CheckKeyboardAction();
  void TestWorldBoxFall();
private:
  Gui gui_;
  Settings user_settings_;
  Physics physics_;
  void IterateGraphics();
  void IteratePhysics();
};
}
#endif  //WORLD_H__
