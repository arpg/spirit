#ifndef WORLD_H__
#define WORLD_H__

#include <spirit/Gui.h>
#include <spirit/Settings.h>
#include <spirit/GeneralTools.h>

namespace spirit {

class World{
public:
  World(Settings& user_settings);
  ~World();
  void Create();
  bool ShouldRun();
  void IterateGraphics();
  void CheckKeyboardAction();
private:
  std::shared_ptr<Gui> gui_;
  std::shared_ptr<Settings> user_settings_;
};
}
#endif  //WORLD_H__
