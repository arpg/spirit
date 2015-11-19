#ifndef SPIRIT_CAR_H_
#define SPIRIT_CAR_H_

#include <CarPlanner/LocalPlanner.h>
#include <spirit/objects/CommonObj.h>
#include <spirit/objects/GLCar.h>


struct SpiritCar {
  BulletCarModel physicscar;
  GLCar glcar;
  SceneGraph::GLAxis glaxis;
  CarParameters params;
  SceneGraph::GLCachedPrimitives carlinesegments;
  double tic_time;
};

class SpiritCars : public SpiritCommonObj {
 public:
  SpiritCars(SceneGraph::GLSceneGraph& graph);
  ~SpiritCars();

  int AddObj(Eigen::Vector6d T_w_c);
  int NumOfObjs();
  int DelObj(int obj_num);
  void InitCarParams();
  void InitializeMap(btCollisionShape* col_shape);
  void SetCarState(const int& id, const VehicleState& state,
                   bool bAddToTrajectory /* = false */);
  void SetCarVisibility(const int& id, const bool& bVisible);
  void UpdateVisualsFromPhysics(const int& world_id);

  void Clear() { for(size_t ii = 0; ii < NumOfObjs() ; ii++ ) { DelObj(ii); } }

 private:
  // glgraph to be updated
  SceneGraph::GLSceneGraph* glgraph_;

  ControlCommand cmd_;
  std::vector<std::unique_ptr<SpiritCar>> vec_;
  btCollisionShape* collision_shape_;
  CarParameterMap default_params_map_;
  int num_of_worlds_;
};

#endif  // SPIRIT_CAR_H_
