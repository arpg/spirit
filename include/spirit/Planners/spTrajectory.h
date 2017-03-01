#ifndef SP_TRAJECTORY_H__
#define SP_TRAJECTORY_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>
#include <spirit/Objects.h>
#include <thread>
#include <spirit/Gui.h>

// Local planner includes a set of

class spTrajectory {
public:
  spTrajectory(Gui& gui, Objects& objects);
  ~spTrajectory();

  void AddWaypoint(const spWaypoint& waypoint, unsigned int index);
  spObjectHandle AddWaypoint(const spPose& pose);
  const spWaypoint& GetWaypoint(unsigned int index);
  void UpdateWaypoint(const spWaypoint& planpoint,unsigned int index);
  const spCurve& GetCurve(unsigned int index);
  void RemoveWaypoint(unsigned int index_in_plan);
  void IsLoop(bool is_loop);
  int GetNumWaypoints();
  void UpdateCurves();

private:
  Gui&  gui_;
  Objects& objects_;

  bool is_loop_;
  spCtrlPts3ord_3dof ctrl_pts_;

//  std::vector<std::shared_ptr<spObjectHandle>> waypoint_handle_vec_;
//  std::vector<std::shared_ptr<spObjectHandle>> linestrip_handle_vec_;
  std::vector<spLineStrip*> linestrip_vec_;
  std::vector<spWaypoint*> waypoint_vec_;
  std::vector<std::shared_ptr<spCurve>> curve_vec_;
  std::vector<bool> needs_update_vec_;
  const int num_pts_per_curve = 50;
};

#endif  //  SP_TRAJECTORY_H__
