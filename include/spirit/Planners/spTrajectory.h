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
  spObjectHandle AddWaypoint(const spPose& pose, double velocity=1, bool en_default_3ord3dof_traj=false);
  spWaypoint& GetWaypoint(unsigned int index);
  void UpdateWaypoint(const spWaypoint& planpoint,unsigned int index);
  const spCurve& GetCurve(unsigned int index);
  void RemoveWaypoint(unsigned int index_in_plan);
  void IsLoop(bool is_loop);
  int GetNumWaypoints();
  void UpdateCurves();
  void SetControls(int waypoint_index, const spCtrlPts2ord_2dof& control_command);
  spCtrlPts2ord_2dof& GetControls(int waypoint_index);
  void SetTrajectoryPoints(int waypoint_index, const spPoints3d& traj_pts);
  spVehicleConstructionInfo* GetVehicleInfo(int waypoint_index);

private:
  Gui&  gui_;
  Objects& objects_;

  bool is_loop_;
  spCtrlPts3ord_3dof ctrl_pts_;
//  bool enable_default_traj;

//  std::vector<std::shared_ptr<spObjectHandle>> waypoint_handle_vec_;
  std::vector<spObjectHandle> linestrip_handle_vec_;
  std::vector<spLineStrip*> trajectorystrip_vec_;
  std::vector<spWaypoint*> waypoint_vec_;
  std::vector<spVehicleConstructionInfo*> vehicle_info_vec_;
  std::vector<std::shared_ptr<spCtrlPts2ord_2dof>> control_command_vec_;
  std::vector<std::shared_ptr<spCurve>> curve_vec_;
  std::vector<bool> has_def_traj_vec_;
  const int num_pts_per_curve = 50;

  void InitControlCommand(spCtrlPts2ord_2dof* cntrl_cmd);

};

#endif  //  SP_TRAJECTORY_H__
