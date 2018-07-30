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
  spTrajectory(Gui& gui, std::shared_ptr<Objects> &objects);
  ~spTrajectory();

  spObjectHandle AddWaypoint(const spPose& pose, double velocity=1/*, bool en_default_3ord3dof_traj=false*/);
  void PlaybackTrajectoryOnGUI(const spVehicleConstructionInfo& vehicle_params, int waypoint_index, double playback_ratio=1, int max_num_steps = -1);
  void SetTrajectoryStateSeries(int waypoint_index, std::shared_ptr<spStateSeries> state_series );
  std::shared_ptr<spStateSeries> GetTrajectoryStateSeries(int waypoint_index) const;
  void SetControls(int waypoint_index, const spCtrlPts2ord_2dof& control_command);
  spCtrlPts2ord_2dof& GetControls(int waypoint_index);
  spWaypoint& GetWaypoint(unsigned int index) const;
  int GetNumWaypoints() const;
  void IsLoop(bool is_loop);
  bool IsLoop() const;
  void SetTravelDuration(int waypoint_index, double travel_time);
  double GetTravelDuration(int waypoint_index) const;
  //  void UpdateCurves();

private:
  Gui&  gui_;
  std::shared_ptr<Objects> objects_;
  bool is_loop_;
  spCtrlPts3ord_3dof ctrl_pts_;
  std::vector<spObjectHandle> linestrip_handle_vec_;
//  std::vector<spLineStrip*> trajectorystrip_vec_;
//  std::vector<std::vector<spState*>*> trajectorystrip_state_vec_;
  std::vector<spWaypoint*> waypoint_vec_;
  std::vector<spVehicleConstructionInfo*> vehicle_info_vec_;
  std::vector<std::shared_ptr<spCtrlPts2ord_2dof>> control_command_vec_;
//  std::vector<std::shared_ptr<spCurve>> curve_vec_;
//  std::vector<bool> has_def_traj_vec_;
  std::vector<double> travel_duration_vec_;
//  const int num_pts_per_curve = 50;
  std::vector<std::shared_ptr<spStateSeries>> stateseries_vec_;

  void InitControlCommand(spCtrlPts2ord_2dof* cntrl_cmd);

};

#endif  //  SP_TRAJECTORY_H__
