#ifndef SP_SIMCOMMONFUNCTOR_H__
#define SP_SIMCOMMONFUNCTOR_H__

#include <spirit/Types/spTypes.h>

class spSimCommonFunctor {
  public:
    virtual void RunInThread(int thread_id, double num_sim_steps, double step_size,
                             const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                             int pert_index,
                             std::shared_ptr<spStateSeries> traj_states,
                             std::shared_ptr<spState> init_state) = 0;
    virtual void WaitForThreadJoin() = 0;
    virtual void operator()(int thread_id, double num_sim_steps, double step_size,
                            const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                            int pert_index,
                            std::shared_ptr<spStateSeries> traj_states,
                            std::shared_ptr<spState> init_state) = 0;
};

#endif  // SP_SIMCOMMONFUNCTOR_H__
