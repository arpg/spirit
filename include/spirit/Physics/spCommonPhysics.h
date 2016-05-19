#ifndef SP_COMMONPHYSICS_H__
#define SP_COMMONPHYSICS_H__

class spCommonPhysics {
public:
  virtual void InitPhysics() = 0;
  virtual void StepSimulation(double time_step) = 0;
  virtual void ExitPhysics() = 0;
};

#endif // SP_COMMONPHYSICS_H__
