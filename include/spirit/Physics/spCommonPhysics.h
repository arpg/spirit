#ifndef SP_COMMONPHYSICS_H__
#define SP_COMMONPHYSICS_H__

class spCommonPhysics {
public:
  virtual InitPhysics() = 0;
  virtual StepSimulation(double time_step) = 0;
  virtual ExitPhysics() = 0;
};

#endif // SP_COMMONPHYSICS_H__
