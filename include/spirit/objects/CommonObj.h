#ifndef COMMON_OBJ_
#define COMMON_OBJ_

class SpiritCommonObj {
 public:
  virtual int AddObj(Eigen::Vector6d T_w_a) =0;
  virtual int NumOfObjs() =0;
  virtual int DelObj(int objnum) =0;

};

#endif    //COMMON_OBJ_
