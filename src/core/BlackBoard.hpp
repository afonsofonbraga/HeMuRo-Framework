#ifndef blackBoard_hpp
#define blackBoard_hpp

#include <iostream>
#include <string>
#include <thread>



struct s_pose
{
  float x;
  float y;
  float theta;
};



class BlackBoard {
private:
  s_pose position;

public:
  BlackBoard();
  ~BlackBoard();
};

#endif
