//
//  datatypes.h
//  MRSFramework
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef dataTypes_h
#define dataTypes_h

enum class TaskDescription{null, chargeBattery, turnOn, goTo};
enum class Operation{null, setRobotsPosition, missionAssignment};
enum class RobotType{null, uav, ugv, usv};
enum class TaskStatus{null, waiting, running, completed};

enum class enum_AtomicTask{null, chargeBattery, turnOn, goTo};
enum class enum_DecomposableTask{null, checkPosition};


template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}


struct s_pose
{
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct s_robotsPose
{
    std::string robotName;
    s_pose position;
};

struct s_UDPMessage
{
    char address[15];
    char buffer[500];
    int messageSize;
};

#endif /* datatypes_h */
