//
//  ChargingRequest.h
//  MRSMac
//
//  Created by Afonso Braga on 04/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ChargingRequest_hpp
#define ChargingRequest_hpp

#include "dataTypes.hpp"

struct s_ChargingRequest
{
    char requestID[MAX_ID]= "null";
    enum_ChargingOperation operation = enum_ChargingOperation::null;
    enum_RobotCategory robotCat;
    s_pose robotsPosition;
    char robotsAddress[MAX_IP] = "null";
    char robotsName[MAX_ROBOT_ID] = "null";
};
#endif /* ChargingRequest_hpp */
