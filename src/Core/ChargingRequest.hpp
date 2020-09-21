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
    char requestID[10]= "null";
    enum_ChargingOperation operation = enum_ChargingOperation::null;
    enum_RobotCategory robotCat;
    s_pose robotsPosition;
    char robotsAddress[16] = "null";
    char robotsName[10] = "null";
};
#endif /* ChargingRequest_hpp */
