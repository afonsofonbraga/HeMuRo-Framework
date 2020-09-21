//
//  DecomposableTasks.hpp
//  MRSMac
//
//  Created by Afonso Braga on 31/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef DecomposableTasks_hpp
#define DecomposableTasks_hpp

#include <stdio.h>
#include "BlackBoard.hpp"
#include "MissionExecution.hpp"


#include "GoTo.hpp"
#include "ChargeBattery.hpp"
#include "TurnOn.hpp"
#include "TakePicture.hpp"

void decomposableTaskList(BlackBoard* monitor);
void addAtomicTask2(BlackBoard* monitor, MissionExecution& vMissionDecomposable);
#endif /* DecomposableTasks_hpp */
