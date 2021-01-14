//
//  TurnOnSim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TurnOnSim_hpp
#define TurnOnSim_hpp

#include "AtomicTask.hpp"
#include <cmath>
#include "BlackBoard.hpp"

class TurnOnSim: public AtomicTask
{
protected:
public:
    TurnOnSim(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~TurnOnSim();
    void run() override;
    void calculateCost() override;
    
};

#endif /* TurnOnSim_hpp */
