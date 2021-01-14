//
//  turnOn.hpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TurnOn_hpp
#define TurnOn_hpp

#include "AtomicTask.hpp"
#include <cmath>
#include "BlackBoard.hpp"

class TurnOn: public AtomicTask
{
protected:
    float costMeter = 4.0;
public:
    TurnOn(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~TurnOn();
    void run() override;
    void calculateCost() override;
    
};

#endif /* turnOn_hpp */
