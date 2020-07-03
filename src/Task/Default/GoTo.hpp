//
//  GoTo.hpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef GoTo_hpp
#define GoTo_hpp

#include "AtomicTask.hpp"
#include <cmath>

class GoTo : public AtomicTask
{
protected:
    float costMeter = 2.0;
public:
    GoTo(s_pose& start, s_pose& end);
    ~GoTo();
    void run() override;
    void calculateCost() override;
    
};

#endif /* GoTo_hpp */