//
//  ModulePeriodic.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

/*! \class ModulePeriodic
*  \author Afonso Braga
*  \date 2020
* \brief This class implements a Periodic Module.
 *
* This class implements a Periodic Module.
 *
* It will be executed the \run method every tick.
 *
* \warning Do not create a loop inside run()
*/

#ifndef ModulePeriodic_hpp
#define ModulePeriodic_hpp

#include <iostream>
#include <thread>
#include <chrono>
#include "Blackboard.hpp"
#include "Module.hpp"


class ModulePeriodic: public Module {
protected:
    void mainThread() override;                                          /*!< Thread object */
    void run() override ;                                                /*!< Thread Implementation */
    std::chrono::milliseconds tick = std::chrono::milliseconds(1000);    /*!< Threads period */
public:
    ModulePeriodic(Blackboard* monitor);                                 /*!< Constructor with a Default Period */
    ModulePeriodic(Blackboard* monitor, int vTick);                      /*!< Ordnary Constructor */
    ~ModulePeriodic();                                                   /*!< Destructor */
};

#endif /* ModulePeriodic_hpp */
