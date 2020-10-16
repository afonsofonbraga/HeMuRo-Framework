//
//  dataTask.h
//  MRSMac
//
//  Created by Afonso Braga on 31/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef dataTask_hpp
#define dataTask_hpp

enum class enum_AtomicTask{null, chargeBattery, turnOn, goTo, MoveBaseGoal, takePicture};
enum class enum_DecomposableTask{null, checkPosition, lowBattery, takePicture, flightTest, deliverPicture}; //Trocar por DecomposableMission

#endif /* dataTask_h */
