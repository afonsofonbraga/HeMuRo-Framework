#include "BlackBoard.hpp"

BlackBoard::BlackBoard(std::string& name, enum_RobotCategory cat)
{
    this->position.x = 0;
    this->position.y = 0;
    this->position.z = 0;
    this->position.roll = 0;
    this->position.pitch = 0;
    this->position.yaw = 0;
    this->batteryLevel = 20;
    setRobotsName(name);
    setRobotCategory(cat);
    this->setRobotIP();
    this->robotStatus = enum_RobotStatus::available;
    
}

BlackBoard::~BlackBoard(){
}

BlackBoard::BlackBoard(const BlackBoard& other)
{
    this->batteryLevel = other.batteryLevel;
    this->position.x = other.position.x;
    this->position.y = other.position.y;
    this->position.z = other.position.z;
    this->position.roll = other.position.roll;
    this->position.pitch = other.position.pitch;
    this->position.yaw = other.position.yaw;
    //std::string vName = other.robotName;
    this->setRobotIP();
    this->setRobotsName(other.robotName);
    this->setRobotCategory(other.robotCategory);
    this->robotStatus = enum_RobotStatus::available;
}

BlackBoard& BlackBoard::operator=(const BlackBoard& other)
{
    if(this != &other) {
        this->batteryLevel = other.batteryLevel;
        this->position.x = other.position.x;
        this->position.y = other.position.y;
        this->position.z = other.position.z;
        this->position.roll = other.position.roll;
        this->position.pitch = other.position.pitch;
        this->position.yaw = other.position.yaw;
        //std::string vName = new std::string{other.robotName};
        this->setRobotIP();
        this->setRobotsName(other.robotName);
        this->setRobotCategory(other.robotCategory);
        this->robotStatus = enum_RobotStatus::available;
    }
    return *this;
}

//****************************************************
//*               Robot's Description                *
//****************************************************


void BlackBoard::setRobotsName(std::string name)
{
    this->robotName = name;
}

void BlackBoard::getRobotsName(std::string& name)
{
    name = this->robotName;
}

void BlackBoard::getRobotsName(char& name)
{
    strcpy(&name, this->robotName.c_str());
}

void BlackBoard::setRobotCategory(enum_RobotCategory cat)
{
    this->robotCategory = cat;
}

enum_RobotCategory BlackBoard::getRobotsCategory()
{
    return this->robotCategory;
}




void BlackBoard::setRobotIP()
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family==AF_INET && strstr(ifa->ifa_name,"lo")==nullptr) {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            strcpy(this->robotIP, inet_ntoa(sa->sin_addr));
            break;
        }
    }
    freeifaddrs(ifap);
    strncpy(this->broadcastIP, this->robotIP, sizeof(robotIP));
    char* lastDot = strrchr(this->broadcastIP,'.');
    strcpy(lastDot + 1, "255");
    //strncpy(this->broadcastIP, "127.0.0.1", sizeof(robotIP));
}

void BlackBoard::getRobotsIP(char& vIP)
{
    strcpy(&vIP, this->robotIP);
}

void BlackBoard::getBroadcastIP(char& vBroadcast)
{
    strcpy(&vBroadcast, this->broadcastIP);
}


void BlackBoard::chargeBattery(float energy)
{
    std::unique_lock<std::mutex> lk(mutex_battery);
    this->batteryLevel + energy < 100.0 ? this->batteryLevel += energy : this->batteryLevel = 100.0;
    lk.unlock();
}

void BlackBoard::consumeBattery(float energy)
{
    std::unique_lock<std::mutex> lk(mutex_battery);
    this->batteryLevel - energy > 0.0 ? this->batteryLevel -= energy : this->batteryLevel = 0;
    lk.unlock();
}

float BlackBoard::getBatteryLevel()
{
    float vBattery;
    std::unique_lock<std::mutex> lk(mutex_battery);
    vBattery = this->batteryLevel;
    lk.unlock();
    return vBattery;
}

void BlackBoard::setBatteryLevel(float energy)
{
    std::unique_lock<std::mutex> lk(mutex_battery);
    this->batteryLevel = energy;
    lk.unlock();
}


//****************************************************
//*           Robot's Position Variables             *
//****************************************************


void BlackBoard::getPosition(s_pose& p)
{
    std::unique_lock<std::mutex> lk(mutex_position);
    //start = std::chrono::high_resolution_clock::now();
    memcpy(&p, &position, sizeof(position));
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "getPositionMemcpy time: " << duration << std::endl;
    lk.unlock();
}

void BlackBoard::setPosition(s_pose& p)
{
    std::unique_lock<std::mutex> lk(mutex_position);
    memcpy(&this->position, &p, sizeof(p));
    lk.unlock();
}

void BlackBoard::setAllRobotsPosition(s_BroadcastMessage &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
    //start = std::chrono::high_resolution_clock::now();
    this->mapRobotsPosition.insert_or_assign(p.robotName, p);
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "set " << p.robotName << " time: " << duration << std::endl;
    lk.unlock();
}

void BlackBoard::removeAllRobotsPosition(s_BroadcastMessage &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
    //start = std::chrono::high_resolution_clock::now();
    this->mapRobotsPosition.erase(p.robotName);
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "Remove " << p.robotName << " time: " << duration << std::endl;
    lk.unlock();
}

void BlackBoard::getAllRobotsPosition(std::unordered_map<std::string, s_BroadcastMessage> &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
    //start = std::chrono::high_resolution_clock::now();
    //memcpy(&p, &this->mapRobotsPosition, sizeof(mapRobotsPosition));
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "getAllRobotsPosition time: " << duration << std::endl;
    for (auto n : mapRobotsPosition)
    {
        p[n.first] = n.second;
    }
    lk.unlock();
}


//****************************************************
//*             Map Related Functions                *
//****************************************************

void BlackBoard::setMapCoodinates(std::array<float,2>& coord)
{
    //this->mutex_map.lock();
    std::unique_lock<std::mutex> lk(mutex_map);
    this->mapSize = coord;
    //this->mutex_map.unlock();
    lk.unlock();
}

void BlackBoard::getMapCoodinates(std::array<float,2>& coord)
{
    this->mutex_map.lock();
    std::unique_lock<std::mutex> lk(mutex_map);
    coord = this->mapSize;
    //this->mutex_map.unlock();
    lk.unlock();
}


//****************************************************
//*      Decomposable Tasks Related Functions        *
//****************************************************


void BlackBoard::addDecomposableTaskList(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask> vAtomicTask)
{
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
    this->decomposableTaskAvaliable.insert_or_assign(vTaskToBeDecomposed, vAtomicTask);
    lk.unlock();
}

bool BlackBoard::getDecomposableTask(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask>& vAtomicTask)
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
    auto search = this->decomposableTaskAvaliable.find(vTaskToBeDecomposed);
    if(search != this->decomposableTaskAvaliable.end())
    {
        for(auto n : search->second)
        {
            vAtomicTask.push_back(n);
        }
        status = true;
    }
    lk.unlock();
    return status;
}

bool BlackBoard::isDecomposable(enum_DecomposableTask vTaskToBeDecomposed)
{
    bool status= false;
    
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
    auto search = this->decomposableTaskAvaliable.find(vTaskToBeDecomposed);
    lk.unlock();
    
    if (search != this->decomposableTaskAvaliable.end())
        status = true;
    return status;
}
/*
 float BlackBoard::getDecomposableTaskCost(enum_DecomposableTask vTaskToBeDecomposed)
 {
 return 0.0;
 }
 
 void BlackBoard::acceptDecomposableTask(enum_DecomposableTask vDecomposableTask)
 {
 std::unique_lock<std::mutex> lk(mutex_decomposableTask);
 
 lk.unlock();
 }*/

//****************************************************
//*         MissionMessage Related Functions         *
//****************************************************

bool BlackBoard::isMissionMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_missionList);
    status = this->missionMessageList.empty();
    lk.unlock();
    return status;
}

void BlackBoard::addMissionMessage(s_MissionMessage& vMissionMessage)
{
    std::unique_lock<std::mutex> lk(mutex_missionList);
    this->missionMessageList.push_back(vMissionMessage);
    lk.unlock();
    this->conditional_MissionMessageList.notify_one();
}

void BlackBoard::getMissionMessage(s_MissionMessage& vMissionMessage)
{
    std::unique_lock<std::mutex> lk(mutex_missionList);
    
    if (this->missionMessageList.empty() == false){
        vMissionMessage = this->missionMessageList.front();
        this->missionMessageList.erase(missionMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_MissionMessageList.wait(lk);
        
        if (this->missionMessageList.empty() == false){
            vMissionMessage = this->missionMessageList.front();
            this->missionMessageList.erase(missionMessageList.begin());
            lk.unlock();
        }
    }
}

// Selected Mission

bool BlackBoard::isMissionCompleted()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    //if(this->selectedMission.enum_execution == enum_MissionExecution::missionComplete)
    //    status = true;
    lk.unlock();
    return status;
}

bool BlackBoard::isRobotAvailable()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_mission);
    //this->executingMission ? status = false:status = true;
    this->robotStatus == enum_RobotStatus::available ? status = true : status = false;
    lk.unlock();
    return status;
}

bool BlackBoard::lockRobot(enum_RobotStatus statusRequest)
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    /*
    if (this->robotStatus == enum_RobotStatus::available && statusRequest == enum_RobotStatus::executing)
    {
        status = true;
        this->robotStatus = enum_RobotStatus::executing;
    } else if ((this->robotStatus == enum_RobotStatus::available || this->robotStatus == enum_RobotStatus::executing) && statusRequest == enum_RobotStatus::emergency)
    {
        status = true;
        this->robotStatus = enum_RobotStatus::emergency;
    } else
        status = false;*/
    
    switch(statusRequest)
    {
        case enum_RobotStatus::executing:
            if(this->robotStatus == enum_RobotStatus::available)
            {
                status = true;
                this->robotStatus = enum_RobotStatus::executing;
            } else
                status = false;
            break;
        case enum_RobotStatus::emergency:
            if(this->robotStatus != enum_RobotStatus::failure)
            {
                status = true;
                this->robotStatus = enum_RobotStatus::emergency;
            } else
                status = false;
            break;
        case enum_RobotStatus::lowBattery:
            if(this->robotStatus != enum_RobotStatus::failure)
            {
                status = true;
                this->robotStatus = enum_RobotStatus::lowBattery;
            } else
                status = false;
            break;
        default:
            status = false;
            break;
    }
    lk.unlock();
    return status;
}

bool BlackBoard::unlockRobot()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    
    if(this->robotStatus == enum_RobotStatus::executing || this->robotStatus == enum_RobotStatus::emergency)
    {
        status = true;
        this->robotStatus = enum_RobotStatus::available;
    } else
        status = false;
    /*
     if (this->executingMission == true)
     {
     status= true;
     this->executingMission= false;
     } else
     status = false;
     */
    lk.unlock();
    return status;
}

enum_RobotStatus BlackBoard::getRobotStatus()
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    enum_RobotStatus status = this->robotStatus;
    lk.unlock();
    return status;
}

// Deprecated
/*
 void BlackBoard::addMissionToExecute(MissionExecution& vMission)
 {
 std::unique_lock<std::mutex> lk(mutex_mission);
 if (this->selectedMission.enum_execution == enum_MissionExecution::missionComplete || this->selectedMission.enum_execution == enum_MissionExecution::null) // Is necessary to check if the previous mission is already completed.
 {
 this->selectedMission = vMission;
 this->selectedMission.atomicTaskIndex = 0;
 this->selectedMission.enum_execution = enum_MissionExecution::waitingStart;
 std::cout << "Tamanho da mensagem "<< sizeof(this->selectedMission) << std::endl;
 //this->conditional_missionTask.notify_one();
 }
 lk.unlock();
 }
 
 std::shared_ptr<AtomicTask> BlackBoard::getTaskFromMission()
 {
 std::unique_lock<std::mutex> lk(mutex_mission);
 
 std::shared_ptr<AtomicTask> vtask = nullptr;
 if (this->selectedMission.enum_execution == enum_MissionExecution::executing)
 {
 vtask = this->selectedMission.selectNextAction();
 } else
 {
 this->conditional_missionTask.wait(lk);
 vtask = this->selectedMission.selectNextAction();
 }
 lk.unlock();
 return vtask;
 }
 
 void BlackBoard::startMissionExecution()
 {
 std::unique_lock<std::mutex> lk(mutex_mission);
 if(this->selectedMission.enum_execution == enum_MissionExecution::waitingStart)
 {
 this->selectedMission.enum_execution = enum_MissionExecution::executing;
 this->conditional_missionTask.notify_one();
 }
 lk.unlock();
 }
 
 void BlackBoard::cancelMission()
 {
 std::unique_lock<std::mutex> lk(mutex_mission);
 this->selectedMission.enum_execution = enum_MissionExecution::null;
 lk.unlock();
 }
 */


//****************************************************
//*         UDPMessages Related Functions            *
//****************************************************

bool BlackBoard::isUDPMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_UDPMessageList);
    status = this->UDPMessageList.empty();
    lk.unlock();
    return status;
}

void BlackBoard::addUDPMessage(s_UDPMessage& vUDPMessage)
{
    std::unique_lock<std::mutex> lk(mutex_UDPMessageList);
    this->UDPMessageList.push_back(vUDPMessage);
    lk.unlock();
    this->conditional_UDPMessageList.notify_one();
}

void BlackBoard::getUDPMessage(s_UDPMessage& vUDPMessage)
{
    std::unique_lock<std::mutex> lk(mutex_UDPMessageList);
    
    if (this->UDPMessageList.empty() == false){
        vUDPMessage = this->UDPMessageList.front();
        this->UDPMessageList.erase(UDPMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_UDPMessageList.wait(lk);
        
        if (this->UDPMessageList.empty() == false){
            vUDPMessage = this->UDPMessageList.front();
            this->UDPMessageList.erase(UDPMessageList.begin());
            lk.unlock();
        }
    }
}

//****************************************************
//*         ROSBridgeMessages Related Functions      *
//****************************************************

bool BlackBoard::isROSBridgeMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_ROSBridgeMessageList);
    status = this->ROSBridgeMessageList.empty();
    lk.unlock();
    return status;
}

void BlackBoard::addROSBridgeMessage(s_ROSBridgeMessage& vROSBridgeMessage)
{
    std::unique_lock<std::mutex> lk(mutex_ROSBridgeMessageList);
    this->ROSBridgeMessageList.push_back(vROSBridgeMessage);
    lk.unlock();
    this->conditional_ROSBridgeMessageList.notify_one();
}

void BlackBoard::getROSBridgeMessage(s_ROSBridgeMessage& vROSBridgeMessage)
{
    std::unique_lock<std::mutex> lk(mutex_ROSBridgeMessageList);
    
    if (this->ROSBridgeMessageList.empty() == false){
        vROSBridgeMessage = this->ROSBridgeMessageList.front();
        this->ROSBridgeMessageList.erase(ROSBridgeMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_ROSBridgeMessageList.wait(lk);
        
        if (this->ROSBridgeMessageList.empty() == false){
            vROSBridgeMessage = this->ROSBridgeMessageList.front();
            this->ROSBridgeMessageList.erase(ROSBridgeMessageList.begin());
            lk.unlock();
        }
    }
}

//****************************************************
//*         BatteryMessage Related Functions         *
//****************************************************

bool BlackBoard::isBatteryMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_batteryList);
    status = this->batteryMessageList.empty();
    lk.unlock();
    return status;
}

void BlackBoard::addBatteryMessage(s_BatteryMessage& vBatteryMessage)
{
    std::unique_lock<std::mutex> lk(mutex_batteryList);
    this->batteryMessageList.push_back(vBatteryMessage);
    lk.unlock();
    this->conditional_BatteryMessageList.notify_one();
}

void BlackBoard::getBatteryMessage(s_BatteryMessage& vBatteryMessage)
{
    std::unique_lock<std::mutex> lk(mutex_batteryList);
    
    if (this->batteryMessageList.empty() == false){
        vBatteryMessage = this->batteryMessageList.front();
        this->batteryMessageList.erase(batteryMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_BatteryMessageList.wait(lk);
        
        if (this->batteryMessageList.empty() == false){
            vBatteryMessage = this->batteryMessageList.front();
            this->batteryMessageList.erase(batteryMessageList.begin());
            lk.unlock();
        }
    }
}

//****************************************************
//*          LoggerMessage Related Functions         *
//****************************************************

bool BlackBoard::isLoggerMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_loggerMessageList);
    status = this->loggerMessageList.empty();
    lk.unlock();
    return status;
}

void BlackBoard::addLoggerMessage(s_LoggerMessage &vLoggerMessage)
{
    std::unique_lock<std::mutex> lk(mutex_loggerMessageList);
    this->loggerMessageList.push_back(vLoggerMessage);
    lk.unlock();
    this->conditional_LoggerMessageList.notify_one();
}

void BlackBoard::getLoggerMessage(s_LoggerMessage &vLoggerMessage)
{
    std::unique_lock<std::mutex> lk(mutex_loggerMessageList);
    
    if (this->loggerMessageList.empty() == false){
        vLoggerMessage = this->loggerMessageList.front();
        this->loggerMessageList.erase(loggerMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_LoggerMessageList.wait(lk);
        
        if (this->loggerMessageList.empty() == false){
            vLoggerMessage = this->loggerMessageList.front();
            this->loggerMessageList.erase(loggerMessageList.begin());
            lk.unlock();
        }
    }
}

void BlackBoard::print(std::string vText)
{
    s_LoggerMessage message;
    this->getRobotsName(*message.robotName);
    message.operation = enum_LoggerOperation::print;
    
    strncpy(message.buffer,vText.c_str(),sizeof(message.buffer)-1);
    message.buffer[sizeof(message.buffer)-1]='\0';
    
    s_UDPMessage UDPMessage;
    
    strcpy(UDPMessage.address, this->broadcastIP);
    strcpy(UDPMessage.name, "Logger");
    memcpy(UDPMessage.buffer, "Logger",MAX_ROBOT_ID);
    
    Operation operation = Operation::loggerMessage;
    *((Operation*)(UDPMessage.buffer + MAX_ROBOT_ID)) = operation;
    *((int*)(UDPMessage.buffer + MAX_ROBOT_ID + 4)) = sizeof(message);
    memmove(UDPMessage.buffer + MAX_ROBOT_ID + 8,(const unsigned char*)&message,sizeof(message));
    UDPMessage.messageSize = sizeof(UDPMessage.buffer);
    this->addUDPMessage(UDPMessage);
}
