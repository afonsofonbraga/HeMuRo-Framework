#include "Blackboard.hpp"

Blackboard::Blackboard(std::string& name, enum_RobotCategory cat)
{
    this->position.x = 0;
    this->position.y = 0;
    this->position.z = 0;
    this->position.roll = 0;
    this->position.pitch = 0;
    this->position.yaw = 0;
    this->batteryLevel = 100;
    setAgentsName(name);
    setRobotCategory(cat);
    this->setAgentsIP();
    this->agentStatus = enum_RobotStatus::available;
    
}

Blackboard::~Blackboard(){
}

Blackboard::Blackboard(const Blackboard& other)
{
    this->batteryLevel = other.batteryLevel;
    this->position.x = other.position.x;
    this->position.y = other.position.y;
    this->position.z = other.position.z;
    this->position.roll = other.position.roll;
    this->position.pitch = other.position.pitch;
    this->position.yaw = other.position.yaw;
    //std::string vName = other.robotName;
    this->setAgentsIP();
    this->setAgentsName(other.agentName);
    this->setRobotCategory(other.agentCategory);
    this->agentStatus = enum_RobotStatus::available;
}

Blackboard& Blackboard::operator=(const Blackboard& other)
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
        this->setAgentsIP();
        this->setAgentsName(other.agentName);
        this->setRobotCategory(other.agentCategory);
        this->agentStatus = enum_RobotStatus::available;
    }
    return *this;
}

//****************************************************
//*               Robot's Description                *
//****************************************************


void Blackboard::setAgentsName(std::string name)
{
    this->agentName = name;
}

void Blackboard::getRobotsName(std::string& name)
{
    name = this->agentName;
}

void Blackboard::getRobotsName(char& name)
{
    strcpy(&name, this->agentName.c_str());
}

void Blackboard::setRobotCategory(enum_RobotCategory cat)
{
    this->agentCategory = cat;
}

enum_RobotCategory Blackboard::getRobotsCategory()
{
    return this->agentCategory;
}




void Blackboard::setAgentsIP()
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family==AF_INET && strstr(ifa->ifa_name,"lo")==nullptr) {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            strcpy(this->agentIP, inet_ntoa(sa->sin_addr));
            break;
        }
    }
    freeifaddrs(ifap);
    strncpy(this->broadcastIP, this->agentIP, sizeof(agentIP));
    char* lastDot = strrchr(this->broadcastIP,'.');
    strcpy(lastDot + 1, "255");
    //strncpy(this->broadcastIP, "127.0.0.1", sizeof(robotIP));
}

void Blackboard::getRobotsIP(char& vIP)
{
    strcpy(&vIP, this->agentIP);
}

void Blackboard::getBroadcastIP(char& vBroadcast)
{
    strcpy(&vBroadcast, this->broadcastIP);
}


void Blackboard::chargeBattery(float energy)
{
    std::unique_lock<std::mutex> lk(mutex_battery);
    this->batteryLevel + energy < 100.0 ? this->batteryLevel += energy : this->batteryLevel = 100.0;
    lk.unlock();
}

void Blackboard::consumeBattery(float energy)
{
    std::unique_lock<std::mutex> lk(mutex_battery);
    this->batteryLevel - energy > 0.0 ? this->batteryLevel -= energy : this->batteryLevel = 0;
    lk.unlock();
}

float Blackboard::getBatteryLevel()
{
    float vBattery;
    std::unique_lock<std::mutex> lk(mutex_battery);
    vBattery = this->batteryLevel;
    lk.unlock();
    return vBattery;
}

void Blackboard::setBatteryLevel(float energy)
{
    std::unique_lock<std::mutex> lk(mutex_battery);
    this->batteryLevel = energy;
    lk.unlock();
}


//****************************************************
//*           Robot's Position Variables             *
//****************************************************


void Blackboard::getPosition(s_pose& p)
{
    std::unique_lock<std::mutex> lk(mutex_position);
    //start = std::chrono::high_resolution_clock::now();
    memcpy(&p, &position, sizeof(position));
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "getPositionMemcpy time: " << duration << std::endl;
    lk.unlock();
}

void Blackboard::setPosition(s_pose& p)
{
    std::unique_lock<std::mutex> lk(mutex_position);
    memcpy(&this->position, &p, sizeof(p));
    lk.unlock();
}

void Blackboard::getBasisPosition(s_pose& p)
{
    std::unique_lock<std::mutex> lk(mutex_position);
    memcpy(&p, &basisPosition, sizeof(s_pose));
    lk.unlock();
}

void Blackboard::setBasisPosition(s_pose& p)
{
    std::unique_lock<std::mutex> lk(mutex_position);
    memcpy(&this->basisPosition, &p, sizeof(s_pose));
    lk.unlock();
}

void Blackboard::setAllRobotsPosition(s_BroadcastMessage &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapAgentsPosition);
    //start = std::chrono::high_resolution_clock::now();
    this->mapAgentsPosition.insert_or_assign(p.robotName, p);
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "set " << p.robotName << " time: " << duration << std::endl;
    lk.unlock();
}

void Blackboard::removeAllRobotsPosition(s_BroadcastMessage &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapAgentsPosition);
    //start = std::chrono::high_resolution_clock::now();
    this->mapAgentsPosition.erase(p.robotName);
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "Remove " << p.robotName << " time: " << duration << std::endl;
    lk.unlock();
}

void Blackboard::getAllRobotsPosition(std::unordered_map<std::string, s_BroadcastMessage> &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapAgentsPosition);
    //start = std::chrono::high_resolution_clock::now();
    //memcpy(&p, &this->mapRobotsPosition, sizeof(mapRobotsPosition));
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
    //std::cout << "getAllRobotsPosition time: " << duration << std::endl;
    for (auto n : mapAgentsPosition)
    {
        p[n.first] = n.second;
    }
    lk.unlock();
}


//****************************************************
//*             Map Related Functions                *
//****************************************************

void Blackboard::setMapCoodinates(std::array<float,2>& coord)
{
    //this->mutex_map.lock();
    std::unique_lock<std::mutex> lk(mutex_map);
    this->mapSize = coord;
    //this->mutex_map.unlock();
    lk.unlock();
}

void Blackboard::getMapCoodinates(std::array<float,2>& coord)
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


void Blackboard::addDecomposableTaskList(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask> vAtomicTask)
{
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
    this->decomposableTaskAvailable.insert_or_assign(vTaskToBeDecomposed, vAtomicTask);
    lk.unlock();
}

bool Blackboard::getDecomposableTask(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask>& vAtomicTask)
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
    auto search = this->decomposableTaskAvailable.find(vTaskToBeDecomposed);
    if(search != this->decomposableTaskAvailable.end())
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

bool Blackboard::isDecomposable(enum_DecomposableTask vTaskToBeDecomposed)
{
    bool status= false;
    
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
    auto search = this->decomposableTaskAvailable.find(vTaskToBeDecomposed);
    lk.unlock();
    
    if (search != this->decomposableTaskAvailable.end())
        status = true;
    return status;
}
/*
 float Blackboard::getDecomposableTaskCost(enum_DecomposableTask vTaskToBeDecomposed)
 {
 return 0.0;
 }
 
 void Blackboard::acceptDecomposableTask(enum_DecomposableTask vDecomposableTask)
 {
 std::unique_lock<std::mutex> lk(mutex_decomposableTask);
 
 lk.unlock();
 }*/

//****************************************************
//*         TaskMessage Related Functions         *
//****************************************************

bool Blackboard::isTaskMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_taskList);
    status = this->taskMessageList.empty();
    lk.unlock();
    return status;
}

void Blackboard::addTaskMessage(s_TaskMessage& vTaskMessage)
{
    std::unique_lock<std::mutex> lk(mutex_taskList);
    this->taskMessageList.push_back(vTaskMessage);
    lk.unlock();
    this->conditional_TaskMessageList.notify_one();
}

void Blackboard::getTaskMessage(s_TaskMessage& vTaskMessage)
{
    std::unique_lock<std::mutex> lk(mutex_taskList);
    
    if (this->taskMessageList.empty() == false){
        vTaskMessage = this->taskMessageList.front();
        this->taskMessageList.erase(taskMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_TaskMessageList.wait(lk);
        
        if (this->taskMessageList.empty() == false){
            vTaskMessage = this->taskMessageList.front();
            this->taskMessageList.erase(taskMessageList.begin());
            lk.unlock();
        }
    }
}


//****************************************************
//*         MissionMessage Related Functions         *
//****************************************************

bool Blackboard::isMissionMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_missionList);
    status = this->missionMessageList.empty();
    lk.unlock();
    return status;
}

void Blackboard::addMissionMessage(s_MissionMessage& vMissionMessage)
{
    std::unique_lock<std::mutex> lk(mutex_missionList);
    this->missionMessageList.push_back(vMissionMessage);
    lk.unlock();
    this->conditional_MissionMessageList.notify_one();
}

void Blackboard::getMissionMessage(s_MissionMessage& vMissionMessage)
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

void Blackboard::setMissionStatus(s_MissionStatus &p)
{
    /* Primeiro deve checar se já tem algum ítem inscrito, se tiver apenas atualizar os dados.
       Se for dono da missão, pode atualizar os dados que foram enviados.
       Se fo o agente designado para executar a missão, pode atualizar apenas dados referentes a tempo de execução, conclusão.
     */
    std::unique_lock<std::mutex> lk(mutex_mapMissionStatus);
        auto search = this->map_missionStatus.find(p.missionCode);
        if (search != this->map_missionStatus.end()) {
            //EXISTE
            if (strcmp(search->second.missionOwner, p.missionOwner) == 0)
            {
                if (strcmp(p.missionExecutioner,"null") != 0 && strcmp(search->second.missionExecutioner, p.missionExecutioner) != 0) strcpy(search->second.missionExecutioner,p.missionExecutioner);
                if (p.relativeDeadline != std::chrono::seconds(0)) search->second.relativeDeadline = p.relativeDeadline;
                if (p.executionTime != std::chrono::seconds(0)) search->second.executionTime = p.executionTime;
                if (p.estimatedExecutionTime != std::chrono::seconds(0)) search->second.estimatedExecutionTime = p.estimatedExecutionTime;
                if (p.status != enum_MissionStatus::null) search->second.status = p.status;
                
                this->map_missionStatus.insert_or_assign(search->first, search->second);
            }
            
        } else {
            this->map_missionStatus.insert_or_assign(p.missionCode, p);
        }
    lk.unlock();
}

void Blackboard::getAllMissionStatus(std::unordered_map<std::string, s_MissionStatus> &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapMissionStatus);
    for (auto n : map_missionStatus)
    {
        p[n.first] = n.second;
    }
    lk.unlock();
}

void Blackboard::getMissionStatus(s_MissionStatus &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapMissionStatus);
    
    auto search = map_missionStatus.find(p.missionCode);
    if (search != map_missionStatus.end()) {
        p = search->second;
    }
    lk.unlock();
}

void Blackboard::removeMissionStatus(s_MissionStatus &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapMissionStatus);
        this->map_missionStatus.erase(p.missionCode);
    lk.unlock();
}

// Selected Mission

bool Blackboard::isMissionCompleted()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    //if(this->selectedMission.enum_execution == enum_MissionExecution::missionComplete)
    //    status = true;
    lk.unlock();
    return status;
}

bool Blackboard::isRobotAvailable()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_mission);
    //this->executingMission ? status = false:status = true;
    this->agentStatus == enum_RobotStatus::available ? status = true : status = false;
    lk.unlock();
    return status;
}

bool Blackboard::lockRobot(enum_RobotStatus statusRequest)
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
            if(this->agentStatus == enum_RobotStatus::available)
            {
                status = true;
                this->agentStatus = enum_RobotStatus::executing;
            } else
                status = false;
            break;
        case enum_RobotStatus::emergency:
            if(this->agentStatus != enum_RobotStatus::failure)
            {
                status = true;
                this->agentStatus = enum_RobotStatus::emergency;
            } else
                status = false;
            break;
        case enum_RobotStatus::lowBattery:
            if(this->agentStatus != enum_RobotStatus::failure)
            {
                status = true;
                this->agentStatus = enum_RobotStatus::lowBattery;
            } else
                status = false;
            break;
        case enum_RobotStatus::failure:
        {
            status = true;
            this->agentStatus = enum_RobotStatus::failure;
            break;
        }
        default:
            status = false;
            break;
    }
    lk.unlock();
    return status;
}

bool Blackboard::unlockRobot()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    
    if(this->agentStatus == enum_RobotStatus::executing || this->agentStatus == enum_RobotStatus::emergency || this->agentStatus == enum_RobotStatus::lowBattery)
    {
        status = true;
        this->agentStatus = enum_RobotStatus::available;
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

enum_RobotStatus Blackboard::getRobotStatus()
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    enum_RobotStatus status = this->agentStatus;
    lk.unlock();
    return status;
}

float Blackboard::getCostToExecute()
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    float cost = this->costToExecute;
    lk.unlock();
    return cost;
}

void Blackboard::setCostToExecute(float cost)
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    this->costToExecute = cost;
    lk.unlock();
}

void Blackboard::clearCostToExecute()
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    this->costToExecute = 0;
    lk.unlock();
}

// Deprecated
/*
 void Blackboard::addMissionToExecute(MissionExecution& vMission)
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
 
 std::shared_ptr<AtomicTask> Blackboard::getTaskFromMission()
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
 
 void Blackboard::startMissionExecution()
 {
 std::unique_lock<std::mutex> lk(mutex_mission);
 if(this->selectedMission.enum_execution == enum_MissionExecution::waitingStart)
 {
 this->selectedMission.enum_execution = enum_MissionExecution::executing;
 this->conditional_missionTask.notify_one();
 }
 lk.unlock();
 }
 
 void Blackboard::cancelMission()
 {
 std::unique_lock<std::mutex> lk(mutex_mission);
 this->selectedMission.enum_execution = enum_MissionExecution::null;
 lk.unlock();
 }
 */


//****************************************************
//*         UDPMessages Related Functions            *
//****************************************************

bool Blackboard::isUDPMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_UDPMessageList);
    status = this->UDPMessageList.empty();
    lk.unlock();
    return status;
}

void Blackboard::addUDPMessage(s_UDPMessage& vUDPMessage)
{
    std::unique_lock<std::mutex> lk(mutex_UDPMessageList);
    this->UDPMessageList.push_back(vUDPMessage);
    lk.unlock();
    this->conditional_UDPMessageList.notify_one();
}

void Blackboard::getUDPMessage(s_UDPMessage& vUDPMessage)
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
//*         ROSModuleMessages Related Functions      *
//****************************************************

bool Blackboard::isROSModuleMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_ROSModuleMessageList);
    status = this->ROSModuleMessageList.empty();
    lk.unlock();
    return status;
}

void Blackboard::addROSModuleMessage(s_ROSModuleMessage& vROSModuleMessage)
{
    std::unique_lock<std::mutex> lk(mutex_ROSModuleMessageList);
    this->ROSModuleMessageList.push_back(vROSModuleMessage);
    lk.unlock();
    this->conditional_ROSModuleMessageList.notify_one();
}

void Blackboard::getROSModuleMessage(s_ROSModuleMessage& vROSModuleMessage)
{
    std::unique_lock<std::mutex> lk(mutex_ROSModuleMessageList);
    
    if (this->ROSModuleMessageList.empty() == false){
        vROSModuleMessage = this->ROSModuleMessageList.front();
        this->ROSModuleMessageList.erase(ROSModuleMessageList.begin());
        lk.unlock();
    }else
    {
        this->conditional_ROSModuleMessageList.wait(lk);
        
        if (this->ROSModuleMessageList.empty() == false){
            vROSModuleMessage = this->ROSModuleMessageList.front();
            this->ROSModuleMessageList.erase(ROSModuleMessageList.begin());
            lk.unlock();
        }
    }
}

//****************************************************
//*         BatteryMessage Related Functions         *
//****************************************************

bool Blackboard::isBatteryMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_batteryList);
    status = this->batteryMessageList.empty();
    lk.unlock();
    return status;
}

void Blackboard::addBatteryMessage(s_BatteryMessage& vBatteryMessage)
{
    std::unique_lock<std::mutex> lk(mutex_batteryList);
    this->batteryMessageList.push_back(vBatteryMessage);
    lk.unlock();
    this->conditional_BatteryMessageList.notify_one();
}

void Blackboard::getBatteryMessage(s_BatteryMessage& vBatteryMessage)
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

bool Blackboard::isLoggerMessageListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_loggerMessageList);
    status = this->loggerMessageList.empty();
    lk.unlock();
    return status;
}

void Blackboard::addLoggerMessage(s_LoggerMessage &vLoggerMessage)
{
    std::unique_lock<std::mutex> lk(mutex_loggerMessageList);
    this->loggerMessageList.push_back(vLoggerMessage);
    lk.unlock();
    this->conditional_LoggerMessageList.notify_one();
}

void Blackboard::getLoggerMessage(s_LoggerMessage &vLoggerMessage)
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

void Blackboard::print(std::string vText)
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

void Blackboard::printMissionStatus(s_MissionStatus vMission)
{
    s_LoggerMessage message;
    this->getRobotsName(*message.robotName);
    message.operation = enum_LoggerOperation::missionStatus;
    
    //strncpy(message.buffer,vText.c_str(),sizeof(message.buffer)-1);
    memmove(message.buffer, (const unsigned char*)&vMission, sizeof(s_MissionStatus));
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
