#include "BlackBoard.hpp"

BlackBoard::BlackBoard(std::string& name)
{
    this->position.x = 0;
    this->position.y = 0;
    this->position.theta = 0;
    this->batteryLevel = 56;
    setRobotsName(&name);
    char vname[16] = "10.0.0.103";
    setRobotIP(vname);
}

BlackBoard::~BlackBoard(){
    this->conditional_missionTask.notify_all();
}

BlackBoard::BlackBoard(const BlackBoard& other)
{
    this->batteryLevel = other.batteryLevel;
    this->position.x = other.position.x;
    this->position.y = other.position.y;
    this->position.theta = other.position.theta;
    std::string* vName = new std::string{other.robotName};
    char vIP[16];
    strcpy(vIP, other.robotIP);
    this->setRobotIP(vIP);
    //other.getRobotsName(*vName);
    this->setRobotsName(vName);
    delete vName;
}

BlackBoard& BlackBoard::operator=(const BlackBoard& other)
{
    if(this != &other) {
        this->batteryLevel = other.batteryLevel;
        this->position.x = other.position.x;
        this->position.y = other.position.y;
        this->position.theta = other.position.theta;
        std::string* vName = new std::string{other.robotName};
        this->setRobotsName(vName);
        char vIP[16];
        strcpy(vIP, other.robotIP);
        this->setRobotIP(vIP);
    }
    return *this;
}

//****************************************************
//*               Robot's Description                *
//****************************************************


void BlackBoard::setRobotsName(std::string* name)
{
    this->robotName = *name;
}

void BlackBoard::getRobotsName(std::string& name)
{
    name = this->robotName;
}

void BlackBoard::setRobotIP(char* vIP)
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {

        if (ifa->ifa_addr && ifa->ifa_addr->sa_family==AF_INET && strstr(ifa->ifa_name,"lo")==nullptr) {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            
            //addr = inet_ntoa(sa->sin_addr);
            strcpy(this->robotIP, inet_ntoa(sa->sin_addr));
            //printf("Interface: %s\tAddress: %s\n", ifa->ifa_name, addr);
            break;
        }
    }
    freeifaddrs(ifap);
    strncpy(this->broadcastIP, this->robotIP, sizeof(robotIP));
    char* lastDot = strrchr(this->broadcastIP,'.');
    strcpy(lastDot + 1, "255");
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
    //this->mutex_battery.lock();
    std::unique_lock<std::mutex> lk(mutex_battery);
        this->batteryLevel + energy < 100.0 ? this->batteryLevel += energy : this->batteryLevel = 100.0;
    //this->mutex_battery.unlock();
    lk.unlock();
}

void BlackBoard::consumeBattery(float energy)
{
    //this->mutex_battery.lock();
    std::unique_lock<std::mutex> lk(mutex_battery);
        this->batteryLevel - energy > 0.0 ? this->batteryLevel -= energy : this->batteryLevel = 0;
    //this->mutex_battery.unlock();
    lk.unlock();
}

float BlackBoard::getBatteryLevel()
{
    float vBattery;
    std::unique_lock<std::mutex> lk(mutex_battery);
    //this->mutex_battery.lock();
        vBattery = this->batteryLevel;
    //this->mutex_battery.unlock();
    lk.unlock();
    return vBattery;
}


//****************************************************
//*           Robot's Position Variables             *
//****************************************************


void BlackBoard::getPositionAssignment(s_pose& p)
{
    //this->mutex_position.lock();
    std::unique_lock<std::mutex> lk(mutex_position);
            //start = std::chrono::high_resolution_clock::now();
        p.x = position.x;
        p.y = position.y;
        p.theta = position.theta;
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getPositionAssignment time: " << duration << std::endl;
    lk.unlock();
    //this->mutex_position.unlock();
}

void BlackBoard::getPosition(s_pose& p)
{
    //this->mutex_position.lock();
    std::unique_lock<std::mutex> lk(mutex_position);
            //start = std::chrono::high_resolution_clock::now();
        memcpy(&p, &position, sizeof(position));
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getPositionMemcpy time: " << duration << std::endl;
    //this->mutex_position.unlock();
    lk.unlock();
}

void BlackBoard::setPosition(s_pose& p)
{
    //this->mutex_position.lock();
    std::unique_lock<std::mutex> lk(mutex_position);
        memcpy(&this->position, &p, sizeof(position));
    //this->mutex_position.unlock();
    lk.unlock();
}

void BlackBoard::setAllRobotsPosition(s_robotsPose &p)
{
    //this->mutex_mapRobotsPosition.lock();
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
            //start = std::chrono::high_resolution_clock::now();
        this->mapRobotsPosition.insert_or_assign(p.robotName, p.position);
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "set " << p.robotName << " time: " << duration << std::endl;
    //this->mutex_mapRobotsPosition.unlock();
    lk.unlock();
}

void BlackBoard::removeAllRobotsPosition(s_robotsPose &p)
{
    //this->mutex_mapRobotsPosition.lock();
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
            //start = std::chrono::high_resolution_clock::now();
        this->mapRobotsPosition.erase(p.robotName);
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "Remove " << p.robotName << " time: " << duration << std::endl;
    //this->mutex_mapRobotsPosition.unlock();
    lk.unlock();
}

void BlackBoard::getAllRobotsPosition(std::unordered_map<std::string, s_pose> &p)
{
    //this->mutex_mapRobotsPosition.lock();
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
            //start = std::chrono::high_resolution_clock::now();
        memcpy(&p, &this->mapRobotsPosition, sizeof(mapRobotsPosition));
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getAllRobotsPosition time: " << duration << std::endl;
    //this->mutex_mapRobotsPosition.unlock();
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
//*            Task Related Functions                *
//****************************************************
/*
void BlackBoard::addTask(AtomicTask& vTask)
{
    std::unique_lock<std::mutex> lk(mutex_task);
    //this->mutex_task.lock();
        this->taskList.push_back(vTask);
    //this->mutex_task.unlock();
    lk.unlock();
    this->conditional_task.notify_one();
}

void BlackBoard::getTask(AtomicTask& vTask)
{
    std::unique_lock<std::mutex> lk(mutex_task);
    
    if (this->taskList.empty() == false){
        //this->mutex_task.lock();
            vTask = this->taskList.front();
        this->taskList.erase(taskList.begin());
        lk.unlock();
        //this->mutex_task.unlock();
    }else
    {
        this->conditional_task.wait(lk);
        
        if (this->taskList.empty() == false){
            vTask = this->taskList.front();
            this->taskList.erase(taskList.begin());
            lk.unlock();
        }
    }
}

bool BlackBoard::isTaskListEmpty()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_task);
    //this->mutex_task.lock();
        status = this->taskList.empty();
    //this->mutex_task.unlock();
    lk.unlock();
    return status;
}
*/

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
            //vAtomicTask = *search->second;
            //memcpy(&vAtomicTask, &search->second, sizeof(*search->second));
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

float BlackBoard::getDecomposableTaskCost(enum_DecomposableTask vTaskToBeDecomposed)
{
    return 0.0;
}

void BlackBoard::acceptDecomposableTask(enum_DecomposableTask vDecomposableTask)
{
    std::unique_lock<std::mutex> lk(mutex_decomposableTask);
        
    lk.unlock();
}

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
    if(this->selectedMission.enum_execution == enum_MissionExecution::missionComplete)
        status = true;
    lk.unlock();
    return status;
}

bool BlackBoard::isRobotAvailable()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    if (this->selectedMission.enum_execution == enum_MissionExecution::missionComplete || this->selectedMission.enum_execution == enum_MissionExecution::null)
        status = true;
    lk.unlock();
    return status;
}

void BlackBoard::addMissionToExecute(Mission& vMission)
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    if (this->selectedMission.enum_execution == enum_MissionExecution::missionComplete || this->selectedMission.enum_execution == enum_MissionExecution::null) // Is necessary to check if the previous mission is already completed.
    {
        this->selectedMission = vMission;
        this->selectedMission.atomicTaskIndex = 0;
        this->selectedMission.enum_execution = enum_MissionExecution::waitingStart;
        //this->conditional_missionTask.notify_one();
    }
    lk.unlock();
}

void BlackBoard::getTaskFromMission(AtomicTask& vTask)
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    
    if(this->selectedMission.atomicTaskIndex == this->selectedMission.atomicTaskList.size() && this->selectedMission.atomicTaskList.size() != 0)
    {
        std::cout << "Mission Complete!!!!" << std::endl;
        this->selectedMission.enum_execution = enum_MissionExecution::missionComplete;
    }
    
    if (this->selectedMission.enum_execution == enum_MissionExecution::executing){
        vTask = this->selectedMission.atomicTaskList.at(this->selectedMission.atomicTaskIndex);
        this->selectedMission.atomicTaskIndex++;
        lk.unlock();
    }else
    {
        this->conditional_missionTask.wait(lk);
        
        if (this->selectedMission.enum_execution == enum_MissionExecution::executing){
            vTask = this->selectedMission.atomicTaskList.at(this->selectedMission.atomicTaskIndex);
            this->selectedMission.atomicTaskIndex++;
        lk.unlock();
        }
    }
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

