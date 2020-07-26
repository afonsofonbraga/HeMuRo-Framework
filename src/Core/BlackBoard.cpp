#include "BlackBoard.hpp"

BlackBoard::BlackBoard(std::string& name)
{
    this->position.x = 0;
    this->position.y = 0;
    this->position.theta = 0;
    this->batteryLevel = 56;
    setRobotsName(&name);
    this->setRobotIP();
    
}

BlackBoard::~BlackBoard(){
}

BlackBoard::BlackBoard(const BlackBoard& other)
{
    this->batteryLevel = other.batteryLevel;
    this->position.x = other.position.x;
    this->position.y = other.position.y;
    this->position.theta = other.position.theta;
    std::string* vName = new std::string{other.robotName};
    this->setRobotIP();
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
        this->setRobotIP();
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
        memcpy(&this->position, &p, sizeof(position));
    lk.unlock();
}

void BlackBoard::setAllRobotsPosition(s_robotsPose &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
            //start = std::chrono::high_resolution_clock::now();
        this->mapRobotsPosition.insert_or_assign(p.robotName, p.position);
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "set " << p.robotName << " time: " << duration << std::endl;
    lk.unlock();
}

void BlackBoard::removeAllRobotsPosition(s_robotsPose &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
            //start = std::chrono::high_resolution_clock::now();
        this->mapRobotsPosition.erase(p.robotName);
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "Remove " << p.robotName << " time: " << duration << std::endl;
    lk.unlock();
}

void BlackBoard::getAllRobotsPosition(std::unordered_map<std::string, s_pose> &p)
{
    std::unique_lock<std::mutex> lk(mutex_mapRobotsPosition);
            //start = std::chrono::high_resolution_clock::now();
        memcpy(&p, &this->mapRobotsPosition, sizeof(mapRobotsPosition));
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getAllRobotsPosition time: " << duration << std::endl;
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
    //if(this->selectedMission.enum_execution == enum_MissionExecution::missionComplete)
    //    status = true;
    lk.unlock();
    return status;
}

bool BlackBoard::isRobotAvailable()
{
    bool status;
    std::unique_lock<std::mutex> lk(mutex_mission);
    //if (this->selectedMission.enum_execution == enum_MissionExecution::missionComplete || this->selectedMission.enum_execution == enum_MissionExecution::null)
    //    status = true;
    status = this->executingMission;
    lk.unlock();
    return status;
}

bool BlackBoard::lockRobot()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
    if (this->executingMission == false)
    {
        status = true;
        this->executingMission= true;
    } else
        status = false;
    lk.unlock();
    return status;
}

bool BlackBoard::unlockRobot()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_mission);
        if (this->executingMission == true)
        {
            status= true;
            this->executingMission= false;
        } else
            status = false;
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

