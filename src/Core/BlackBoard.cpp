#include "BlackBoard.hpp"

BlackBoard::BlackBoard(std::string& name)
{
    this->position.x = 0;
    this->position.y = 0;
    this->position.theta = 0;
    this->batteryLevel = 56;
    setRobotsName(&name);
}

BlackBoard::~BlackBoard(){
    this->conditional_task.notify_all();
}

BlackBoard::BlackBoard(const BlackBoard& other)
{
    this->batteryLevel = other.batteryLevel;
    this->position.x = other.position.x;
    this->position.y = other.position.y;
    this->position.theta = other.position.theta;
    std::string* vName = new std::string{other.robotName};
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

float BlackBoard::getBaterryLevel()
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
        std::memcpy(&p, &position, sizeof(position));
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
        std::memcpy(&this->position, &p, sizeof(position));
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

void BlackBoard::addTask(Task& vTask)
{
    std::unique_lock<std::mutex> lk(mutex_task);
    //this->mutex_task.lock();
        this->taskList.push_back(vTask);
    //this->mutex_task.unlock();
    lk.unlock();
    this->conditional_task.notify_one();
}
void BlackBoard::getTask(Task& vTask)
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
