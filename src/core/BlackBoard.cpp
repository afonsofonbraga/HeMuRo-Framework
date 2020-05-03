#include "BlackBoard.hpp"

BlackBoard::BlackBoard(std::string& name)
{
    this->position.x = 0;
    this->position.y = 0;
    this->position.theta = 0;
    //std::string nome = {"Thor"};
    setRobotsName(&name);
}

BlackBoard::~BlackBoard(){

}

void BlackBoard::getPositionAssignment(s_pose& p)
{
    this->mutex_position.lock();
            //start = std::chrono::high_resolution_clock::now();
        p.x = position.x;
        p.y = position.y;
        p.theta = position.theta;
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getPositionAssignment time: " << duration << std::endl;
        
    this->mutex_position.unlock();
}

void BlackBoard::getPosition(s_pose& p)
{
    this->mutex_position.lock();
            //start = std::chrono::high_resolution_clock::now();
        std::memcpy(&p, &position, sizeof(position));
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getPositionMemcpy time: " << duration << std::endl;
    this->mutex_position.unlock();
}

void BlackBoard::setPosition(s_pose& p)
{
    this->mutex_position.lock();
        std::memcpy(&this->position, &p, sizeof(position));
    this->mutex_position.unlock();
}

void BlackBoard::setAllRobotsPosition(s_robotsPose &p)
{
    this->mutex_mapRobotsPosition.lock();
            //start = std::chrono::high_resolution_clock::now();
        this->mapRobotsPosition.insert_or_assign(p.robotName, p.position);
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "set " << p.robotName << " time: " << duration << std::endl;
    this->mutex_mapRobotsPosition.unlock();
}

void BlackBoard::removeAllRobotsPosition(s_robotsPose &p)
{
    this->mutex_mapRobotsPosition.lock();
            //start = std::chrono::high_resolution_clock::now();
        this->mapRobotsPosition.erase(p.robotName);
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "Remove " << p.robotName << " time: " << duration << std::endl;
    this->mutex_mapRobotsPosition.unlock();
}

void BlackBoard::getAllRobotsPosition(std::unordered_map<std::string, s_pose> &p)
{
    this->mutex_mapRobotsPosition.lock();
            //start = std::chrono::high_resolution_clock::now();
        memcpy(&p, &this->mapRobotsPosition, sizeof(mapRobotsPosition));
            //end = std::chrono::high_resolution_clock::now();
            //duration = std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count();
            //std::cout << "getAllRobotsPosition time: " << duration << std::endl;
    this->mutex_mapRobotsPosition.unlock();
}


void BlackBoard::setRobotsName(std::string* name)
{
    this->robotName = *name;
}

void BlackBoard::getRobotsName(std::string& name)
{
    name = this->robotName;
}
