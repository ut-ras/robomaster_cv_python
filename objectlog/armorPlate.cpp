#include <iostream>
#include <fstream>
#include "ArmorPlate.hpp"

ArmorPlate::ArmorPlate(BoundingBox* b, int i){
    id = i;
    box = b;
    position = b->getPosition();
    velocity[0] = 0; velocity[1] = 0; velocity[2];
    acceleration[0] = 0; acceleration[1] = 0; acceleration[2] = 0;
    activity = true;
    timeBuffer = 0;
    nextPosition[0] = 0; nextPosition[1] = 0; nextPosition[2] = 0;
    maxAssocPlates = 5;
    //TODO: create prediction object
}

ArmorPlate::~ArmorPlate(){
    delete box;
}

void ArmorPlate::updateVA(){
    //TODO: fill in this method
}

void ArmorPlate::predictPosition(std::chrono::time_point<std::chrono::system_clock> currentTime){
    //TODO: fill in this method
}

int* ArmorPlate::getPosition(){
    return position;
}

int* ArmorPlate::getID(){
    return &id;
}

int* ArmorPlate::getNextPosition(){
    return nextPosition;
}

BoundingBox* ArmorPlate::getBoundingBox(){
    return box;
}

std::chrono::time_point<std::chrono::system_clock>* ArmorPlate::getLastTime(){
    return &lastTime;
}

bool* ArmorPlate::getActivity(){
    return &activity;
}

void ArmorPlate::setActivity(bool a){
    activity = a;
}

void ArmorPlate::addArmorPlate(ArmorPlate* newPlate, std::chrono::time_point<std::chrono::system_clock> currentTime){
    if(assocPlates.size() == maxAssocPlates){
        assocPlates.pop_back();
    }
    assocPlates.insert(assocPlates.begin(),newPlate);
    //TODO: Call kinematic update with prediction here
    lastTime = currentTime;
}

void ArmorPlate::writeToHistory(std::string* historyFile){
    //There might be a more optimal way to do this
    std::ofstream history;
    history.open(*historyFile);
    history << "ID: " << id << " X: " << position[0] << " Y: " << position[1] << " Depth: " << position[2];
    history << " Activity: " << activity;
    //TODO: fix the printing for the time stamps
    // " Last Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(lastTime).count();
    history.close();
}
