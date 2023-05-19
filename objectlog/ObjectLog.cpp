#include <math.h>
#include "ObjectLog.hpp"

ObjectLog::ObjectLog(){
    maxX = 150;
    maxY = 150;
    maxZ = 50;
    minX = -1;
    minY = -1;
    minZ = -1;
    minArea = 10;
    margainOfError = 0;
    killThreshold = -1;

    outputFile = "ObjectLog.txt";
    idAssign = 0;
}

ObjectLog::~ObjectLog(){
    for(int i = 0; i < plates.size(); i++){
        delete plates.at(i);
    }
    plates.clear();
}

int ObjectLog::boxesInput(std::vector<BoundingBox*> boxes, std::chrono::time_point<std::chrono::system_clock> time){
    //TODO: fill in this method
}

bool ObjectLog::sizeCheck(BoundingBox* box){
    return (*box->getHeight() * *box->getWidth()) > minArea;
}

int ObjectLog::assignPlate(BoundingBox* box){
    //TODO: fill in this method
}

double ObjectLog::getDistance(int* pointOne, int* pointTwo){
    return sqrt(pow(pointTwo[0]-pointOne[0],2) + pow(pointTwo[1]-pointOne[1],2) + pow(pointTwo[2]-pointOne[2],2));
}

void ObjectLog::killPlate(int ID){
    //TODO: fill in this method
}

void ObjectLog::killAll(){
    //TODO: fill in this method
}

std::vector<ArmorPlate*>* ObjectLog::getPlates(){
    return &plates;
}