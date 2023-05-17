#include "BoundingBox.hpp"
#include <chrono>

BoundingBox::BoundingBox(int x, int y, int w, int h){
    position[0] = x; //xCenter
    position[1] = y; //yCenter
    position[2] = 0; //depth
    width = w;
    height = h;
    creation = std::chrono::system_clock::now();
}

void BoundingBox::setXCenter(int x){
    position[0] = x;
}

void BoundingBox::setYCenter(int y){
    position[1] = y;
}

void BoundingBox::setDepthValue(int d){
    position[2] = d;
}

void BoundingBox::setWidth(int w){
    width = w;
}

void BoundingBox::setHeight(int h){
    height = h;
}

int* BoundingBox::getXCenter(){
    return &position[0];
}

int* BoundingBox::getYCenter(){
    return &position[1];
}

int* BoundingBox::getDepth(){
    return &position[2];
}

int* BoundingBox::getWidth(){
    return &width;
}

int* BoundingBox::getHeight(){
    return &height;
}

int* BoundingBox::getPosition(){
    return position;
}