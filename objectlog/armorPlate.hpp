#include <chrono>
#include <vector>
#include "BoundingBox.hpp"

class ArmorPlate{
    private:
    BoundingBox * box;
    int* position;
    int id;
    int velocity[3];
    int acceleration[3];
    bool activity;
    int timeBuffer;
    int nextPosition[3];
    std::chrono::time_point<std::chrono::system_clock> lastTime;
    std::vector<ArmorPlate*> assocPlates;
    int maxAssocPlates;
    //TODO: add prediction object here

    public:
    ArmorPlate(BoundingBox* b, int i);
    ~ArmorPlate();

    void updateVA();
    void predictPosition(std::chrono::time_point<std::chrono::system_clock> currentTime);
    int* getPosition();
    int* getID();
    int* getNextPosition();
    BoundingBox* getBoundingBox();
    std::chrono::time_point<std::chrono::system_clock>* getLastTime();
    bool* getActivity();
    void setActivity(bool a);
    void addArmorPlate(ArmorPlate* newPlate, std::chrono::time_point<std::chrono::system_clock> currentTime);
    //TODO: add writeToHistory

};