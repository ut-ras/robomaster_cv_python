#include <vector>
#include "ArmorPlate.hpp"

class ObjectLog{
    private:
    //These could potentially be made const
    //we still need to fill these in with the correct values
    int maxX;
    int maxY;
    int maxZ;
    int minX;
    int minY;
    int minZ;
    int minArea;
    int margainOfError;
    int killThreshold;

    string outputFile;
    std::vector<ArmorPlate*> plates;
    int idAssign;

    public:

    ObjectLog();
    ~ObjectLog();

    int boxesInput(std::vector<BoundingBox*> boxes, std::chrono::time_point<std::chrono::system_clock> time);
    bool sizeCheck(BoundingBox* box);
    //In the python version, assignPlates also takes in plates as an input
    //however plates is a local variable so it has been omitted
    int assignPlate(BoundingBox* box);
    double getDistance(int* pointOne, int* pointTwo);
    void killPlate(int ID);
    void killAll();
    std::vector<ArmorPlate*>* getPlates();
    
};