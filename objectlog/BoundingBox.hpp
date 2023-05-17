#include <chrono>

class BoundingBox{
    private:

    //int xCenter is position 0
    //int yCenter is position 1
    //int depthValue is position 2
    int position[3];

    int width;
    int height;
    
    std::chrono::time_point<std::chrono::system_clock> creation;

    public:

    BoundingBox(int x, int y, int w, int h);
    //Would default destructor be sufficient?

    void setXCenter(int x);
    void setYCenter(int y);
    void setDepthValue(int d);
    void setWidth(int w);
    void setHeight(int h);

    int* getXCenter();
    int* getYCenter();
    int* getDepth();
    int* getWidth();
    int* getHeight();
    int* getPosition();

};
