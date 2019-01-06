#ifndef CAR_H_
#define CAR_H_

#include <vector>

class Car
{

    public :

        int lane;
        double x;
        double y;
        double vx;
        double vy;
        double speed;
        double s;
        double d;

        Car();
        Car(std::vector<double>& fusion_data);
        virtual ~Car();
        void calculate_speed();
        void determine_lane();
};








#endif