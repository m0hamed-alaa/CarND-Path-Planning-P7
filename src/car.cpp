#include <iostream>
#include <vector>
#include <math.h>
#include "car.h"



using namespace std;

Car::Car()
{

}

Car::Car(std::vector<double>& fusion_data)
{
    this->id = fusion_data[0];
    this->x = fusion_data[1];
    this->y = fusion_data[2];
    this->vx = fusion_data[3];
    this->vy = fusion_data[4];
    this->s = fusion_data[5];
    this->d = fusion_data[6];

    this->calculate_speed();
    this->determine_lane();


}

Car::~Car()
{

}

void Car::calculate_speed()
{
    this->speed = sqrt(this->vx*this->vx + this->vy*this->vy) ; 
}

void Car::determine_lane()
{
    if((this->d > 0) && (this->d < 4))
    {   
        this->lane = 0;
    }
    else if((this->d > 4) && (this->d < 8))
    {   
        this->lane = 1;
    }
    else if((this->d > 8) && (this->d < 12))
    {   
        this->lane = 2;
    }
}