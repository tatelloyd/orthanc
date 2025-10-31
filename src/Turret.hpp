#ifndef TURRET_HPP
#define TURRET_HPP

//Dependencies
#include "ServoController.hpp"

class Turret {
public:
    //Constructor
    Turret(int pan_pin, int tilt_pin);
    //Destructor
    ~Turret();
    
    //Setters
    void setPanAngle(double degrees);
    void setPanPulseWidth(int microseconds);
    void setTiltAngle(double degrees);
    void setTiltPulseWidth(int microseconds);

    //Getters
    double getPanAngle() const;
    double getTiltAngle() const;

    //Reset servo positions
    void centerAll();
private:
    int pi;  //Daemon handle
    ServoController pan;
    ServoController tilt;   
};

#endif // TURRET_HPP