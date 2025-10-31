//Libraries
#include <pigpiod_if2.h>
#include <iostream>

//Dependencies
#include "Turret.hpp"

Turret::Turret(int pan_pin, int tilt_pin)
    : pi(pigpio_start(NULL, NULL)),
      pan(pan_pin, pi),
      tilt(tilt_pin, pi)
{
    if (pi < 0) {
        throw std::runtime_error("Failed to connect to pigpio daemon");
    }
    std::cout << "Turret connected to daemon (handle: " << pi << ")" << std::endl;
}

Turret::~Turret() {
    std::cout << "Shutting down turret..." << std::endl;
    // ServoController destructors will center each servo
    // Then we disconnect from daemon
    pigpio_stop(pi);
}

void Turret::setPanAngle(double degrees) {
    pan.setAngle(degrees);
}

void Turret::setPanPulseWidth(int microseconds){
    pan.setPulseWidth(microseconds);
}

void Turret::setTiltAngle(double degrees) {
    tilt.setAngle(degrees);
}

void Turret::setTiltPulseWidth(int microseconds){
    tilt.setPulseWidth(microseconds);
}

void Turret::centerAll() {
    pan.center();
    tilt.center();
}

double Turret::getPanAngle() const {
    return pan.getCurrentAngle();
}

double Turret::getTiltAngle() const {
    return tilt.getCurrentAngle();
}