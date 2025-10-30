#include <cmath>
#include "SignalGenerator.hpp"
    
SignalGenerator::SignalGenerator(double amp, double per, double off):
    amplitude(amp), period(per), offset(off) {}

// Output a basic square wave
double SignalGenerator::square(double t) const {
    // Which period are we in?
    double phase = fmod(t, period);
    
    // Square wave logic
    if (phase < period / 2.0) {
        return offset - amplitude;  // Low
    } else {
        return offset + amplitude;  // High
    }
}

// Output a sine wave
double SignalGenerator::sine(double t) const {
    return offset + amplitude * sin(2.0 * M_PI * t / period);
}

// Output a sawtooth wave
double SignalGenerator::triangle(double t) const {
    double phase = fmod(t, period);
    if (phase < period / 2.0) {
        return offset - amplitude + (4.0 * amplitude * phase / period);
    } else {
        return offset + amplitude - (4.0 * amplitude * (phase - period/2.0) / period);
    }
}