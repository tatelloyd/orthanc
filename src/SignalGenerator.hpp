#ifndef SIGNAL_GENERATOR_HPP
#define SIGNAL_GENERATOR_HPP

// Class can produce several different types of signals.
class SignalGenerator {
    public:
        SignalGenerator(double amp, double per, double off);

    // Output a basic square wave
    double square(double t) const;

    // Output a sine wave
    double sine(double t) const;

    // Output a sawtooth wave
    double triangle(double t) const;

    private:
        double amplitude;
        double period;
        double offset;
};

#endif // SIGNAL_GENERATOR_HPP