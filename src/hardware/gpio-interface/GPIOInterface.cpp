#include "include/rocko_env/GPIOInterface.hpp"


namespace rocko_env
{
    void GPIOInterface::setupPin(string pinName, bool isOut) {
        // Call the function
        PyObject_CallFunction(setupPinFunc, "sb", pinName.c_str(), isOut);
        PyErr_Print();
    }

    void GPIOInterface::cleanup() {
        PyObject_CallFunction(cleanupFunc, "");
    }

    void GPIOInterface::startPWM(string pinName, int dutyCycle, int freq, bool isFallingEdge) {

    }
        
    void GPIOInterface::setDutyCycle(string pinName, int dutyCycle) {

    }
        
    void GPIOInterface::stop(string pinName) {

    }
}
