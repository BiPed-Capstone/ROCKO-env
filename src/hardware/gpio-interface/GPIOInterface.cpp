#include "include/rocko_env/GPIOInterface.hpp"


namespace rocko_env
{
    bool GPIOInterface::setupPin(string pinName, bool isOut) {
        // Call the function
        PyObject *result = PyObject_CallFunction(setupPinFunc, "sb", pinName.c_str(), isOut);
        if (result == NULL) {
            PyErr_Print();
            return false;
        }
        return true;
    }

    bool GPIOInterface::cleanup() {
        PyObject *result = PyObject_CallFunction(cleanupFunc, "");
        if (result == NULL) {
            PyErr_Print();
            return false;
        }
        return true;
    }

    void GPIOInterface::startPWM(string pinName, int dutyCycle, int freq, bool isFallingEdge) {

    }
        
    void GPIOInterface::setDutyCycle(string pinName, int dutyCycle) {

    }
        
    void GPIOInterface::stop(string pinName) {

    }
}
