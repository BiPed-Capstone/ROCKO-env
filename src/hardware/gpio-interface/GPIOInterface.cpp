#include "include/rocko_env/GPIOInterface.hpp"


namespace rocko_env
{
    bool GPIOInterface::setupPin(string pinName, bool isOut) {
        // Call the function
        PyObject *result = PyObject_CallFunction(setupPinFunc, "sb", pinName.c_str(), isOut);
        if (!PyLong_Check(result)) {
            PyErr_Print();
            return false;
        }
        return PyLong_AsLong(result) == 0;
    }

    bool GPIOInterface::startPWM(string pinName, int dutyCycle, int freq, bool isFallingEdge) {
        PyObject *result = PyObject_CallFunction(startPWMFunc, "siib", pinName.c_str(), dutyCycle, freq, isFallingEdge);
        if (!PyLong_Check(result)) {
            PyErr_Print();
            return false;
        }
        return PyLong_AsLong(result) == 0;
    }

    bool GPIOInterface::stopPWM(string pinName) {
        PyObject *result = PyObject_CallFunction(stopPWMFunc, "s", pinName.c_str());
        if (!PyLong_Check(result)) {
            PyErr_Print();
            return false;
        }
        return PyLong_AsLong(result) == 0;
    }
        
    bool GPIOInterface::setDutyCycle(string pinName, int dutyCycle) {
        PyObject *result = PyObject_CallFunction(setDutyCycleFunc, "si", pinName.c_str(), dutyCycle);

        if (!PyLong_Check(result)) {
            PyErr_Print();
            return false;
        }
        return PyLong_AsLong(result) == 0;
    }
}
