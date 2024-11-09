#include "include/rocko_env/GPIOInterface.hpp"


namespace rocko_env
{
    int GPIOInterface::setupPin(string pinName, bool isOut) {
        // Call the function
        PyObject* result = PyObject_CallFunction(setupPinFunc, "sb", pinName.c_str(), isOut);
        PyErr_Print();

        // Parse the response
        long ret = PyLong_AsLong(result);

        // Free the memory holding objects we are done with
        Py_DECREF(result);
        return ret;
    }

    void GPIOInterface::startPWM(string pinName, int dutyCycle, int freq, bool isFallingEdge) {

    }
        
    void GPIOInterface::setDutyCycle(string pinName, int dutyCycle) {

    }
        
    void GPIOInterface::stop(string pinName) {

    }
}
