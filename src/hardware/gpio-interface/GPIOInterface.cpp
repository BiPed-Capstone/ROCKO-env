#include "include/rocko_env/GPIOInterface.hpp"


namespace rocko_env
{

    int GPIOInterface::setupPin(string pinName, bool isOut) {
        // pack the args
        // PyObject* args = Py_BuildValue(pinName.c_str(), isOut);

        // Call the function
        PyObject* result = PyObject_CallFunction(setupPinFunc, "sb", pinName.c_str(), isOut);
        PyErr_Print();

        // Parse the response

        // Free the memory holding objects we are done with
        // Py_DECREF(result);
        return PyLong_AsLong(result);
    }
}