#ifndef ROCKO_ENV_GPIO_INTERFACE
#define ROCKO_ENV_GPIO_INTERFACE

#include <iostream>
#include <mutex>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "Python.h"

using namespace std;

namespace rocko_env
{
class GPIOInterface
{
    public:
        static GPIOInterface& getInstance()
        {
            static GPIOInterface instance;
            return instance;
        }

        bool initSuccessful() { return initSuccessfulVar; }

        // Python wrapper functions
        int setupPin(string pinName, bool isOut);
        void startPWM(string pinName, int dutyCycle, int freq, bool isFallingEdge);
        void setDutyCycle(string pinName, int dutyCycle);
        void stop(string pinName);

    private:
        GPIOInterface() {
            Py_Initialize();
            // Add the path to the python module from the cwd
            PyObject * sysPath = PySys_GetObject("path");
            PyList_Insert(sysPath, 0, PyUnicode_FromString("src/hardware/gpio-interface"));

            // Get the python module
            PyObject * gpioModule = PyImport_ImportModule("gpio_interface");
            PyErr_Print();

            // Get the python functions
            setupPinFunc = PyObject_GetAttrString(gpioModule, "setupPin");
            if (setupPinFunc == NULL) {
                PyErr_Print();
                return;
            }
        }
        GPIOInterface(GPIOInterface const&); // Don't Implement.
        void operator=(GPIOInterface const&); // Don't implement

        PyObject* setupPinFunc, startPWMFunc, setDutyCycleFunc, stopFunc;
        bool initSuccessfulVar;
};
}

#endif
