#ifndef ROCKO_ENV_GPIO_INTERFACE
#define ROCKO_ENV_GPIO_INTERFACE

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

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
            if (!getPythonFunction(gpioModule, setupPinFunc, "setupPin")) {
                return;
            }
            if (!getPythonFunction(gpioModule, startPWMFunc, "startPWM")) {
                return;
            }
            if (!getPythonFunction(gpioModule, setDutyCycleFunc, "setDutyCycle")) {
                return;
            }
            if (!getPythonFunction(gpioModule, stopFunc, "stop")) {
                return;
            }

            initSuccessfulVar = true;
        }

        bool getPythonFunction(PyObject *pyModule, PyObject* func, char* name) {
            func = PyObject_GetAttrString(pyModule, name);
            if (PyCallable_Check(func) == 0) {
                initSuccessfulVar = false;
                PyErr_Print();
                return false;
            }
            return true;
        }
        GPIOInterface(GPIOInterface const&); // Don't Implement.
        void operator=(GPIOInterface const&); // Don't implement

        PyObject *setupPinFunc, *startPWMFunc, *setDutyCycleFunc, *stopFunc;
        bool initSuccessfulVar;
};
}

#endif
