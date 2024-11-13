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

        bool init() {
            if (initSuccessfulVar) return true;
            
            Py_Initialize();
            // Add the path to the Python module from the cwd
            PyObject * sysPath = PySys_GetObject("path");
            PyList_Insert(sysPath, 0, PyUnicode_FromString("src/hardware/gpio-interface"));

            // Get the Python module
            PyObject * gpioModule = PyImport_ImportModule("gpio_interface");
            PyErr_Print();

            // Get the Python functions
            // if (!initPythonFunction(gpioModule, setupPinFunc, "setup_pin") ||
            //     !initPythonFunction(gpioModule, startPWMFunc, "start_pwm") ||
            //     !initPythonFunction(gpioModule, stopPWMFunc, "stop_pwm") ||
            //     !initPythonFunction(gpioModule, setDutyCycleFunc, "set_duty_cycle")) {
            //     return;
            // }
            setupPinFunc = PyObject_GetAttrString(gpioModule, "setup_pin");
            if (!PyCallable_Check(setupPinFunc)) {
                initSuccessfulVar = false;
                PyErr_Print();
                return false;
            }

            startPWMFunc = PyObject_GetAttrString(gpioModule, "start_pwm");
            if (!PyCallable_Check(startPWMFunc)) {
                initSuccessfulVar = false;
                PyErr_Print();
                return false;
            }

            stopPWMFunc = PyObject_GetAttrString(gpioModule, "stop_pwm");
            if (!PyCallable_Check(stopPWMFunc)) {
                initSuccessfulVar = false;
                PyErr_Print();
                return false;
            }

            setDutyCycleFunc = PyObject_GetAttrString(gpioModule, "set_duty_cycle");
            if (!PyCallable_Check(setDutyCycleFunc)) {
                initSuccessfulVar = false;
                PyErr_Print();
                return false;
            }

            initSuccessfulVar = true;
            return true;
        }

        bool initSuccessful() { return initSuccessfulVar; }

        // Python wrapper functions
        bool setupPin(string pinName, bool isOut);
        bool startPWM(string pinName, int dutyCycle, int freq, bool isFallingEdge);
        bool stopPWM(string pinName);
        bool setDutyCycle(string pinName, int dutyCycle);

    private:
        GPIOInterface() {

        }

        bool initPythonFunction(PyObject *pyModule, PyObject *func, string name) {
            func = PyObject_GetAttrString(pyModule, name.c_str());
            if (!PyCallable_Check(func)) {
                initSuccessfulVar = false;
                PyErr_Print();
                return false;
            }
            return true;
        }
        GPIOInterface(GPIOInterface const&); // Don't Implement.
        void operator=(GPIOInterface const&); // Don't implement

        PyObject *setupPinFunc, *cleanupFunc, *startPWMFunc, *stopPWMFunc, *setDutyCycleFunc;
        bool initSuccessfulVar;
};
}

#endif
