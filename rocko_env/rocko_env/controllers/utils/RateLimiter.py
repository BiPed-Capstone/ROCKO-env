# Direct python port of RateLimiter.hpp by Enrique Fernández from the ros-controls control_toolbox/
# Comments are brought directly over. This is our shitty unofficial port. 

import math

class RateLimiter:
    """
    * \brief Constructor
    *
    * \param [in] min_value Minimum value, e.g. [m/s], usually <= 0
    * \param [in] max_value Maximum value, e.g. [m/s], usually >= 0
    * \param [in] min_first_derivative_neg Minimum first_derivative, negative value, e.g. [m/s^2], usually <= 0
    * \param [in] max_first_derivative_pos Maximum first_derivative, positive value, e.g. [m/s^2], usually >= 0
    * \param [in] min_first_derivative_pos Asymmetric Minimum first_derivative, positive value, e.g. [m/s^2], usually <= 0
    * \param [in] max_first_derivative_neg Asymmetric Maximum first_derivative, negative value, e.g. [m/s^2], usually >= 0
    * \param [in] min_second_derivative Minimum second_derivative, e.g. [m/s^3], usually <= 0
    * \param [in] max_second_derivative Maximum second_derivative, e.g. [m/s^3], usually >= 0
    *
    * \note
    * If max_* values are NAN, the respective limit is deactivated
    * If min_* values are NAN (unspecified), defaults to -max
    * If min_first_derivative_pos/max_first_derivative_neg values are NAN, symmetric limits are used
    *
    * Disclaimer about the jerk limits:
    *    The jerk limit is only applied when accelerating or reverse_accelerating (i.e., "sign(jerk * accel) > 0").
    *    This condition prevents oscillating closed-loop behavior, see discussion details in
    *    https://github.com/ros-controls/control_toolbox/issues/240.
    *    if you use this feature, you should perform a test to check that the behavior is really as you expect.
    *
    """
    def __init__(self, min_value=math.nan, max_value=math.nan,
                 min_first_derivative_neg=math.nan, max_first_derivative_pos=math.nan,
                 min_first_derivative_pos=math.nan, max_first_derivative_neg=math.nan,
                 min_second_derivative=math.nan, max_second_derivative=math.nan):
        self.set_params(min_value, max_value,
                        min_first_derivative_neg, max_first_derivative_pos,
                        min_first_derivative_pos, max_first_derivative_neg,
                        min_second_derivative, max_second_derivative)
    """
        /**
        * \brief Set the parameters
        *
        * \param [in] min_value Minimum value, e.g. [m/s], usually <= 0
        * \param [in] max_value Maximum value, e.g. [m/s], usually >= 0
        * \param [in] min_first_derivative_neg Minimum first_derivative, negative value, e.g. [m/s^2], usually <= 0
        * \param [in] max_first_derivative_pos Maximum first_derivative, positive value, e.g. [m/s^2], usually >= 0
        * \param [in] min_first_derivative_pos Asymmetric Minimum first_derivative, positive value, e.g. [m/s^2], usually <= 0
        * \param [in] max_first_derivative_neg Asymmetric Maximum first_derivative, negative value, e.g. [m/s^2], usually >= 0
        * \param [in] min_second_derivative Minimum second_derivative, e.g. [m/s^3], usually <= 0
        * \param [in] max_second_derivative Maximum second_derivative, e.g. [m/s^3], usually >= 0
        *
        * \note
        * If max_* values are NAN, the respective limit is deactivated
        * If min_* values are NAN  (unspecified), defaults to -max
        * If min_first_derivative_pos/max_first_derivative_neg values are NAN, symmetric limits are used
        */
    """
    def set_params(self, min_value=math.nan, max_value=math.nan,
                   min_first_derivative_neg=math.nan, max_first_derivative_pos=math.nan,
                   min_first_derivative_pos=math.nan, max_first_derivative_neg=math.nan,
                   min_second_derivative=math.nan, max_second_derivative=math.nan):
        # Value Limits
        self.min_value = min_value
        self.max_value = max_value
        self.has_value_limits = not math.isnan(self.max_value)
        if self.has_value_limits and math.isnan(self.min_value):
            self.min_value = -self.max_value
        if self.has_value_limits and self.min_value > self.max_value:
            raise ValueError("Invalid value limits")

        # First Derivative Limits
        self.min_first_derivative_neg = min_first_derivative_neg
        self.max_first_derivative_pos = max_first_derivative_pos
        self.min_first_derivative_pos = min_first_derivative_pos
        self.max_first_derivative_neg = max_first_derivative_neg
        self.has_first_derivative_limits = not math.isnan(self.max_first_derivative_pos)
        if self.has_first_derivative_limits:
            if math.isnan(self.min_first_derivative_neg):
                self.min_first_derivative_neg = -self.max_first_derivative_pos
            if math.isnan(self.max_first_derivative_neg):
                self.max_first_derivative_neg = self.max_first_derivative_pos
            if math.isnan(self.min_first_derivative_pos):
                self.min_first_derivative_pos = self.min_first_derivative_neg
            if (self.min_first_derivative_neg > self.max_first_derivative_pos or
                self.min_first_derivative_pos > self.max_first_derivative_neg):
                raise ValueError("Invalid first derivative limits")

        # Second Derivative Limits
        self.min_second_derivative = min_second_derivative
        self.max_second_derivative = max_second_derivative
        self.has_second_derivative_limits = not math.isnan(self.max_second_derivative)
        if self.has_second_derivative_limits and math.isnan(self.min_second_derivative):
            self.min_second_derivative = -self.max_second_derivative
        if self.has_second_derivative_limits and self.min_second_derivative > self.max_second_derivative:
            raise ValueError("Invalid second derivative limits")

    def limit(self, v, v0, v1, dt):
        original_v = v
        v = self.limit_second_derivative(v, v0, v1, dt)
        v = self.limit_first_derivative(v, v0, dt)
        v = self.limit_value(v)
        return v

    def limit_value(self, v):
        if self.has_value_limits:
            v = max(self.min_value, min(v, self.max_value))
        return v

    def limit_first_derivative(self, v, v0, dt):
        if self.has_first_derivative_limits:
            if v0 > 0.0:
                dv_max = self.max_first_derivative_pos * dt
                dv_min = self.min_first_derivative_pos * dt
            elif v0 < 0.0:
                dv_min = self.min_first_derivative_neg * dt
                dv_max = self.max_first_derivative_neg * dt
            else:
                dv_min = self.min_first_derivative_neg * dt
                dv_max = self.max_first_derivative_pos * dt

            dv = v - v0
            dv_clamped = max(dv_min, min(dv, dv_max))
            v = v0 + dv_clamped
        return v
    
    """ Only limit jerk when accelerating or reverse_accelerating
    // Note: this prevents oscillating closed-loop behavior, see discussion
    // details in https://github.com/ros-controls/control_toolbox/issues/240."""
    def limit_second_derivative(self, v, v0, v1, dt):
        if self.has_second_derivative_limits:
            dv = v - v0
            dv0 = v0 - v1
            if (dv - dv0) * (v - v0) > 0:  # Accelerating or reverse-accelerating
                dt2 = dt * dt
                da_min = self.min_second_derivative * dt2
                da_max = self.max_second_derivative * dt2
                da = dv - dv0
                da_clamped = max(da_min, min(da, da_max))
                v = v0 + dv0 + da_clamped
        return v