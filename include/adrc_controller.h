#pragma once

#include <math.h>

class AdrcController
{
    public:
        AdrcController(double delta_t, double b): _dt(delta_t), _b(b)
        {
        }

        void set_beta(double beta_0, double beta_1, double beta_2)
        {
            _beta[0] = beta_0;
            _beta[1] = beta_1;
            _beta[2] = beta_2;
        }

        void eso_nonlinear(double y, double u, double compensate=0, double* beta=nullptr)
        {
            if (beta==nullptr)
            {
                _beta[0] = 1;
                _beta[1] = 1/(2 * pow(_dt, 0.5));
                _beta[2] = 2/(25 * pow(_dt, 1.2));
            }
            double e = _z1 - y;
            _z1 = _z1 + _dt * (_z2 - _beta[0] * e);
            _z2 = _z2 + _dt * (_z3 + _b * u - _beta[1] * fe(e) + compensate);
            _z3 = _z3 + _dt * (- _beta[2] * fe(e));
        }

        void eso_linear_2d(double y, double u, double compensate=0, double* beta=nullptr)
        {
            if (beta==nullptr)
            {
                _beta[0] = 1;
                _beta[1] = 1/(3 * _dt);
            }

            double e = _z1 - y;
            _z1 = _z1 + _dt * (_z2 + _b * u - _beta[0] * e + compensate);
            _z2 = _z2 + _dt * (- _beta[1] * e);
        }

        void eso_linear(double y, double u, double compensate=0, double* beta=nullptr)
        {
            if (beta==nullptr)
            {
                _beta[0] = 1;
                _beta[1] = 1/(3 * _dt);
                _beta[2] = 2/(64 * _dt * _dt);
            }

            double e = _z1 - y;
            _z1 = _z1 + _dt * (_z2 - _beta[0] * e);
            _z2 = _z2 + _dt * (_z3 + _b * u - _beta[1] * e + compensate);
            _z3 = _z3 + _dt * (- _beta[2] * e);
        }

        double get_z1()
        {
            return _z1;
        }

        double get_z2()
        {
            return _z2;
        }
    
    private:
        double fe(double e)
        {
            return fal(e, 0.25, _delta);
        }

        double fal(double e, double alpha=0.1, double delta=0.01)
        {
            double result;
            if (abs(e) < delta)
            {
                result = e/(pow(delta, 1-alpha));
            }
            else
            {
                result = pow(abs(e), alpha)*sign(e);
            }
            return result;
        }

        double sign(double x)
        {
            if (x > 0)
                return 1;
            else if (x < 0)
                return -1;
            else 
                return 0;   
        }

        const double _dt = 0.02;
        double _delta = 0.01;
        double _b = 1;
        double _z1 = 0;
        double _z2 = 0;
        double _z3 = 0;
        double _beta[3] = {1, 1/(2 * pow(_dt, 0.5)), 2/(25 * pow(_dt, 1.2))};
};
