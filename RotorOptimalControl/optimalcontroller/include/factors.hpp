#pragma once


#include <ceres/ceres.h>


class ConstantVelocityFactor : public ceres::SizedCostFunction<1, 1, 1>
{
  public:
    ConstantVelocityFactor(const double &_const_velocity, const double &_weight)
    {
        const_velocity = _const_velocity;
    	weight = _weight;
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
    	*residuals = weight * (parameters[1][0] - parameters[0][0] - const_velocity);
    	if (jacobians)
    	{
    		if (jacobians[0])
    		{
    		    jacobians[0][0] = -weight;
                
    		}
            if (jacobians[1])
            {
                jacobians[1][0] = weight;
            }
    	}
    	return true;

    }

    double const_velocity;
    double weight;
};

class UncertaintyFactor : public ceres::SizedCostFunction<1, 1>
{
  public:
    UncertaintyFactor(const double &_x0, const double &_fx0, const double &_jx0, const double &_weight)
    {
        x0 = _x0;
        fx0 = _fx0;
    	jx0 = _jx0;
        weight = _weight;
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
    	double dx = (parameters[0][0] - x0);
        double fx = fx0 + dx * jx0;
        double jx = jx0;
        *residuals = weight * fx;
    	if (jacobians)
    	{
    		if (jacobians[0])
    		{
    		    jacobians[0][0] = weight * jx0;
    		}
    	}
    	return true;
    }

    double x0;
    double fx0;
    double jx0;
    double weight;
};