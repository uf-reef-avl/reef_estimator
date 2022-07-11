#ifndef REEF_ESTIMATOR_ESTIMATOR_H
#define REEF_ESTIMATOR_ESTIMATOR_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace reef_estimator
{
    class Estimator
    {
    protected:
        Eigen::MatrixXd B;
        Eigen::MatrixXd G;
        Eigen::MatrixXd I;

    public:
        Eigen::MatrixXd K;
        Eigen::MatrixXd F;
        Eigen::MatrixXd f;
        Eigen::MatrixXd P;
        Eigen::MatrixXd u;
        Eigen::MatrixXd z; //Measurement vector
        Eigen::MatrixXd xHat;
        Eigen::MatrixXd H;
        Eigen::MatrixXd h;
        Eigen::MatrixXd R;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd Q0;

        Eigen::MatrixXd P0forFlying;
        Eigen::MatrixXd xHat0;
        Eigen::MatrixXd xHatforFlying;
        Eigen::MatrixXd P0;
        Eigen::MatrixXd R0;
        Eigen::MatrixXd RforFlying;

        //Alpha/beta parameters for partial update
        Eigen::MatrixXd alphaVector;
        Eigen::MatrixXd betaVector; 
        Eigen::MatrixXd gammas;

        //Timestep for discrete integration
        double dt;

        Estimator();
        ~Estimator();
        void initialize();
        void propagate();
        void update();
        void partialUpdate();
    };
}

#endif //REEF_ESTIMATOR_Z_ESTIMATOR_H
