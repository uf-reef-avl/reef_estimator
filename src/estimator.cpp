#include "estimator.h"
#include "ros/ros.h"
namespace reef_estimator
{
    Estimator::Estimator() {}
    Estimator::~Estimator() {}

    void Estimator::initialize()
    {
        xHat = xHat0;
        P = P0;
        R = R0;

        //Calculate partial update alphas
        int numStates = betaVector.rows();
        alphaVector = Eigen::MatrixXd(numStates, 1);
        gammas = Eigen::MatrixXd(numStates, numStates);
        //gammas = 1 - beta
        alphaVector = Eigen::MatrixXd::Ones(numStates, 1) - betaVector;
        gammas.setZero();
        gammas.diagonal() = alphaVector;
    }

    void Estimator::propagate()
    {
        xHat = F*xHat + B*u;
        P = F*P*F.transpose() + G*Q*G.transpose();
    }

    void Estimator::update()
    {
        Eigen::MatrixXd S;
        S = (H*P*H.transpose() + R);
        K = P*H.transpose()* S.inverse();

        xHat = xHat + K*(z - H*xHat);

        P = (I - K*H)*P;
    }

    //Partial update for states based on beta (%) parameters
    //See "Partial-Update Schmidt–Kalman Filter" by Brink for details:
    //https://arc.aiaa.org/doi/10.2514/1.G002808
    void Estimator::partialUpdate()
    {
        Eigen::MatrixXd S;
        S = (H*P*H.transpose() + R);
        K = P*H.transpose()* S.inverse();

        int numStates = xHat.rows();
        Eigen::MatrixXd xHatMinus(numStates, 1);
        Eigen::MatrixXd PMinus(numStates, numStates);
        Eigen::MatrixXd PPlus(numStates, numStates);

        //Save xHat and P minus values
        xHatMinus = xHat;
        PMinus = P;

        //Perform update
        xHat = xHat + K*(z - H*xHat);
        P = (I - K*H)*P;

        //Apply partial update
        PPlus = P;

        xHat = alphaVector.cwiseProduct(xHatMinus) + betaVector.cwiseProduct(xHat);
        P = gammas * (PMinus - PPlus) * gammas + PPlus;

    }
}