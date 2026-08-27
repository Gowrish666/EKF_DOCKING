#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <cmath>

class EKF
{
    private:
       Eigen::Vector3d state_;
       Eigen::Matrix3d P_;
       Eigen::Matrix3d Q_;
       Eigen::Matrix3d R_;

       Eigen::Matrix3d baseQ_;
       Eigen::Matrix3d baseR_;

       double qScale_;
       double rScale_;

       Eigen::Vector3d residual_;

       double mahalanobisDistance_;
       bool measurementAccepted_;
    
    static double normalizeAngle(double angle)
    {
        while(angle > M_PI)
            angle -= 2.0 *M_PI;

        while (angle < -M_PI)
            angle += 2.0 *M_PI;

        return angle;
    }

    public:
       EKF()
       {
        state_.setZero();

        P_.setZero();
        P_(0,0) = 0.25;
        P_(1,1) = 0.25;
        P_(2,2) = 0.10;
        baseQ_.setZero();
        baseQ_(0,0)= 0.0001;
        baseQ_(1,1)= 0.0001;
        baseQ_(2,2)= 0.00001;


        
        baseR_.setZero();
        baseR_(0,0)= 0.0025;
        baseR_(1,1)= 0.0025;
        baseR_(2,2)= 0.0004;

        qScale_ = 1.0;
        rScale_ = 1.0;


        Q_= baseQ_ * qScale_;
        
        R_= baseR_ * rScale_;
        
        residual_.setZero();

        mahalanobisDistance_ = 0.0;
        measurementAccepted_ = true;
       }

    void initialize(double x, double y, double theta)
    {
        state_ << x,y, theta;

    }

    void predict()
    {
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        
        state_ = F*state_;
        
        P_ = F*P_*F.transpose() + Q_;
    
    }

    void update(double x, double y, double theta)
    {
        Eigen::Vector3d z;
       
        z << x, y,theta;
        
        Eigen::Matrix3d H =Eigen::Matrix3d::Identity();

            residual_  = z - H * state_;

        residual_(2) = normalizeAngle(residual_(2));

       
        Eigen::Matrix3d S= H*P_*H.transpose() + R_;


        Eigen::Vector3d solved = S.ldlt().solve(residual_);

        
        mahalanobisDistance_ = residual_.dot(solved);

        
        const double GATE_THRESHOLD = 11.34;

        if(mahalanobisDistance_ > GATE_THRESHOLD)
          {
            measurementAccepted_ = false;
            return;
          }

        measurementAccepted_ = true;

        
        Eigen::Matrix3d K = P_*H.transpose()* S.inverse();

        state_ = state_ + K * residual_;

        state_(2) = normalizeAngle(state_(2));
       
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

        P_ = (I -K *H)*P_*(I -K * H).transpose() + K* R_* K.transpose();
    }


    void setNoiseScales(
        double qScale,
        double rScale)
    
    {
        qScale_ = std::max(0.001,qScale);
        rScale_ = std::max(0.001,rScale);

        Q_ = baseQ_ * qScale_;
        R_ = baseR_ * rScale_;
    }

    Eigen::Vector3d getState() const
    {
        return state_;
    }

    Eigen::Matrix3d getCovariance() const
    {
        return P_;
    }

    Eigen::Vector3d getResidual() const
    {
        return residual_;
    }

    double getmahalanobisDistance() const
    {
        return mahalanobisDistance_;
    }
    
    bool wasMeasurementAccepted() const
    {
        return measurementAccepted_;
    }
    

};
#endif