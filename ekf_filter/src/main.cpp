#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include<mqtt/async_client.h>
#include "measurement.h"
#include "ekf.h"

const std::string SERVER_ADDRESS = "tcp://mqtt:1883";
const std::string CLIENT_ID = "ekf";
const std::string INPUT_TOPIC = "ekf/measurement";
const std::string CONFIG_TOPIC = "ekf/config";
const std::string OUTPUT_TOPIC = "ekf/filtered";
const std::string COVARIANCE_TOPIC = "ekf/covariance";
const std::string RESIDUAL_TOPIC = "ekf/residual";
const std::string  STATUS_TOPIC = "ekf/status";
class Callback : public  mqtt::callback
{   
    private:
        
        EKF ekf_;

        bool initialized_ = false;

        mqtt::async_client* client_;

    public :


         Callback(mqtt::async_client* client)
            : client_(client)
            {

            }



         void message_arrived(mqtt::const_message_ptr msg) override
         {   
             const std::string topic = msg->get_topic();

             const std::string payload = msg->get_payload();

             if(topic == CONFIG_TOPIC)
             {
                std::stringstream ss(payload);

                double qScale;
                double rScale;

                char comma;

                ss>> qScale
                  >> comma
                  >> rScale;
             

             
             if ((ss.fail()))
             {
                std::cerr
                 << "Invalid measurement:"
                 << payload
                 << std::endl;

             return;
             }
             ekf_.setNoiseScales(qScale,rScale);

             std::cout 
                << "EKF config updated:"
                << "Q=" << qScale
                << "R=" << rScale
                << std::endl;

             return;
            }

            if(topic != INPUT_TOPIC)
            {
                return;
            }


            std::stringstream ss(payload);
            double x;
            double y;
            double theta;

            char comma1;
            char comma2;

            ss>>x
              >>comma1
              >>y
              >>comma2
              >>theta;

            if(ss.fail())
            {
                std::cerr
                   << "Invalid measurement"
                   << payload
                   << std::endl;

                return;
            }

             Measurement measurement;

             measurement.pose.x = x;
             measurement.pose.y = y;
             measurement.pose.theta = theta;

                 std::cout 
                 << "EKF measurement:"
                 << "x="<< measurement.pose.x
                 <<"y="<<measurement.pose.y
                 <<"theta="<<measurement.pose.theta
                 << std::endl;



            if(!initialized_)
            {
                ekf_.initialize(x,y,theta);
                initialized_ = true;
            }

            ekf_.predict();
            ekf_.update(x,y,theta);

            Eigen::Vector3d state =ekf_.getState();
            
            Eigen::Vector3d residual = ekf_.getResidual();
           
            

            Eigen::Matrix3d covariance = ekf_.getCovariance();
            
            double mahalanobis = ekf_.getmahalanobisDistance();
            
            bool accepted = ekf_.wasMeasurementAccepted();
            
            std::string statusPayload;

            if(accepted)
            {
                statusPayload = "ACCEPTED";

            }
            else
            {
                statusPayload = "REJECTED";
            }

            auto statusMessage = mqtt::make_message(STATUS_TOPIC, statusPayload);
            statusMessage->set_qos(1);

            client_->publish(statusMessage);

            std::cout

                << "Mahalanobis distance:"
                << mahalanobis
                << std::endl;

        
              
            if(accepted)
            {
                std::cout
                   << "Measurement ACCEPTED"
                   << std::endl;
            }
            else
            {
                std::cout 
                   <<"Measurement REJECTED"
                   <<std::endl;
            }

          

            std::cout<<"EKF State:"
                     <<"x="<<state(0)
                     <<"y="<<state(1)
                     <<"theta"<<state(2)
                     <<std::endl;

            std::string filteredPayload = 
                std::to_string(state(0)) +","+
                std::to_string(state(1)) +","+
                std::to_string(state(2));

            auto filteredMessage = 
               mqtt::make_message(OUTPUT_TOPIC, filteredPayload);
               filteredMessage->set_qos(1);

             std::string residualpayload = 
                std::to_string(residual(0)) + "," +
                std::to_string(residual(1))+ "," +
                std::to_string(residual(2));
            auto residualMessage = 
               mqtt::make_message(RESIDUAL_TOPIC, residualpayload);
               residualMessage->set_qos(1);

                std::string covariancePayload = 
                 std::to_string(covariance(0,0)) + "," +
                 std::to_string(covariance(0,1)) + "," +
                 std::to_string(covariance(0,2)) + "," +
                 std::to_string(covariance(1,0)) + "," +
                 std::to_string(covariance(1,1)) + "," +
                 std::to_string(covariance(1,2)) + "," +
                 std::to_string(covariance(2,0)) + "," +
                 std::to_string(covariance(2,1)) + "," +
                 std::to_string(covariance(2,2));
                 
                 auto covarianceMessage = 
                    mqtt::make_message(COVARIANCE_TOPIC, covariancePayload);   
                    covarianceMessage->set_qos(1);
                 
            try
            {   
                client_->publish(filteredMessage);
                client_->publish(residualMessage);
                client_->publish(covarianceMessage);
                client_->publish(statusMessage);
                     std::cout
                << "Published:"
                << std::endl
                << "  ekf/filtered: "
                << filteredPayload
                << std::endl
                << "  ekf/residual: "
                << residualpayload
                << std::endl
                << "  ekf/covariance: "
                << covariancePayload
                << std::endl;

            
                       
            }
            catch(const mqtt::exception& e)
            {
               std::cerr
                 <<"Failed to publish filtered state:"
                 <<e.what()
                 << std::endl;
            }
        }
    };
int main()
{
    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
   
    try
    {
        client.connect()->wait();

        std::cout<< "EKF connected to MQTT broker"
                 << std::endl;

        Callback callback(&client);
        client.set_callback(callback);

        

        client.subscribe(INPUT_TOPIC,1)->wait();

        std::cout<<"Subscribed to:"
                 <<INPUT_TOPIC
                 <<std::endl;

        client.subscribe(CONFIG_TOPIC,1)->wait();
        std::cout 
            << "Subscribed to:"
            << CONFIG_TOPIC
            << std::endl;

        while(true)
        {
            std::this_thread::sleep_for(
                std::chrono::seconds(1)
            );
        }
    }

        
    
    catch(const mqtt::exception& e)
    {
        std::cerr <<"MQTT error"
                  <<e.what()
                  <<std::endl;
        return 1;
    }
    return 0;
}