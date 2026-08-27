#include <iostream>
#include <thread>
#include <chrono>
#include "noise_generator.h"
#include "pose.h"
#include "measurement.h"
#include <mqtt/async_client.h>



const std::string SERVER_ADDRESS = "tcp://mqtt:1883";
const std::string CLIENT_ID = "simulator";
const std::string MEASUREMENT_TOPIC = "ekf/measurement";
const std::string GROUND_TRUTH_TOPIC = "ekf/ground_truth";
int main()
{  mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

    try
    {
        client.connect()->wait();

        std::cout << "Connected to MQTT broker" << std::endl;

    }

    catch (const mqtt::exception& e)
    {
        std::cerr << "MQTT error: "
                  << e.what()
                  << std::endl;

        return 1;
    }



    
    NoiseGenerator noise;
    Pose groundTruth;

    groundTruth.x = 1.0;
    groundTruth.y = 0.5;
    groundTruth.theta = 0.0;

    while(true)
    {   
        Measurement measurement;


        std::string groundTruthPayload =
          std::to_string(groundTruth.x) + ","+
          std::to_string(groundTruth.y) +"," +
          std::to_string(groundTruth.theta);


        auto groundTruthMessage = 
           mqtt::make_message(
            GROUND_TRUTH_TOPIC, groundTruthPayload
           );

        groundTruthMessage->set_qos(1);

        client.publish(groundTruthMessage)->wait();

        measurement.valid=!noise.isDropout();

        if(!measurement.valid)
        {
            std::cout <<"SENSOR DROPOUT" << std::endl;
        }
        else
        {
            
        
            measurement.pose.x = groundTruth.x +noise.x();

            measurement.pose.y = groundTruth.y + noise.y();

            measurement.pose.theta = groundTruth.theta + noise.theta();

            bool outlier = noise.isOutlier();
        

        if(outlier)
        {
            measurement.pose.x += noise.outlier();
            measurement.pose.y += noise.outlier();
            measurement.pose.theta += noise.outlier();

            std::cout<< "OUTLIER" << std::endl;
        }
    std::string payload = 
        std::to_string(measurement.pose.x)+","+
        std::to_string(measurement.pose.y)+","+
        std::to_string(measurement.pose.theta);

    auto message = mqtt::make_message(MEASUREMENT_TOPIC, payload);

    message->set_qos(1);

    client.publish(message)->wait();

        std::cout
            <<"Measurement:"
            <<"x="<< measurement.pose.x
            <<"y="<< measurement.pose.y
            <<"theta="<< measurement.pose.theta
            <<std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    }
    return 0;
}
    