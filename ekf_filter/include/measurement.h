#pragma once

#include "pose.h"

struct Measurement
{
    Pose pose;
    double Timestamp;
    bool valid;
};