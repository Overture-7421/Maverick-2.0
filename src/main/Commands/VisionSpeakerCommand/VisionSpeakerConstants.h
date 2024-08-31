// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <algorithm>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include <units/length.h>
#include <units/time.h>
#include <units/angle.h>

class VisionSpeakerConstants {
 public:

  InterpolatingTable<units::meter_t, units::degree_t> DistanceToLowerAngle{
      {
        {0.1_m, 0_deg},
        {0.5_m, 0_deg},
        {1.0_m, 0_deg},
        {1.5_m, 0_deg},
        {2.0_m, 0_deg},
        {2.5_m, 0_deg},
        {3.0_m, 0_deg},
        {3.5_m, 0_deg},
        {4.0_m, 0_deg},
        {4.5_m, 0_deg},

      }

  }; 

  InterpolatingTable<units::meter_t, units::degree_t> DistanceToUpperAngle{
    {   
        {0.1_m, 90_deg},
        {0.5_m, 90_deg},
        {1.0_m, 90_deg},
        {1.5_m, 90_deg},
        {2.0_m, 90_deg},
        {2.5_m, 90_deg},
        {3.0_m, 90_deg},
        {3.5_m, 90_deg},
        {4.0_m, 90_deg},
        {4.5_m, 90_deg},
      
      }

  };

};
