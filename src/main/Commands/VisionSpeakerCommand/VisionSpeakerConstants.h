// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <algorithm>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include <units/length.h>
#include <units/time.h>
#include <units/angle.h>

namespace VisionSpeakerConstants {

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToLowerAngle{
      {
        {0.1_m, -30_deg},
        {0.5_m, -25_deg},
        {1.0_m, -20_deg},
        {1.5_m, -15_deg},
        {2.0_m, -10_deg},
        {2.5_m, -5_deg},
        {3.0_m, 0_deg},
        {3.5_m, 5_deg},
        {4.0_m, 10_deg},
        {4.5_m, 15_deg},

      }

  }; 

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToUpperAngle{
    {   
        {0.1_m, 90_deg},
        {0.5_m, 85_deg},
        {1.0_m, 80_deg},
        {1.5_m, 75_deg},
        {2.0_m, 70_deg},
        {2.5_m, 65_deg},
        {3.0_m, 60_deg},
        {3.5_m, 55_deg},
        {4.0_m, 50_deg},
        {4.5_m, 45_deg},
      
      }

  };

};
