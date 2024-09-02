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
        {1.66_m, -12.0_deg},
        {1.9_m, -12.0_deg},
        {2.4_m, -12.0_deg},
        {2.9_m, -12.0_deg},
        {3.4_m, -12.0_deg},
        {3.9_m, -12.0_deg},
        {4.4_m, -13.0_deg},
        {4.9_m, -13.0_deg},
        {5.4_m, -13.0_deg},
        {5.9_m, -13.0_deg},
        {6.4_m, -13.0_deg},
        {6.9_m, -13.5_deg}

      }

  }; 

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToUpperAngle{
    {   
        {1.66_m, 63.0_deg},
        {1.9_m, 67.0_deg},
        {2.4_m, 72.0_deg},
        {2.9_m, 75.0_deg},
        {3.4_m, 79.0_deg},
        {3.9_m, 82.0_deg},
        {4.4_m, 85.3_deg},
        {4.9_m, 86.2_deg},
        {5.4_m, 87.5_deg},
        {5.9_m, 88.0_deg},
        {6.4_m, 88.2_deg},
        {6.9_m, 88.7_deg},
      
      }

  };

};
