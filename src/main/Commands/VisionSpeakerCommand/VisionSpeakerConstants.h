// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <algorithm>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include <units/length.h>
#include <units/time.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>

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

  static const InterpolatingTable<units::meter_t, double> DistanceToShooter{
    {
        {0.1_m, 100},
        {0.5_m, 100},
        {1.0_m, 100},
        {1.5_m, 100},
        {2.0_m, 110},
        {2.5_m, 120},
        {3.0_m, 130},
        {3.5_m, 140},
        {4.0_m, 150},
        {4.5_m, 160},
    }
    
  };

};
