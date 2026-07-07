#ifndef RADAR_CONTI_SRR308_STRUCTS_HPP
#define RADAR_CONTI_SRR308_STRUCTS_HPP

#include <cstdint>

namespace radar_conti_srr308_structs
{
  struct MotionInputSignal
  {
    double speed;
    double yaw_rate;
    uint8_t direction;
  };
}

#endif // RADAR_CONTI_SRR308_STRUCTS_HPP