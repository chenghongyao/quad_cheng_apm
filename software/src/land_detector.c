#include "land_detector.h"
#include "paramter.h"
#include "AP_InertialSensor.h"



uint8_t land_complete_maybe(void)
{
    return (ap.flags.land_complete || ap.flags.land_complete_maybe);
}



void update_land_detector(void)
{
	//bool climb_rate_low = (abs(climb_rate) < LAND_DETECTOR_CLIMBRATE_MAX) && (abs(baro_climbrate) < LAND_DETECTOR_BARO_CLIMBRATE_MAX);


}
