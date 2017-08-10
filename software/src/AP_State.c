#include "AP_State.h"
#include "paramter.h"
#include "notify.h"


void set_auto_armed(uint8_t b)
{
    // if no change, exit immediately
	if( ap.flags.auto_armed == b )
			return;

	ap.flags.auto_armed = b;
  
}

void set_land_complete(uint8_t b)
{
    // if no change, exit immediately
    if( ap.flags.land_complete == b )		//没有改变
        return;

	if (b == 1)
	{
		debug("land");
	}
	else
	{
		debug("non land");
	}
    ap.flags.land_complete = b;
}

// set land complete maybe flag
void set_land_complete_maybe(uint8_t b)
{
    // if no change, exit immediately
    if (ap.flags.land_complete_maybe == b)
        return;
		
    ap.flags.land_complete_maybe = b;
}

void set_pre_arm_check(uint8_t b)
{
    if(ap.flags.pre_arm_check != b) {
        ap.flags.pre_arm_check = b;   
				notify.flags.pre_arm_check = ap.flags.pre_arm_check;
				
    }
}

void set_pre_arm_rc_check(uint8_t b)
{
    if(ap.flags.pre_arm_rc_check != b) {
        ap.flags.pre_arm_rc_check  = b;
    }
}

void set_simple_mode(uint8_t b)
{
	if (ap.flags.simple_mode != b)
	{	
		if (b == 0)
		{
			
		}
		else if (b == 1)
		{
			
		}
		else//无头模式？，记录起飞时的方向
		{
			// initialise super simple heading
			//update_super_simple_bearing(1);		//强制无头模式,
		}
		ap.flags.simple_mode = b;
	}
}

