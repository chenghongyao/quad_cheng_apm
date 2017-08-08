#include "inertia.h"
#include "InertialNav.h"



extern float G_Dt;

void read_inertia()
{
	InertialNav_update(G_Dt);
	
}

