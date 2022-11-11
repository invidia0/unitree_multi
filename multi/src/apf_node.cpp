#include "APF/APF.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "APF");
	APF ce; // Create an instance of the class
	ros::Rate r(100);

	while (ros::ok())
	{
		ce.spinner();
		r.sleep();
	}

	return 0;
}
