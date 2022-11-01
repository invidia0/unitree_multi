#include "RRTStar/RRTStar.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "RRTStar");
	RRTStar ce; // Create an instance of the class
	ros::Rate r(100);

	while (ros::ok())
	{
		ce.spinner();
		r.sleep();
	}

	return 0;
}
