#include "pathmaker/master.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pathmaker");

	pm::Master master(true); // for gzb
	master.spin();

	return 0;
}
