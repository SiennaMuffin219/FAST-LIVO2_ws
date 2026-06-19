#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <termios.h>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Time.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

using namespace std;

ros::Publisher imu_pub;
ros::Publisher lidar_pub;
ros::Publisher camera_pub;

inline void SignalHandler(int signum)
{
	printf("wit imu will exit\r\n");
	ros::shutdown();
	exit(signum);
}

int open_serial_port(const char* port, int baud)
{
	int serial_port = open(port, O_RDONLY);
	
	if (serial_port < 0)
	{
		ROS_ERROR("Serial port opening failure");
		return -1;
	}
	else
	{
		ROS_INFO("Serial port opened successfully");
	}

	struct termios tty;

	if(tcgetattr(serial_port, &tty) != 0)
	{
    	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		close(serial_port);
		return -1;
	}

	cfsetispeed(&tty, B230400);
	tty.c_cflag &= ~HUPCL;
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_oflag &= ~OPOST;
	tty.c_cflag &= ~(CSIZE | PARENB);
	tty.c_cflag |= CS8;
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);


	if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
	{
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		close(serial_port);
		return -1;
	}

	return serial_port;
}

void handle_payload(std::string payload)
{
	int values[12];
	size_t pos0 = 0;
	for (int i = 0; i < 12; i++)
	{
		size_t pos1 = payload.find(';', pos0);
		if (pos1 == std::string::npos && i != 11 || i == 11 && pos1 != std::string::npos)
			return; // Wrong number of values

		if (i == 11)
			pos1 = payload.length();
		
		values[i] = stoi(payload.substr(pos0, pos1 - pos0));

		pos0 = pos1 + 1;
	}

	ros::Time time = ros::Time((values[0] * 60 + values[1]) * 60 + values[2], values[3] * 1000000);
	
	if (values[10]) // LiDAR
	{
		std_msgs::Time msg_time;
		msg_time.data = time;
		lidar_pub.publish(msg_time);
	}
    if (values[11]) // Camera
    {
		std_msgs::Time msg_time;
		msg_time.data = time;
	    camera_pub.publish(msg_time);
	}

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = time;
	imu_msg.header.frame_id = "base_link";

	imu_msg.linear_acceleration.x = values[4] / 32768.0 * 16;
	imu_msg.linear_acceleration.y = values[5] / 32768.0 * 16;
	imu_msg.linear_acceleration.z = values[6] / 32768.0 * 16;

	imu_msg.angular_velocity.x = values[7] / 32768.0 * 2000 * M_PI / 180;
	imu_msg.angular_velocity.y = values[8] / 32768.0 * 2000 * M_PI / 180;
	imu_msg.angular_velocity.z = values[9] / 32768.0 * 2000 * M_PI / 180;

	imu_pub.publish(imu_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wit_imu_publisher");
	ros::NodeHandle imu_node;
	signal(SIGINT, SignalHandler);
	
	std::string port = "/dev/esp_usb";
	int baud = 230400;
	int serial_port = open_serial_port(port.c_str(), baud);
	if (serial_port < 0)
	{
		ROS_ERROR("Exiting");
		return 0;
	}

	imu_pub = imu_node.advertise<sensor_msgs::Imu>("wit/imu", 100);
	lidar_pub = imu_node.advertise<std_msgs::Time>("wit/lidar", 1);
	camera_pub = imu_node.advertise<std_msgs::Time>("wit/camera", 1);

	std::string big_buffer;
	char buffer[256];

	while (ros::ok())
	{
		memset(buffer, 0, sizeof(buffer));
		int n = read(serial_port, &buffer, sizeof(buffer));

		if (n < 0)
		{
			cout << "ERROR" << endl;
			ROS_ERROR("Read error (%d), exiting", n);
			close(serial_port);
			return 0;
		}

		big_buffer.append(buffer, n);

		while (true)
		{
			size_t start = big_buffer.find('<'); // Start of text
			size_t end = big_buffer.find('>', start + 1); // End of text

			if (start == std::string::npos)
			{
				big_buffer.clear();
				break;
			}
			if (end == std::string::npos)
			{
				break; // Incomplete message
			}

			std::string payload = big_buffer.substr(start+1, end-start-1);

			handle_payload(payload);
			
			big_buffer.erase(0, end+1);
		}
	}

	cout << "End (ros::ok = " << ros::ok() << ")" << endl;

	return 0;
}
