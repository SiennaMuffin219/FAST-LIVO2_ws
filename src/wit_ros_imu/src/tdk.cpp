#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <termios.h>
#include <math.h>
#include <ros/ros.h>
#include "imu_packet.h"
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

void handle_payload(imu_packet* packet)
{
	ros::Time time = ros::Time(packet->packet_id / 200, (packet->packet_id % 200) * 5 * 1000000);
	//printf("%d\r\n", packet->packet_id);
	if ((packet->packet_id % 200) == 0) // LiDAR
	{
		std_msgs::Time msg_time;
		msg_time.data = time;
		lidar_pub.publish(msg_time);
	}
    if ((packet->packet_id % 20) == 0) // Camera
    {
		std_msgs::Time msg_time;
		msg_time.data = time;
	    camera_pub.publish(msg_time);
	}

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = time;
	imu_msg.header.frame_id = "base_link";

	imu_msg.linear_acceleration.x = packet->acc.x / 32768.0 * 8 * 9.80665;
	imu_msg.linear_acceleration.y = packet->acc.y / 32768.0 * 8 * 9.80665;
	imu_msg.linear_acceleration.z = packet->acc.z / 32768.0 * 8 * 9.80665;

	imu_msg.angular_velocity.x = packet->ang.x / 32768.0 * 500 * M_PI / 180;
	imu_msg.angular_velocity.y = packet->ang.y / 32768.0 * 500 * M_PI / 180;
	imu_msg.angular_velocity.z = packet->ang.z / 32768.0 * 500 * M_PI / 180;

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

	uint8_t buffer[128];
	std::vector<uint8_t> big_buffer;

	while (ros::ok())
	{
		memset(buffer, 0, sizeof(buffer));
		int n = read(serial_port, buffer, sizeof(buffer));

		if (n < 0)
		{
			cout << "ERROR" << endl;
			ROS_ERROR("Read error (%d), exiting", n);
			close(serial_port);
			return 0;
		}
		
		big_buffer.insert(big_buffer.end(), buffer, buffer + n);
		
		while (big_buffer.size() >= 20)
		{
			if (big_buffer[0] == 0x9E && big_buffer[1] == 0xE9)
			{
				// for (int i = 0; i < 20; i++)
				// {
				// 	printf("0x%02X ", big_buffer[i]);
				// }
				// printf("\r\n");
			
				imu_packet *packet = reinterpret_cast<imu_packet*>(&big_buffer[0]);	
				handle_payload(packet);
				big_buffer.erase(big_buffer.begin(), big_buffer.begin() + 20);
			}
			else
				big_buffer.erase(big_buffer.begin());
		}

	}

	cout << "End (ros::ok = " << ros::ok() << ")" << endl;

	return 0;
}
