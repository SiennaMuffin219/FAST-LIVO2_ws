#include <stdint.h>
#ifndef IMU_PACKET_H
#define IMU_PACKET_H

struct imu_acceleration
{
	int16_t x;
	int16_t y;
	int16_t z;
};

struct imu_angular_velocity
{
	int16_t x;
	int16_t y;
	int16_t z;
};

#pragma pack(push,1)
struct imu_packet
{
	uint16_t magic_packet = 0;
	uint16_t temp;
	uint32_t packet_id;
	imu_acceleration acc;
	imu_angular_velocity ang;
};
#pragma pack(pop)

#endif // IMU_PACKET_H
