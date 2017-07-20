/** @file
 *	@brief MAVLink comm protocol testsuite generated from telemetry.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef telemetry_TESTSUITE_H
#define telemetry_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_telemetry(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_telemetry(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_offboard_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_offboard_control_t packet_in = {
		17.0,45.0,73.0,101.0,53,120
    };
	mavlink_offboard_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.F = packet_in.F;
        	packet1.mode = packet_in.mode;
        	packet1.ignore = packet_in.ignore;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_offboard_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_pack(system_id, component_id, &msg , packet1.mode , packet1.ignore , packet1.x , packet1.y , packet1.z , packet1.F );
	mavlink_msg_offboard_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.ignore , packet1.x , packet1.y , packet1.z , packet1.F );
	mavlink_msg_offboard_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_offboard_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_send(MAVLINK_COMM_1 , packet1.mode , packet1.ignore , packet1.x , packet1.y , packet1.z , packet1.F );
	mavlink_msg_offboard_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_imu_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
	mavlink_small_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_us = packet_in.time_boot_us;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_pack(system_id, component_id, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature );
	mavlink_msg_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature );
	mavlink_msg_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_send(MAVLINK_COMM_1 , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature );
	mavlink_msg_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_mag(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_mag_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_small_mag_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xmag = packet_in.xmag;
        	packet1.ymag = packet_in.ymag;
        	packet1.zmag = packet_in.zmag;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_mag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_pack(system_id, component_id, &msg , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_small_mag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_small_mag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_mag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_send(MAVLINK_COMM_1 , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_small_mag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_baro(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_baro_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_small_baro_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.altitude = packet_in.altitude;
        	packet1.pressure = packet_in.pressure;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_baro_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_pack(system_id, component_id, &msg , packet1.altitude , packet1.pressure , packet1.temperature );
	mavlink_msg_small_baro_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.altitude , packet1.pressure , packet1.temperature );
	mavlink_msg_small_baro_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_baro_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_send(MAVLINK_COMM_1 , packet1.altitude , packet1.pressure , packet1.temperature );
	mavlink_msg_small_baro_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_diff_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_diff_pressure_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_diff_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.velocity = packet_in.velocity;
        	packet1.diff_pressure = packet_in.diff_pressure;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_diff_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_pack(system_id, component_id, &msg , packet1.velocity , packet1.diff_pressure , packet1.temperature );
	mavlink_msg_diff_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.velocity , packet1.diff_pressure , packet1.temperature );
	mavlink_msg_diff_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_diff_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_send(MAVLINK_COMM_1 , packet1.velocity , packet1.diff_pressure , packet1.temperature );
	mavlink_msg_diff_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_camera_stamped_small_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_camera_stamped_small_imu_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113
    };
	mavlink_camera_stamped_small_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_us = packet_in.time_boot_us;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.temperature = packet_in.temperature;
        	packet1.image = packet_in.image;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_camera_stamped_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_pack(system_id, component_id, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature , packet1.image );
	mavlink_msg_camera_stamped_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature , packet1.image );
	mavlink_msg_camera_stamped_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_camera_stamped_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_send(MAVLINK_COMM_1 , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature , packet1.image );
	mavlink_msg_camera_stamped_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_named_command_struct(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_named_command_struct_t packet_in = {
		17.0,45.0,73.0,101.0,"QRSTUVWXY",211,22
    };
	mavlink_named_command_struct_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.F = packet_in.F;
        	packet1.type = packet_in.type;
        	packet1.ignore = packet_in.ignore;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_named_command_struct_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_pack(system_id, component_id, &msg , packet1.name , packet1.type , packet1.ignore , packet1.x , packet1.y , packet1.z , packet1.F );
	mavlink_msg_named_command_struct_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.type , packet1.ignore , packet1.x , packet1.y , packet1.z , packet1.F );
	mavlink_msg_named_command_struct_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_named_command_struct_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_send(MAVLINK_COMM_1 , packet1.name , packet1.type , packet1.ignore , packet1.x , packet1.y , packet1.z , packet1.F );
	mavlink_msg_named_command_struct_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_sonar(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_sonar_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_small_sonar_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.range = packet_in.range;
        	packet1.max_range = packet_in.max_range;
        	packet1.min_range = packet_in.min_range;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_sonar_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_sonar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_sonar_pack(system_id, component_id, &msg , packet1.range , packet1.max_range , packet1.min_range );
	mavlink_msg_small_sonar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_sonar_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.range , packet1.max_range , packet1.min_range );
	mavlink_msg_small_sonar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_sonar_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_sonar_send(MAVLINK_COMM_1 , packet1.range , packet1.max_range , packet1.min_range );
	mavlink_msg_small_sonar_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_telemetry_cmd_t packet_in = {
		5
    };
	mavlink_telemetry_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_telemetry_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_pack(system_id, component_id, &msg , packet1.command );
	mavlink_msg_telemetry_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command );
	mavlink_msg_telemetry_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_telemetry_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_send(MAVLINK_COMM_1 , packet1.command );
	mavlink_msg_telemetry_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry_cmd_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_telemetry_cmd_ack_t packet_in = {
		5,72
    };
	mavlink_telemetry_cmd_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        	packet1.success = packet_in.success;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_telemetry_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_ack_pack(system_id, component_id, &msg , packet1.command , packet1.success );
	mavlink_msg_telemetry_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command , packet1.success );
	mavlink_msg_telemetry_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_telemetry_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_cmd_ack_send(MAVLINK_COMM_1 , packet1.command , packet1.success );
	mavlink_msg_telemetry_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry_output_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_telemetry_output_raw_t packet_in = {
		93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }
    };
	mavlink_telemetry_output_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.stamp = packet_in.stamp;
        
        	mav_array_memcpy(packet1.values, packet_in.values, sizeof(float)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_output_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_telemetry_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_output_raw_pack(system_id, component_id, &msg , packet1.stamp , packet1.values );
	mavlink_msg_telemetry_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_output_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stamp , packet1.values );
	mavlink_msg_telemetry_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_telemetry_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_output_raw_send(MAVLINK_COMM_1 , packet1.stamp , packet1.values );
	mavlink_msg_telemetry_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_telemetry_status_t packet_in = {
		17235,17339,17,84,151
    };
	mavlink_telemetry_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.num_errors = packet_in.num_errors;
        	packet1.loop_time_us = packet_in.loop_time_us;
        	packet1.status = packet_in.status;
        	packet1.error_code = packet_in.error_code;
        	packet1.control_mode = packet_in.control_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_telemetry_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_status_pack(system_id, component_id, &msg , packet1.status , packet1.error_code , packet1.control_mode , packet1.num_errors , packet1.loop_time_us );
	mavlink_msg_telemetry_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status , packet1.error_code , packet1.control_mode , packet1.num_errors , packet1.loop_time_us );
	mavlink_msg_telemetry_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_telemetry_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_status_send(MAVLINK_COMM_1 , packet1.status , packet1.error_code , packet1.control_mode , packet1.num_errors , packet1.loop_time_us );
	mavlink_msg_telemetry_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry_version(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_telemetry_version_t packet_in = {
		"ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
	mavlink_telemetry_version_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.version, packet_in.version, sizeof(char)*50);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_version_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_telemetry_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_version_pack(system_id, component_id, &msg , packet1.version );
	mavlink_msg_telemetry_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_version_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.version );
	mavlink_msg_telemetry_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_telemetry_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_version_send(MAVLINK_COMM_1 , packet1.version );
	mavlink_msg_telemetry_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_offboard_control(system_id, component_id, last_msg);
	mavlink_test_small_imu(system_id, component_id, last_msg);
	mavlink_test_small_mag(system_id, component_id, last_msg);
	mavlink_test_small_baro(system_id, component_id, last_msg);
	mavlink_test_diff_pressure(system_id, component_id, last_msg);
	mavlink_test_camera_stamped_small_imu(system_id, component_id, last_msg);
	mavlink_test_named_command_struct(system_id, component_id, last_msg);
	mavlink_test_small_sonar(system_id, component_id, last_msg);
	mavlink_test_telemetry_cmd(system_id, component_id, last_msg);
	mavlink_test_telemetry_cmd_ack(system_id, component_id, last_msg);
	mavlink_test_telemetry_output_raw(system_id, component_id, last_msg);
	mavlink_test_telemetry_status(system_id, component_id, last_msg);
	mavlink_test_telemetry_version(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // telemetry_TESTSUITE_H
