/** @file
 *	@brief MAVLink comm protocol testsuite generated from magicc_telem.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef MAGICC_TELEM_TESTSUITE_H
#define MAGICC_TELEM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_magicc_telem(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_magicc_telem(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mav_state_small(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mav_state_small_t packet_in = {
		{ 17.0, 18.0, 19.0 },101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0
    };
	mavlink_mav_state_small_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Va = packet_in.Va;
        	packet1.phi = packet_in.phi;
        	packet1.theta = packet_in.theta;
        	packet1.psi = packet_in.psi;
        	packet1.chi = packet_in.chi;
        	packet1.initial_lat = packet_in.initial_lat;
        	packet1.initial_lon = packet_in.initial_lon;
        	packet1.initial_alt = packet_in.initial_alt;
        
        	mav_array_memcpy(packet1.position, packet_in.position, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_state_small_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mav_state_small_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_state_small_pack(system_id, component_id, &msg , packet1.position , packet1.Va , packet1.phi , packet1.theta , packet1.psi , packet1.chi , packet1.initial_lat , packet1.initial_lon , packet1.initial_alt );
	mavlink_msg_mav_state_small_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_state_small_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.position , packet1.Va , packet1.phi , packet1.theta , packet1.psi , packet1.chi , packet1.initial_lat , packet1.initial_lon , packet1.initial_alt );
	mavlink_msg_mav_state_small_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mav_state_small_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_state_small_send(MAVLINK_COMM_1 , packet1.position , packet1.Va , packet1.phi , packet1.theta , packet1.psi , packet1.chi , packet1.initial_lat , packet1.initial_lon , packet1.initial_alt );
	mavlink_msg_mav_state_small_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mav_current_path(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mav_current_path_t packet_in = {
		17.0,{ 45.0, 46.0, 47.0 },{ 129.0, 130.0, 131.0 },{ 213.0, 214.0, 215.0 },297.0,137,204
    };
	mavlink_mav_current_path_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Va_d = packet_in.Va_d;
        	packet1.rho = packet_in.rho;
        	packet1.path_type = packet_in.path_type;
        	packet1.lambda = packet_in.lambda;
        
        	mav_array_memcpy(packet1.r, packet_in.r, sizeof(float)*3);
        	mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*3);
        	mav_array_memcpy(packet1.c, packet_in.c, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_current_path_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mav_current_path_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_current_path_pack(system_id, component_id, &msg , packet1.path_type , packet1.Va_d , packet1.r , packet1.q , packet1.c , packet1.rho , packet1.lambda );
	mavlink_msg_mav_current_path_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_current_path_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.path_type , packet1.Va_d , packet1.r , packet1.q , packet1.c , packet1.rho , packet1.lambda );
	mavlink_msg_mav_current_path_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mav_current_path_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_current_path_send(MAVLINK_COMM_1 , packet1.path_type , packet1.Va_d , packet1.r , packet1.q , packet1.c , packet1.rho , packet1.lambda );
	mavlink_msg_mav_current_path_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mav_waypoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mav_waypoint_t packet_in = {
		{ 17.0, 18.0, 19.0 },101.0,129.0,65,132,199
    };
	mavlink_mav_waypoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.chi_d = packet_in.chi_d;
        	packet1.Va_d = packet_in.Va_d;
        	packet1.chi_valid = packet_in.chi_valid;
        	packet1.set_current = packet_in.set_current;
        	packet1.clear_wp_list = packet_in.clear_wp_list;
        
        	mav_array_memcpy(packet1.w, packet_in.w, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_waypoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mav_waypoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_waypoint_pack(system_id, component_id, &msg , packet1.w , packet1.chi_d , packet1.chi_valid , packet1.Va_d , packet1.set_current , packet1.clear_wp_list );
	mavlink_msg_mav_waypoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_waypoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.w , packet1.chi_d , packet1.chi_valid , packet1.Va_d , packet1.set_current , packet1.clear_wp_list );
	mavlink_msg_mav_waypoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mav_waypoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_waypoint_send(MAVLINK_COMM_1 , packet1.w , packet1.chi_d , packet1.chi_valid , packet1.Va_d , packet1.set_current , packet1.clear_wp_list );
	mavlink_msg_mav_waypoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_raw_t packet_in = {
		{ 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242 }
    };
	mavlink_rc_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.values, packet_in.values, sizeof(uint16_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_raw_pack(system_id, component_id, &msg , packet1.values );
	mavlink_msg_rc_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.values );
	mavlink_msg_rc_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rc_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_raw_send(MAVLINK_COMM_1 , packet1.values );
	mavlink_msg_rc_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_data_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,18483,211
    };
	mavlink_gps_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.speed = packet_in.speed;
        	packet1.ground_course = packet_in.ground_course;
        	packet1.covariance = packet_in.covariance;
        	packet1.NumSat = packet_in.NumSat;
        	packet1.fix = packet_in.fix;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_data_pack(system_id, component_id, &msg , packet1.fix , packet1.NumSat , packet1.latitude , packet1.longitude , packet1.altitude , packet1.speed , packet1.ground_course , packet1.covariance );
	mavlink_msg_gps_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.fix , packet1.NumSat , packet1.latitude , packet1.longitude , packet1.altitude , packet1.speed , packet1.ground_course , packet1.covariance );
	mavlink_msg_gps_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_data_send(MAVLINK_COMM_1 , packet1.fix , packet1.NumSat , packet1.latitude , packet1.longitude , packet1.altitude , packet1.speed , packet1.ground_course , packet1.covariance );
	mavlink_msg_gps_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mav_controller_internals(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mav_controller_internals_t packet_in = {
		17.0,45.0,{ 73.0, 74.0, 75.0, 76.0 },77,144
    };
	mavlink_mav_controller_internals_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.theta_c = packet_in.theta_c;
        	packet1.phi_c = packet_in.phi_c;
        	packet1.alt_zone = packet_in.alt_zone;
        	packet1.aux_valid = packet_in.aux_valid;
        
        	mav_array_memcpy(packet1.aux, packet_in.aux, sizeof(float)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_internals_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mav_controller_internals_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_internals_pack(system_id, component_id, &msg , packet1.theta_c , packet1.phi_c , packet1.alt_zone , packet1.aux , packet1.aux_valid );
	mavlink_msg_mav_controller_internals_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_internals_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.theta_c , packet1.phi_c , packet1.alt_zone , packet1.aux , packet1.aux_valid );
	mavlink_msg_mav_controller_internals_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mav_controller_internals_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_internals_send(MAVLINK_COMM_1 , packet1.theta_c , packet1.phi_c , packet1.alt_zone , packet1.aux , packet1.aux_valid );
	mavlink_msg_mav_controller_internals_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mav_controller_commands(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mav_controller_commands_t packet_in = {
		17.0,45.0,73.0,101.0,{ 129.0, 130.0, 131.0, 132.0 },101
    };
	mavlink_mav_controller_commands_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Va_c = packet_in.Va_c;
        	packet1.h_c = packet_in.h_c;
        	packet1.chi_c = packet_in.chi_c;
        	packet1.phi_ff = packet_in.phi_ff;
        	packet1.aux_valid = packet_in.aux_valid;
        
        	mav_array_memcpy(packet1.aux, packet_in.aux, sizeof(float)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_commands_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mav_controller_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_commands_pack(system_id, component_id, &msg , packet1.Va_c , packet1.h_c , packet1.chi_c , packet1.phi_ff , packet1.aux , packet1.aux_valid );
	mavlink_msg_mav_controller_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_commands_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Va_c , packet1.h_c , packet1.chi_c , packet1.phi_ff , packet1.aux , packet1.aux_valid );
	mavlink_msg_mav_controller_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mav_controller_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mav_controller_commands_send(MAVLINK_COMM_1 , packet1.Va_c , packet1.h_c , packet1.chi_c , packet1.phi_ff , packet1.aux , packet1.aux_valid );
	mavlink_msg_mav_controller_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_magicc_telem(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_mav_state_small(system_id, component_id, last_msg);
	mavlink_test_mav_current_path(system_id, component_id, last_msg);
	mavlink_test_mav_waypoint(system_id, component_id, last_msg);
	mavlink_test_rc_raw(system_id, component_id, last_msg);
	mavlink_test_gps_data(system_id, component_id, last_msg);
	mavlink_test_mav_controller_internals(system_id, component_id, last_msg);
	mavlink_test_mav_controller_commands(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAGICC_TELEM_TESTSUITE_H
