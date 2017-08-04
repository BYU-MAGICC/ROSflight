// MESSAGE MAV_CONTROLLER_COMMANDS PACKING

#define MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS 206

typedef struct __mavlink_mav_controller_commands_t
{
 float Va_c; /*< Commanded airspeed (m/s)*/
 float h_c; /*< Commanded altitude (m)*/
 float chi_c; /*< Commanded course (rad)*/
 float phi_ff; /*< Feed forward command for orbits (rad)*/
 float aux[4]; /*< Optional auxiliary commands*/
 uint8_t aux_valid; /*< Auxiliary commands valid (1 = true, 0 = false)*/
} mavlink_mav_controller_commands_t;

#define MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN 33
#define MAVLINK_MSG_ID_206_LEN 33

#define MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC 93
#define MAVLINK_MSG_ID_206_CRC 93

#define MAVLINK_MSG_MAV_CONTROLLER_COMMANDS_FIELD_AUX_LEN 4

#define MAVLINK_MESSAGE_INFO_MAV_CONTROLLER_COMMANDS { \
	"MAV_CONTROLLER_COMMANDS", \
	6, \
	{  { "Va_c", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mav_controller_commands_t, Va_c) }, \
         { "h_c", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mav_controller_commands_t, h_c) }, \
         { "chi_c", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mav_controller_commands_t, chi_c) }, \
         { "phi_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mav_controller_commands_t, phi_ff) }, \
         { "aux", NULL, MAVLINK_TYPE_FLOAT, 4, 16, offsetof(mavlink_mav_controller_commands_t, aux) }, \
         { "aux_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_mav_controller_commands_t, aux_valid) }, \
         } \
}


/**
 * @brief Pack a mav_controller_commands message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Va_c Commanded airspeed (m/s)
 * @param h_c Commanded altitude (m)
 * @param chi_c Commanded course (rad)
 * @param phi_ff Feed forward command for orbits (rad)
 * @param aux Optional auxiliary commands
 * @param aux_valid Auxiliary commands valid (1 = true, 0 = false)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_controller_commands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float Va_c, float h_c, float chi_c, float phi_ff, const float *aux, uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN];
	_mav_put_float(buf, 0, Va_c);
	_mav_put_float(buf, 4, h_c);
	_mav_put_float(buf, 8, chi_c);
	_mav_put_float(buf, 12, phi_ff);
	_mav_put_uint8_t(buf, 32, aux_valid);
	_mav_put_float_array(buf, 16, aux, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#else
	mavlink_mav_controller_commands_t packet;
	packet.Va_c = Va_c;
	packet.h_c = h_c;
	packet.chi_c = chi_c;
	packet.phi_ff = phi_ff;
	packet.aux_valid = aux_valid;
	mav_array_memcpy(packet.aux, aux, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
}

/**
 * @brief Pack a mav_controller_commands message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Va_c Commanded airspeed (m/s)
 * @param h_c Commanded altitude (m)
 * @param chi_c Commanded course (rad)
 * @param phi_ff Feed forward command for orbits (rad)
 * @param aux Optional auxiliary commands
 * @param aux_valid Auxiliary commands valid (1 = true, 0 = false)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_controller_commands_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float Va_c,float h_c,float chi_c,float phi_ff,const float *aux,uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN];
	_mav_put_float(buf, 0, Va_c);
	_mav_put_float(buf, 4, h_c);
	_mav_put_float(buf, 8, chi_c);
	_mav_put_float(buf, 12, phi_ff);
	_mav_put_uint8_t(buf, 32, aux_valid);
	_mav_put_float_array(buf, 16, aux, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#else
	mavlink_mav_controller_commands_t packet;
	packet.Va_c = Va_c;
	packet.h_c = h_c;
	packet.chi_c = chi_c;
	packet.phi_ff = phi_ff;
	packet.aux_valid = aux_valid;
	mav_array_memcpy(packet.aux, aux, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
}

/**
 * @brief Encode a mav_controller_commands struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_controller_commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_controller_commands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_controller_commands_t* mav_controller_commands)
{
	return mavlink_msg_mav_controller_commands_pack(system_id, component_id, msg, mav_controller_commands->Va_c, mav_controller_commands->h_c, mav_controller_commands->chi_c, mav_controller_commands->phi_ff, mav_controller_commands->aux, mav_controller_commands->aux_valid);
}

/**
 * @brief Encode a mav_controller_commands struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_controller_commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_controller_commands_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_controller_commands_t* mav_controller_commands)
{
	return mavlink_msg_mav_controller_commands_pack_chan(system_id, component_id, chan, msg, mav_controller_commands->Va_c, mav_controller_commands->h_c, mav_controller_commands->chi_c, mav_controller_commands->phi_ff, mav_controller_commands->aux, mav_controller_commands->aux_valid);
}

/**
 * @brief Send a mav_controller_commands message
 * @param chan MAVLink channel to send the message
 *
 * @param Va_c Commanded airspeed (m/s)
 * @param h_c Commanded altitude (m)
 * @param chi_c Commanded course (rad)
 * @param phi_ff Feed forward command for orbits (rad)
 * @param aux Optional auxiliary commands
 * @param aux_valid Auxiliary commands valid (1 = true, 0 = false)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_controller_commands_send(mavlink_channel_t chan, float Va_c, float h_c, float chi_c, float phi_ff, const float *aux, uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN];
	_mav_put_float(buf, 0, Va_c);
	_mav_put_float(buf, 4, h_c);
	_mav_put_float(buf, 8, chi_c);
	_mav_put_float(buf, 12, phi_ff);
	_mav_put_uint8_t(buf, 32, aux_valid);
	_mav_put_float_array(buf, 16, aux, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
#else
	mavlink_mav_controller_commands_t packet;
	packet.Va_c = Va_c;
	packet.h_c = h_c;
	packet.chi_c = chi_c;
	packet.phi_ff = phi_ff;
	packet.aux_valid = aux_valid;
	mav_array_memcpy(packet.aux, aux, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, (const char *)&packet, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, (const char *)&packet, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_controller_commands_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Va_c, float h_c, float chi_c, float phi_ff, const float *aux, uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, Va_c);
	_mav_put_float(buf, 4, h_c);
	_mav_put_float(buf, 8, chi_c);
	_mav_put_float(buf, 12, phi_ff);
	_mav_put_uint8_t(buf, 32, aux_valid);
	_mav_put_float_array(buf, 16, aux, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
#else
	mavlink_mav_controller_commands_t *packet = (mavlink_mav_controller_commands_t *)msgbuf;
	packet->Va_c = Va_c;
	packet->h_c = h_c;
	packet->chi_c = chi_c;
	packet->phi_ff = phi_ff;
	packet->aux_valid = aux_valid;
	mav_array_memcpy(packet->aux, aux, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, (const char *)packet, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS, (const char *)packet, MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAV_CONTROLLER_COMMANDS UNPACKING


/**
 * @brief Get field Va_c from mav_controller_commands message
 *
 * @return Commanded airspeed (m/s)
 */
static inline float mavlink_msg_mav_controller_commands_get_Va_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field h_c from mav_controller_commands message
 *
 * @return Commanded altitude (m)
 */
static inline float mavlink_msg_mav_controller_commands_get_h_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field chi_c from mav_controller_commands message
 *
 * @return Commanded course (rad)
 */
static inline float mavlink_msg_mav_controller_commands_get_chi_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field phi_ff from mav_controller_commands message
 *
 * @return Feed forward command for orbits (rad)
 */
static inline float mavlink_msg_mav_controller_commands_get_phi_ff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field aux from mav_controller_commands message
 *
 * @return Optional auxiliary commands
 */
static inline uint16_t mavlink_msg_mav_controller_commands_get_aux(const mavlink_message_t* msg, float *aux)
{
	return _MAV_RETURN_float_array(msg, aux, 4,  16);
}

/**
 * @brief Get field aux_valid from mav_controller_commands message
 *
 * @return Auxiliary commands valid (1 = true, 0 = false)
 */
static inline uint8_t mavlink_msg_mav_controller_commands_get_aux_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Decode a mav_controller_commands message into a struct
 *
 * @param msg The message to decode
 * @param mav_controller_commands C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_controller_commands_decode(const mavlink_message_t* msg, mavlink_mav_controller_commands_t* mav_controller_commands)
{
#if MAVLINK_NEED_BYTE_SWAP
	mav_controller_commands->Va_c = mavlink_msg_mav_controller_commands_get_Va_c(msg);
	mav_controller_commands->h_c = mavlink_msg_mav_controller_commands_get_h_c(msg);
	mav_controller_commands->chi_c = mavlink_msg_mav_controller_commands_get_chi_c(msg);
	mav_controller_commands->phi_ff = mavlink_msg_mav_controller_commands_get_phi_ff(msg);
	mavlink_msg_mav_controller_commands_get_aux(msg, mav_controller_commands->aux);
	mav_controller_commands->aux_valid = mavlink_msg_mav_controller_commands_get_aux_valid(msg);
#else
	memcpy(mav_controller_commands, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS_LEN);
#endif
}
