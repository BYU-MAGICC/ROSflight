// MESSAGE MAV_CONTROLLER_INTERNALS PACKING

#define MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS 205

typedef struct __mavlink_mav_controller_internals_t
{
 float theta_c; /*< Commanded pitch (rad)*/
 float phi_c; /*< Commanded roll (rad)*/
 float aux[4]; /*< Optional auxiliary commands*/
 uint8_t alt_zone; /*< Zone in the latitude state machine*/
 uint8_t aux_valid; /*< Auxiliary commands valid (1 = true, 0 = false)*/
} mavlink_mav_controller_internals_t;

#define MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN 26
#define MAVLINK_MSG_ID_205_LEN 26

#define MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC 131
#define MAVLINK_MSG_ID_205_CRC 131

#define MAVLINK_MSG_MAV_CONTROLLER_INTERNALS_FIELD_AUX_LEN 4

#define MAVLINK_MESSAGE_INFO_MAV_CONTROLLER_INTERNALS { \
	"MAV_CONTROLLER_INTERNALS", \
	5, \
	{  { "theta_c", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mav_controller_internals_t, theta_c) }, \
         { "phi_c", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mav_controller_internals_t, phi_c) }, \
         { "aux", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_mav_controller_internals_t, aux) }, \
         { "alt_zone", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_mav_controller_internals_t, alt_zone) }, \
         { "aux_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_mav_controller_internals_t, aux_valid) }, \
         } \
}


/**
 * @brief Pack a mav_controller_internals message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param theta_c Commanded pitch (rad)
 * @param phi_c Commanded roll (rad)
 * @param alt_zone Zone in the latitude state machine
 * @param aux Optional auxiliary commands
 * @param aux_valid Auxiliary commands valid (1 = true, 0 = false)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_controller_internals_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float theta_c, float phi_c, uint8_t alt_zone, const float *aux, uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN];
	_mav_put_float(buf, 0, theta_c);
	_mav_put_float(buf, 4, phi_c);
	_mav_put_uint8_t(buf, 24, alt_zone);
	_mav_put_uint8_t(buf, 25, aux_valid);
	_mav_put_float_array(buf, 8, aux, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#else
	mavlink_mav_controller_internals_t packet;
	packet.theta_c = theta_c;
	packet.phi_c = phi_c;
	packet.alt_zone = alt_zone;
	packet.aux_valid = aux_valid;
	mav_array_memcpy(packet.aux, aux, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
}

/**
 * @brief Pack a mav_controller_internals message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param theta_c Commanded pitch (rad)
 * @param phi_c Commanded roll (rad)
 * @param alt_zone Zone in the latitude state machine
 * @param aux Optional auxiliary commands
 * @param aux_valid Auxiliary commands valid (1 = true, 0 = false)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_controller_internals_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float theta_c,float phi_c,uint8_t alt_zone,const float *aux,uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN];
	_mav_put_float(buf, 0, theta_c);
	_mav_put_float(buf, 4, phi_c);
	_mav_put_uint8_t(buf, 24, alt_zone);
	_mav_put_uint8_t(buf, 25, aux_valid);
	_mav_put_float_array(buf, 8, aux, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#else
	mavlink_mav_controller_internals_t packet;
	packet.theta_c = theta_c;
	packet.phi_c = phi_c;
	packet.alt_zone = alt_zone;
	packet.aux_valid = aux_valid;
	mav_array_memcpy(packet.aux, aux, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
}

/**
 * @brief Encode a mav_controller_internals struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_controller_internals C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_controller_internals_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_controller_internals_t* mav_controller_internals)
{
	return mavlink_msg_mav_controller_internals_pack(system_id, component_id, msg, mav_controller_internals->theta_c, mav_controller_internals->phi_c, mav_controller_internals->alt_zone, mav_controller_internals->aux, mav_controller_internals->aux_valid);
}

/**
 * @brief Encode a mav_controller_internals struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_controller_internals C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_controller_internals_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_controller_internals_t* mav_controller_internals)
{
	return mavlink_msg_mav_controller_internals_pack_chan(system_id, component_id, chan, msg, mav_controller_internals->theta_c, mav_controller_internals->phi_c, mav_controller_internals->alt_zone, mav_controller_internals->aux, mav_controller_internals->aux_valid);
}

/**
 * @brief Send a mav_controller_internals message
 * @param chan MAVLink channel to send the message
 *
 * @param theta_c Commanded pitch (rad)
 * @param phi_c Commanded roll (rad)
 * @param alt_zone Zone in the latitude state machine
 * @param aux Optional auxiliary commands
 * @param aux_valid Auxiliary commands valid (1 = true, 0 = false)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_controller_internals_send(mavlink_channel_t chan, float theta_c, float phi_c, uint8_t alt_zone, const float *aux, uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN];
	_mav_put_float(buf, 0, theta_c);
	_mav_put_float(buf, 4, phi_c);
	_mav_put_uint8_t(buf, 24, alt_zone);
	_mav_put_uint8_t(buf, 25, aux_valid);
	_mav_put_float_array(buf, 8, aux, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
#else
	mavlink_mav_controller_internals_t packet;
	packet.theta_c = theta_c;
	packet.phi_c = phi_c;
	packet.alt_zone = alt_zone;
	packet.aux_valid = aux_valid;
	mav_array_memcpy(packet.aux, aux, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, (const char *)&packet, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, (const char *)&packet, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_controller_internals_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float theta_c, float phi_c, uint8_t alt_zone, const float *aux, uint8_t aux_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, theta_c);
	_mav_put_float(buf, 4, phi_c);
	_mav_put_uint8_t(buf, 24, alt_zone);
	_mav_put_uint8_t(buf, 25, aux_valid);
	_mav_put_float_array(buf, 8, aux, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, buf, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
#else
	mavlink_mav_controller_internals_t *packet = (mavlink_mav_controller_internals_t *)msgbuf;
	packet->theta_c = theta_c;
	packet->phi_c = phi_c;
	packet->alt_zone = alt_zone;
	packet->aux_valid = aux_valid;
	mav_array_memcpy(packet->aux, aux, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, (const char *)packet, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS, (const char *)packet, MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAV_CONTROLLER_INTERNALS UNPACKING


/**
 * @brief Get field theta_c from mav_controller_internals message
 *
 * @return Commanded pitch (rad)
 */
static inline float mavlink_msg_mav_controller_internals_get_theta_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field phi_c from mav_controller_internals message
 *
 * @return Commanded roll (rad)
 */
static inline float mavlink_msg_mav_controller_internals_get_phi_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field alt_zone from mav_controller_internals message
 *
 * @return Zone in the latitude state machine
 */
static inline uint8_t mavlink_msg_mav_controller_internals_get_alt_zone(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field aux from mav_controller_internals message
 *
 * @return Optional auxiliary commands
 */
static inline uint16_t mavlink_msg_mav_controller_internals_get_aux(const mavlink_message_t* msg, float *aux)
{
	return _MAV_RETURN_float_array(msg, aux, 4,  8);
}

/**
 * @brief Get field aux_valid from mav_controller_internals message
 *
 * @return Auxiliary commands valid (1 = true, 0 = false)
 */
static inline uint8_t mavlink_msg_mav_controller_internals_get_aux_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Decode a mav_controller_internals message into a struct
 *
 * @param msg The message to decode
 * @param mav_controller_internals C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_controller_internals_decode(const mavlink_message_t* msg, mavlink_mav_controller_internals_t* mav_controller_internals)
{
#if MAVLINK_NEED_BYTE_SWAP
	mav_controller_internals->theta_c = mavlink_msg_mav_controller_internals_get_theta_c(msg);
	mav_controller_internals->phi_c = mavlink_msg_mav_controller_internals_get_phi_c(msg);
	mavlink_msg_mav_controller_internals_get_aux(msg, mav_controller_internals->aux);
	mav_controller_internals->alt_zone = mavlink_msg_mav_controller_internals_get_alt_zone(msg);
	mav_controller_internals->aux_valid = mavlink_msg_mav_controller_internals_get_aux_valid(msg);
#else
	memcpy(mav_controller_internals, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS_LEN);
#endif
}
