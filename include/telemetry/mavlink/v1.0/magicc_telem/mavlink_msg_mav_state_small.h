// MESSAGE MAV_STATE_SMALL PACKING

#define MAVLINK_MSG_ID_MAV_STATE_SMALL 200

typedef struct __mavlink_mav_state_small_t
{
 float position[3]; /*< NED position (m)*/
 float Va; /*< Airspeed (m/s)*/
 float phi; /*< Roll angle (rad)*/
 float theta; /*< Pitch angle (rad)*/
 float psi; /*< Yaw angle (rad)*/
 float chi; /*< Course angle (rad)*/
 float initial_lat; /*< Initial/origin latitude (lat. deg)*/
 float initial_lon; /*< Initial/origin longitude (lon. deg)*/
 float initial_alt; /*< Initial/origin altitude (m)*/
} mavlink_mav_state_small_t;

#define MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN 44
#define MAVLINK_MSG_ID_200_LEN 44

#define MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC 179
#define MAVLINK_MSG_ID_200_CRC 179

#define MAVLINK_MSG_MAV_STATE_SMALL_FIELD_POSITION_LEN 3

#define MAVLINK_MESSAGE_INFO_MAV_STATE_SMALL { \
	"MAV_STATE_SMALL", \
	9, \
	{  { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_mav_state_small_t, position) }, \
         { "Va", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mav_state_small_t, Va) }, \
         { "phi", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mav_state_small_t, phi) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mav_state_small_t, theta) }, \
         { "psi", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mav_state_small_t, psi) }, \
         { "chi", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mav_state_small_t, chi) }, \
         { "initial_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_mav_state_small_t, initial_lat) }, \
         { "initial_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_mav_state_small_t, initial_lon) }, \
         { "initial_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_mav_state_small_t, initial_alt) }, \
         } \
}


/**
 * @brief Pack a mav_state_small message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param position NED position (m)
 * @param Va Airspeed (m/s)
 * @param phi Roll angle (rad)
 * @param theta Pitch angle (rad)
 * @param psi Yaw angle (rad)
 * @param chi Course angle (rad)
 * @param initial_lat Initial/origin latitude (lat. deg)
 * @param initial_lon Initial/origin longitude (lon. deg)
 * @param initial_alt Initial/origin altitude (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_state_small_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *position, float Va, float phi, float theta, float psi, float chi, float initial_lat, float initial_lon, float initial_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN];
	_mav_put_float(buf, 12, Va);
	_mav_put_float(buf, 16, phi);
	_mav_put_float(buf, 20, theta);
	_mav_put_float(buf, 24, psi);
	_mav_put_float(buf, 28, chi);
	_mav_put_float(buf, 32, initial_lat);
	_mav_put_float(buf, 36, initial_lon);
	_mav_put_float(buf, 40, initial_alt);
	_mav_put_float_array(buf, 0, position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#else
	mavlink_mav_state_small_t packet;
	packet.Va = Va;
	packet.phi = phi;
	packet.theta = theta;
	packet.psi = psi;
	packet.chi = chi;
	packet.initial_lat = initial_lat;
	packet.initial_lon = initial_lon;
	packet.initial_alt = initial_alt;
	mav_array_memcpy(packet.position, position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_STATE_SMALL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN, MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
}

/**
 * @brief Pack a mav_state_small message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param position NED position (m)
 * @param Va Airspeed (m/s)
 * @param phi Roll angle (rad)
 * @param theta Pitch angle (rad)
 * @param psi Yaw angle (rad)
 * @param chi Course angle (rad)
 * @param initial_lat Initial/origin latitude (lat. deg)
 * @param initial_lon Initial/origin longitude (lon. deg)
 * @param initial_alt Initial/origin altitude (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_state_small_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *position,float Va,float phi,float theta,float psi,float chi,float initial_lat,float initial_lon,float initial_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN];
	_mav_put_float(buf, 12, Va);
	_mav_put_float(buf, 16, phi);
	_mav_put_float(buf, 20, theta);
	_mav_put_float(buf, 24, psi);
	_mav_put_float(buf, 28, chi);
	_mav_put_float(buf, 32, initial_lat);
	_mav_put_float(buf, 36, initial_lon);
	_mav_put_float(buf, 40, initial_alt);
	_mav_put_float_array(buf, 0, position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#else
	mavlink_mav_state_small_t packet;
	packet.Va = Va;
	packet.phi = phi;
	packet.theta = theta;
	packet.psi = psi;
	packet.chi = chi;
	packet.initial_lat = initial_lat;
	packet.initial_lon = initial_lon;
	packet.initial_alt = initial_alt;
	mav_array_memcpy(packet.position, position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_STATE_SMALL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN, MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
}

/**
 * @brief Encode a mav_state_small struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_state_small C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_state_small_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_state_small_t* mav_state_small)
{
	return mavlink_msg_mav_state_small_pack(system_id, component_id, msg, mav_state_small->position, mav_state_small->Va, mav_state_small->phi, mav_state_small->theta, mav_state_small->psi, mav_state_small->chi, mav_state_small->initial_lat, mav_state_small->initial_lon, mav_state_small->initial_alt);
}

/**
 * @brief Encode a mav_state_small struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_state_small C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_state_small_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_state_small_t* mav_state_small)
{
	return mavlink_msg_mav_state_small_pack_chan(system_id, component_id, chan, msg, mav_state_small->position, mav_state_small->Va, mav_state_small->phi, mav_state_small->theta, mav_state_small->psi, mav_state_small->chi, mav_state_small->initial_lat, mav_state_small->initial_lon, mav_state_small->initial_alt);
}

/**
 * @brief Send a mav_state_small message
 * @param chan MAVLink channel to send the message
 *
 * @param position NED position (m)
 * @param Va Airspeed (m/s)
 * @param phi Roll angle (rad)
 * @param theta Pitch angle (rad)
 * @param psi Yaw angle (rad)
 * @param chi Course angle (rad)
 * @param initial_lat Initial/origin latitude (lat. deg)
 * @param initial_lon Initial/origin longitude (lon. deg)
 * @param initial_alt Initial/origin altitude (m)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_state_small_send(mavlink_channel_t chan, const float *position, float Va, float phi, float theta, float psi, float chi, float initial_lat, float initial_lon, float initial_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN];
	_mav_put_float(buf, 12, Va);
	_mav_put_float(buf, 16, phi);
	_mav_put_float(buf, 20, theta);
	_mav_put_float(buf, 24, psi);
	_mav_put_float(buf, 28, chi);
	_mav_put_float(buf, 32, initial_lat);
	_mav_put_float(buf, 36, initial_lon);
	_mav_put_float(buf, 40, initial_alt);
	_mav_put_float_array(buf, 0, position, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, buf, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN, MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, buf, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
#else
	mavlink_mav_state_small_t packet;
	packet.Va = Va;
	packet.phi = phi;
	packet.theta = theta;
	packet.psi = psi;
	packet.chi = chi;
	packet.initial_lat = initial_lat;
	packet.initial_lon = initial_lon;
	packet.initial_alt = initial_alt;
	mav_array_memcpy(packet.position, position, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, (const char *)&packet, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN, MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, (const char *)&packet, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_state_small_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *position, float Va, float phi, float theta, float psi, float chi, float initial_lat, float initial_lon, float initial_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 12, Va);
	_mav_put_float(buf, 16, phi);
	_mav_put_float(buf, 20, theta);
	_mav_put_float(buf, 24, psi);
	_mav_put_float(buf, 28, chi);
	_mav_put_float(buf, 32, initial_lat);
	_mav_put_float(buf, 36, initial_lon);
	_mav_put_float(buf, 40, initial_alt);
	_mav_put_float_array(buf, 0, position, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, buf, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN, MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, buf, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
#else
	mavlink_mav_state_small_t *packet = (mavlink_mav_state_small_t *)msgbuf;
	packet->Va = Va;
	packet->phi = phi;
	packet->theta = theta;
	packet->psi = psi;
	packet->chi = chi;
	packet->initial_lat = initial_lat;
	packet->initial_lon = initial_lon;
	packet->initial_alt = initial_alt;
	mav_array_memcpy(packet->position, position, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, (const char *)packet, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN, MAVLINK_MSG_ID_MAV_STATE_SMALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_STATE_SMALL, (const char *)packet, MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAV_STATE_SMALL UNPACKING


/**
 * @brief Get field position from mav_state_small message
 *
 * @return NED position (m)
 */
static inline uint16_t mavlink_msg_mav_state_small_get_position(const mavlink_message_t* msg, float *position)
{
	return _MAV_RETURN_float_array(msg, position, 3,  0);
}

/**
 * @brief Get field Va from mav_state_small message
 *
 * @return Airspeed (m/s)
 */
static inline float mavlink_msg_mav_state_small_get_Va(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field phi from mav_state_small message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_mav_state_small_get_phi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field theta from mav_state_small message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_mav_state_small_get_theta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field psi from mav_state_small message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_mav_state_small_get_psi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field chi from mav_state_small message
 *
 * @return Course angle (rad)
 */
static inline float mavlink_msg_mav_state_small_get_chi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field initial_lat from mav_state_small message
 *
 * @return Initial/origin latitude (lat. deg)
 */
static inline float mavlink_msg_mav_state_small_get_initial_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field initial_lon from mav_state_small message
 *
 * @return Initial/origin longitude (lon. deg)
 */
static inline float mavlink_msg_mav_state_small_get_initial_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field initial_alt from mav_state_small message
 *
 * @return Initial/origin altitude (m)
 */
static inline float mavlink_msg_mav_state_small_get_initial_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a mav_state_small message into a struct
 *
 * @param msg The message to decode
 * @param mav_state_small C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_state_small_decode(const mavlink_message_t* msg, mavlink_mav_state_small_t* mav_state_small)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_mav_state_small_get_position(msg, mav_state_small->position);
	mav_state_small->Va = mavlink_msg_mav_state_small_get_Va(msg);
	mav_state_small->phi = mavlink_msg_mav_state_small_get_phi(msg);
	mav_state_small->theta = mavlink_msg_mav_state_small_get_theta(msg);
	mav_state_small->psi = mavlink_msg_mav_state_small_get_psi(msg);
	mav_state_small->chi = mavlink_msg_mav_state_small_get_chi(msg);
	mav_state_small->initial_lat = mavlink_msg_mav_state_small_get_initial_lat(msg);
	mav_state_small->initial_lon = mavlink_msg_mav_state_small_get_initial_lon(msg);
	mav_state_small->initial_alt = mavlink_msg_mav_state_small_get_initial_alt(msg);
#else
	memcpy(mav_state_small, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAV_STATE_SMALL_LEN);
#endif
}
