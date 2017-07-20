// MESSAGE SMALL_SONAR PACKING

#define MAVLINK_MSG_ID_SMALL_SONAR 187

typedef struct __mavlink_small_sonar_t
{
 float range; /*< Range Measurement (m)*/
 float max_range; /*< Max Range (m)*/
 float min_range; /*< Min Range (m)*/
} mavlink_small_sonar_t;

#define MAVLINK_MSG_ID_SMALL_SONAR_LEN 12
#define MAVLINK_MSG_ID_187_LEN 12

#define MAVLINK_MSG_ID_SMALL_SONAR_CRC 100
#define MAVLINK_MSG_ID_187_CRC 100



#define MAVLINK_MESSAGE_INFO_SMALL_SONAR { \
	"SMALL_SONAR", \
	3, \
	{  { "range", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_small_sonar_t, range) }, \
         { "max_range", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_small_sonar_t, max_range) }, \
         { "min_range", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_small_sonar_t, min_range) }, \
         } \
}


/**
 * @brief Pack a small_sonar message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param range Range Measurement (m)
 * @param max_range Max Range (m)
 * @param min_range Min Range (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_sonar_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float range, float max_range, float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_SONAR_LEN];
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#else
	mavlink_small_sonar_t packet;
	packet.range = range;
	packet.max_range = max_range;
	packet.min_range = min_range;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SMALL_SONAR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_SONAR_LEN, MAVLINK_MSG_ID_SMALL_SONAR_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
}

/**
 * @brief Pack a small_sonar message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param range Range Measurement (m)
 * @param max_range Max Range (m)
 * @param min_range Min Range (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_sonar_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float range,float max_range,float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_SONAR_LEN];
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#else
	mavlink_small_sonar_t packet;
	packet.range = range;
	packet.max_range = max_range;
	packet.min_range = min_range;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SMALL_SONAR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_SONAR_LEN, MAVLINK_MSG_ID_SMALL_SONAR_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
}

/**
 * @brief Encode a small_sonar struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param small_sonar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_sonar_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_small_sonar_t* small_sonar)
{
	return mavlink_msg_small_sonar_pack(system_id, component_id, msg, small_sonar->range, small_sonar->max_range, small_sonar->min_range);
}

/**
 * @brief Encode a small_sonar struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param small_sonar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_sonar_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_small_sonar_t* small_sonar)
{
	return mavlink_msg_small_sonar_pack_chan(system_id, component_id, chan, msg, small_sonar->range, small_sonar->max_range, small_sonar->min_range);
}

/**
 * @brief Send a small_sonar message
 * @param chan MAVLink channel to send the message
 *
 * @param range Range Measurement (m)
 * @param max_range Max Range (m)
 * @param min_range Min Range (m)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_small_sonar_send(mavlink_channel_t chan, float range, float max_range, float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_SONAR_LEN];
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, buf, MAVLINK_MSG_ID_SMALL_SONAR_LEN, MAVLINK_MSG_ID_SMALL_SONAR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, buf, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
#else
	mavlink_small_sonar_t packet;
	packet.range = range;
	packet.max_range = max_range;
	packet.min_range = min_range;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, (const char *)&packet, MAVLINK_MSG_ID_SMALL_SONAR_LEN, MAVLINK_MSG_ID_SMALL_SONAR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, (const char *)&packet, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SMALL_SONAR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_small_sonar_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float range, float max_range, float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, buf, MAVLINK_MSG_ID_SMALL_SONAR_LEN, MAVLINK_MSG_ID_SMALL_SONAR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, buf, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
#else
	mavlink_small_sonar_t *packet = (mavlink_small_sonar_t *)msgbuf;
	packet->range = range;
	packet->max_range = max_range;
	packet->min_range = min_range;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, (const char *)packet, MAVLINK_MSG_ID_SMALL_SONAR_LEN, MAVLINK_MSG_ID_SMALL_SONAR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_SONAR, (const char *)packet, MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SMALL_SONAR UNPACKING


/**
 * @brief Get field range from small_sonar message
 *
 * @return Range Measurement (m)
 */
static inline float mavlink_msg_small_sonar_get_range(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field max_range from small_sonar message
 *
 * @return Max Range (m)
 */
static inline float mavlink_msg_small_sonar_get_max_range(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field min_range from small_sonar message
 *
 * @return Min Range (m)
 */
static inline float mavlink_msg_small_sonar_get_min_range(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a small_sonar message into a struct
 *
 * @param msg The message to decode
 * @param small_sonar C-struct to decode the message contents into
 */
static inline void mavlink_msg_small_sonar_decode(const mavlink_message_t* msg, mavlink_small_sonar_t* small_sonar)
{
#if MAVLINK_NEED_BYTE_SWAP
	small_sonar->range = mavlink_msg_small_sonar_get_range(msg);
	small_sonar->max_range = mavlink_msg_small_sonar_get_max_range(msg);
	small_sonar->min_range = mavlink_msg_small_sonar_get_min_range(msg);
#else
	memcpy(small_sonar, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SMALL_SONAR_LEN);
#endif
}
