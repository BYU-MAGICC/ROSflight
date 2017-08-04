// MESSAGE RC_RAW PACKING

#define MAVLINK_MSG_ID_RC_RAW 203

typedef struct __mavlink_rc_raw_t
{
 uint16_t values[8]; /*< RC Channel values*/
} mavlink_rc_raw_t;

#define MAVLINK_MSG_ID_RC_RAW_LEN 16
#define MAVLINK_MSG_ID_203_LEN 16

#define MAVLINK_MSG_ID_RC_RAW_CRC 110
#define MAVLINK_MSG_ID_203_CRC 110

#define MAVLINK_MSG_RC_RAW_FIELD_VALUES_LEN 8

#define MAVLINK_MESSAGE_INFO_RC_RAW { \
	"RC_RAW", \
	1, \
	{  { "values", NULL, MAVLINK_TYPE_UINT16_T, 8, 0, offsetof(mavlink_rc_raw_t, values) }, \
         } \
}


/**
 * @brief Pack a rc_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param values RC Channel values
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_RAW_LEN];

	_mav_put_uint16_t_array(buf, 0, values, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_RAW_LEN);
#else
	mavlink_rc_raw_t packet;

	mav_array_memcpy(packet.values, values, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_RAW_LEN, MAVLINK_MSG_ID_RC_RAW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
}

/**
 * @brief Pack a rc_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param values RC Channel values
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_RAW_LEN];

	_mav_put_uint16_t_array(buf, 0, values, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_RAW_LEN);
#else
	mavlink_rc_raw_t packet;

	mav_array_memcpy(packet.values, values, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_RAW_LEN, MAVLINK_MSG_ID_RC_RAW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
}

/**
 * @brief Encode a rc_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_raw_t* rc_raw)
{
	return mavlink_msg_rc_raw_pack(system_id, component_id, msg, rc_raw->values);
}

/**
 * @brief Encode a rc_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_raw_t* rc_raw)
{
	return mavlink_msg_rc_raw_pack_chan(system_id, component_id, chan, msg, rc_raw->values);
}

/**
 * @brief Send a rc_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param values RC Channel values
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_raw_send(mavlink_channel_t chan, const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_RAW_LEN];

	_mav_put_uint16_t_array(buf, 0, values, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, buf, MAVLINK_MSG_ID_RC_RAW_LEN, MAVLINK_MSG_ID_RC_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, buf, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
#else
	mavlink_rc_raw_t packet;

	mav_array_memcpy(packet.values, values, sizeof(uint16_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, (const char *)&packet, MAVLINK_MSG_ID_RC_RAW_LEN, MAVLINK_MSG_ID_RC_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, (const char *)&packet, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RC_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rc_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_uint16_t_array(buf, 0, values, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, buf, MAVLINK_MSG_ID_RC_RAW_LEN, MAVLINK_MSG_ID_RC_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, buf, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
#else
	mavlink_rc_raw_t *packet = (mavlink_rc_raw_t *)msgbuf;

	mav_array_memcpy(packet->values, values, sizeof(uint16_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, (const char *)packet, MAVLINK_MSG_ID_RC_RAW_LEN, MAVLINK_MSG_ID_RC_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_RAW, (const char *)packet, MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RC_RAW UNPACKING


/**
 * @brief Get field values from rc_raw message
 *
 * @return RC Channel values
 */
static inline uint16_t mavlink_msg_rc_raw_get_values(const mavlink_message_t* msg, uint16_t *values)
{
	return _MAV_RETURN_uint16_t_array(msg, values, 8,  0);
}

/**
 * @brief Decode a rc_raw message into a struct
 *
 * @param msg The message to decode
 * @param rc_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_raw_decode(const mavlink_message_t* msg, mavlink_rc_raw_t* rc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_rc_raw_get_values(msg, rc_raw->values);
#else
	memcpy(rc_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RC_RAW_LEN);
#endif
}
