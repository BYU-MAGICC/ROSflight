// MESSAGE telemetry_OUTPUT_RAW PACKING

#define MAVLINK_MSG_ID_telemetry_OUTPUT_RAW 190

typedef struct __mavlink_telemetry_output_raw_t
{
 uint64_t stamp; /*< */
 float values[8]; /*< */
} mavlink_telemetry_output_raw_t;

#define MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN 40
#define MAVLINK_MSG_ID_190_LEN 40

#define MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC 230
#define MAVLINK_MSG_ID_190_CRC 230

#define MAVLINK_MSG_telemetry_OUTPUT_RAW_FIELD_VALUES_LEN 8

#define MAVLINK_MESSAGE_INFO_telemetry_OUTPUT_RAW { \
	"telemetry_OUTPUT_RAW", \
	2, \
	{  { "stamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_telemetry_output_raw_t, stamp) }, \
         { "values", NULL, MAVLINK_TYPE_FLOAT, 8, 8, offsetof(mavlink_telemetry_output_raw_t, values) }, \
         } \
}


/**
 * @brief Pack a telemetry_output_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param stamp 
 * @param values 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_telemetry_output_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t stamp, const float *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN];
	_mav_put_uint64_t(buf, 0, stamp);
	_mav_put_float_array(buf, 8, values, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#else
	mavlink_telemetry_output_raw_t packet;
	packet.stamp = stamp;
	mav_array_memcpy(packet.values, values, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_telemetry_OUTPUT_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
}

/**
 * @brief Pack a telemetry_output_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stamp 
 * @param values 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_telemetry_output_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t stamp,const float *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN];
	_mav_put_uint64_t(buf, 0, stamp);
	_mav_put_float_array(buf, 8, values, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#else
	mavlink_telemetry_output_raw_t packet;
	packet.stamp = stamp;
	mav_array_memcpy(packet.values, values, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_telemetry_OUTPUT_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
}

/**
 * @brief Encode a telemetry_output_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param telemetry_output_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_telemetry_output_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_telemetry_output_raw_t* telemetry_output_raw)
{
	return mavlink_msg_telemetry_output_raw_pack(system_id, component_id, msg, telemetry_output_raw->stamp, telemetry_output_raw->values);
}

/**
 * @brief Encode a telemetry_output_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param telemetry_output_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_telemetry_output_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_telemetry_output_raw_t* telemetry_output_raw)
{
	return mavlink_msg_telemetry_output_raw_pack_chan(system_id, component_id, chan, msg, telemetry_output_raw->stamp, telemetry_output_raw->values);
}

/**
 * @brief Send a telemetry_output_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param stamp 
 * @param values 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_telemetry_output_raw_send(mavlink_channel_t chan, uint64_t stamp, const float *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN];
	_mav_put_uint64_t(buf, 0, stamp);
	_mav_put_float_array(buf, 8, values, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, buf, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, buf, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
#else
	mavlink_telemetry_output_raw_t packet;
	packet.stamp = stamp;
	mav_array_memcpy(packet.values, values, sizeof(float)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, (const char *)&packet, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, (const char *)&packet, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_telemetry_output_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t stamp, const float *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, stamp);
	_mav_put_float_array(buf, 8, values, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, buf, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, buf, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
#else
	mavlink_telemetry_output_raw_t *packet = (mavlink_telemetry_output_raw_t *)msgbuf;
	packet->stamp = stamp;
	mav_array_memcpy(packet->values, values, sizeof(float)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, (const char *)packet, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW, (const char *)packet, MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE telemetry_OUTPUT_RAW UNPACKING


/**
 * @brief Get field stamp from telemetry_output_raw message
 *
 * @return 
 */
static inline uint64_t mavlink_msg_telemetry_output_raw_get_stamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field values from telemetry_output_raw message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_telemetry_output_raw_get_values(const mavlink_message_t* msg, float *values)
{
	return _MAV_RETURN_float_array(msg, values, 8,  8);
}

/**
 * @brief Decode a telemetry_output_raw message into a struct
 *
 * @param msg The message to decode
 * @param telemetry_output_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_telemetry_output_raw_decode(const mavlink_message_t* msg, mavlink_telemetry_output_raw_t* telemetry_output_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	telemetry_output_raw->stamp = mavlink_msg_telemetry_output_raw_get_stamp(msg);
	mavlink_msg_telemetry_output_raw_get_values(msg, telemetry_output_raw->values);
#else
	memcpy(telemetry_output_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_telemetry_OUTPUT_RAW_LEN);
#endif
}
