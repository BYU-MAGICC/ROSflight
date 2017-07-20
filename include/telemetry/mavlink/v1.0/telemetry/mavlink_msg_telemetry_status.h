// MESSAGE telemetry_STATUS PACKING

#define MAVLINK_MSG_ID_telemetry_STATUS 191

typedef struct __mavlink_telemetry_status_t
{
 uint16_t num_errors; /*< */
 uint16_t loop_time_us; /*< */
 uint8_t status; /*< */
 uint8_t error_code; /*< */
 uint8_t control_mode; /*< */
} mavlink_telemetry_status_t;

#define MAVLINK_MSG_ID_telemetry_STATUS_LEN 7
#define MAVLINK_MSG_ID_191_LEN 7

#define MAVLINK_MSG_ID_telemetry_STATUS_CRC 135
#define MAVLINK_MSG_ID_191_CRC 135



#define MAVLINK_MESSAGE_INFO_telemetry_STATUS { \
	"telemetry_STATUS", \
	5, \
	{  { "num_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_telemetry_status_t, num_errors) }, \
         { "loop_time_us", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_telemetry_status_t, loop_time_us) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_telemetry_status_t, status) }, \
         { "error_code", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_telemetry_status_t, error_code) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_telemetry_status_t, control_mode) }, \
         } \
}


/**
 * @brief Pack a telemetry_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status 
 * @param error_code 
 * @param control_mode 
 * @param num_errors 
 * @param loop_time_us 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_telemetry_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t status, uint8_t error_code, uint8_t control_mode, uint16_t num_errors, uint16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_telemetry_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, num_errors);
	_mav_put_uint16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, status);
	_mav_put_uint8_t(buf, 5, error_code);
	_mav_put_uint8_t(buf, 6, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#else
	mavlink_telemetry_status_t packet;
	packet.num_errors = num_errors;
	packet.loop_time_us = loop_time_us;
	packet.status = status;
	packet.error_code = error_code;
	packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_telemetry_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_telemetry_STATUS_LEN, MAVLINK_MSG_ID_telemetry_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
}

/**
 * @brief Pack a telemetry_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status 
 * @param error_code 
 * @param control_mode 
 * @param num_errors 
 * @param loop_time_us 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_telemetry_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t status,uint8_t error_code,uint8_t control_mode,uint16_t num_errors,uint16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_telemetry_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, num_errors);
	_mav_put_uint16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, status);
	_mav_put_uint8_t(buf, 5, error_code);
	_mav_put_uint8_t(buf, 6, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#else
	mavlink_telemetry_status_t packet;
	packet.num_errors = num_errors;
	packet.loop_time_us = loop_time_us;
	packet.status = status;
	packet.error_code = error_code;
	packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_telemetry_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_telemetry_STATUS_LEN, MAVLINK_MSG_ID_telemetry_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
}

/**
 * @brief Encode a telemetry_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param telemetry_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_telemetry_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_telemetry_status_t* telemetry_status)
{
	return mavlink_msg_telemetry_status_pack(system_id, component_id, msg, telemetry_status->status, telemetry_status->error_code, telemetry_status->control_mode, telemetry_status->num_errors, telemetry_status->loop_time_us);
}

/**
 * @brief Encode a telemetry_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param telemetry_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_telemetry_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_telemetry_status_t* telemetry_status)
{
	return mavlink_msg_telemetry_status_pack_chan(system_id, component_id, chan, msg, telemetry_status->status, telemetry_status->error_code, telemetry_status->control_mode, telemetry_status->num_errors, telemetry_status->loop_time_us);
}

/**
 * @brief Send a telemetry_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status 
 * @param error_code 
 * @param control_mode 
 * @param num_errors 
 * @param loop_time_us 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_telemetry_status_send(mavlink_channel_t chan, uint8_t status, uint8_t error_code, uint8_t control_mode, uint16_t num_errors, uint16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_telemetry_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, num_errors);
	_mav_put_uint16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, status);
	_mav_put_uint8_t(buf, 5, error_code);
	_mav_put_uint8_t(buf, 6, control_mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, buf, MAVLINK_MSG_ID_telemetry_STATUS_LEN, MAVLINK_MSG_ID_telemetry_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, buf, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
#else
	mavlink_telemetry_status_t packet;
	packet.num_errors = num_errors;
	packet.loop_time_us = loop_time_us;
	packet.status = status;
	packet.error_code = error_code;
	packet.control_mode = control_mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, (const char *)&packet, MAVLINK_MSG_ID_telemetry_STATUS_LEN, MAVLINK_MSG_ID_telemetry_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, (const char *)&packet, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_telemetry_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_telemetry_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status, uint8_t error_code, uint8_t control_mode, uint16_t num_errors, uint16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, num_errors);
	_mav_put_uint16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, status);
	_mav_put_uint8_t(buf, 5, error_code);
	_mav_put_uint8_t(buf, 6, control_mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, buf, MAVLINK_MSG_ID_telemetry_STATUS_LEN, MAVLINK_MSG_ID_telemetry_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, buf, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
#else
	mavlink_telemetry_status_t *packet = (mavlink_telemetry_status_t *)msgbuf;
	packet->num_errors = num_errors;
	packet->loop_time_us = loop_time_us;
	packet->status = status;
	packet->error_code = error_code;
	packet->control_mode = control_mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, (const char *)packet, MAVLINK_MSG_ID_telemetry_STATUS_LEN, MAVLINK_MSG_ID_telemetry_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_telemetry_STATUS, (const char *)packet, MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE telemetry_STATUS UNPACKING


/**
 * @brief Get field status from telemetry_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_telemetry_status_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field error_code from telemetry_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_telemetry_status_get_error_code(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field control_mode from telemetry_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_telemetry_status_get_control_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field num_errors from telemetry_status message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_telemetry_status_get_num_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field loop_time_us from telemetry_status message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_telemetry_status_get_loop_time_us(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a telemetry_status message into a struct
 *
 * @param msg The message to decode
 * @param telemetry_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_telemetry_status_decode(const mavlink_message_t* msg, mavlink_telemetry_status_t* telemetry_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	telemetry_status->num_errors = mavlink_msg_telemetry_status_get_num_errors(msg);
	telemetry_status->loop_time_us = mavlink_msg_telemetry_status_get_loop_time_us(msg);
	telemetry_status->status = mavlink_msg_telemetry_status_get_status(msg);
	telemetry_status->error_code = mavlink_msg_telemetry_status_get_error_code(msg);
	telemetry_status->control_mode = mavlink_msg_telemetry_status_get_control_mode(msg);
#else
	memcpy(telemetry_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_telemetry_STATUS_LEN);
#endif
}
