// MESSAGE MAV_WAYPOINT PACKING

#define MAVLINK_MSG_ID_MAV_WAYPOINT 202

typedef struct __mavlink_mav_waypoint_t
{
 float w[3]; /*< Waypoint in local NED (m)*/
 float chi_d; /*< Desired course at this waypoint (rad)*/
 float Va_d; /*< Desired airspeed (m/s)*/
 uint8_t chi_valid; /*< Desired course valid (1 = valid, 0 = !valid)*/
 uint8_t set_current; /*< Sets this waypoint to be executed now (1 = true, 0 = false)*/
 uint8_t clear_wp_list; /*< Removes all waypoints and returns to origin. The rest of this message will be ignored.*/
} mavlink_mav_waypoint_t;

#define MAVLINK_MSG_ID_MAV_WAYPOINT_LEN 23
#define MAVLINK_MSG_ID_202_LEN 23

#define MAVLINK_MSG_ID_MAV_WAYPOINT_CRC 193
#define MAVLINK_MSG_ID_202_CRC 193

#define MAVLINK_MSG_MAV_WAYPOINT_FIELD_W_LEN 3

#define MAVLINK_MESSAGE_INFO_MAV_WAYPOINT { \
	"MAV_WAYPOINT", \
	6, \
	{  { "w", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_mav_waypoint_t, w) }, \
         { "chi_d", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mav_waypoint_t, chi_d) }, \
         { "Va_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mav_waypoint_t, Va_d) }, \
         { "chi_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mav_waypoint_t, chi_valid) }, \
         { "set_current", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_mav_waypoint_t, set_current) }, \
         { "clear_wp_list", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mav_waypoint_t, clear_wp_list) }, \
         } \
}


/**
 * @brief Pack a mav_waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param w Waypoint in local NED (m)
 * @param chi_d Desired course at this waypoint (rad)
 * @param chi_valid Desired course valid (1 = valid, 0 = !valid)
 * @param Va_d Desired airspeed (m/s)
 * @param set_current Sets this waypoint to be executed now (1 = true, 0 = false)
 * @param clear_wp_list Removes all waypoints and returns to origin. The rest of this message will be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *w, float chi_d, uint8_t chi_valid, float Va_d, uint8_t set_current, uint8_t clear_wp_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_WAYPOINT_LEN];
	_mav_put_float(buf, 12, chi_d);
	_mav_put_float(buf, 16, Va_d);
	_mav_put_uint8_t(buf, 20, chi_valid);
	_mav_put_uint8_t(buf, 21, set_current);
	_mav_put_uint8_t(buf, 22, clear_wp_list);
	_mav_put_float_array(buf, 0, w, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#else
	mavlink_mav_waypoint_t packet;
	packet.chi_d = chi_d;
	packet.Va_d = Va_d;
	packet.chi_valid = chi_valid;
	packet.set_current = set_current;
	packet.clear_wp_list = clear_wp_list;
	mav_array_memcpy(packet.w, w, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_WAYPOINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN, MAVLINK_MSG_ID_MAV_WAYPOINT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
}

/**
 * @brief Pack a mav_waypoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param w Waypoint in local NED (m)
 * @param chi_d Desired course at this waypoint (rad)
 * @param chi_valid Desired course valid (1 = valid, 0 = !valid)
 * @param Va_d Desired airspeed (m/s)
 * @param set_current Sets this waypoint to be executed now (1 = true, 0 = false)
 * @param clear_wp_list Removes all waypoints and returns to origin. The rest of this message will be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *w,float chi_d,uint8_t chi_valid,float Va_d,uint8_t set_current,uint8_t clear_wp_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_WAYPOINT_LEN];
	_mav_put_float(buf, 12, chi_d);
	_mav_put_float(buf, 16, Va_d);
	_mav_put_uint8_t(buf, 20, chi_valid);
	_mav_put_uint8_t(buf, 21, set_current);
	_mav_put_uint8_t(buf, 22, clear_wp_list);
	_mav_put_float_array(buf, 0, w, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#else
	mavlink_mav_waypoint_t packet;
	packet.chi_d = chi_d;
	packet.Va_d = Va_d;
	packet.chi_valid = chi_valid;
	packet.set_current = set_current;
	packet.clear_wp_list = clear_wp_list;
	mav_array_memcpy(packet.w, w, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_WAYPOINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN, MAVLINK_MSG_ID_MAV_WAYPOINT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
}

/**
 * @brief Encode a mav_waypoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_waypoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_waypoint_t* mav_waypoint)
{
	return mavlink_msg_mav_waypoint_pack(system_id, component_id, msg, mav_waypoint->w, mav_waypoint->chi_d, mav_waypoint->chi_valid, mav_waypoint->Va_d, mav_waypoint->set_current, mav_waypoint->clear_wp_list);
}

/**
 * @brief Encode a mav_waypoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_waypoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_waypoint_t* mav_waypoint)
{
	return mavlink_msg_mav_waypoint_pack_chan(system_id, component_id, chan, msg, mav_waypoint->w, mav_waypoint->chi_d, mav_waypoint->chi_valid, mav_waypoint->Va_d, mav_waypoint->set_current, mav_waypoint->clear_wp_list);
}

/**
 * @brief Send a mav_waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param w Waypoint in local NED (m)
 * @param chi_d Desired course at this waypoint (rad)
 * @param chi_valid Desired course valid (1 = valid, 0 = !valid)
 * @param Va_d Desired airspeed (m/s)
 * @param set_current Sets this waypoint to be executed now (1 = true, 0 = false)
 * @param clear_wp_list Removes all waypoints and returns to origin. The rest of this message will be ignored.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_waypoint_send(mavlink_channel_t chan, const float *w, float chi_d, uint8_t chi_valid, float Va_d, uint8_t set_current, uint8_t clear_wp_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_WAYPOINT_LEN];
	_mav_put_float(buf, 12, chi_d);
	_mav_put_float(buf, 16, Va_d);
	_mav_put_uint8_t(buf, 20, chi_valid);
	_mav_put_uint8_t(buf, 21, set_current);
	_mav_put_uint8_t(buf, 22, clear_wp_list);
	_mav_put_float_array(buf, 0, w, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, buf, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN, MAVLINK_MSG_ID_MAV_WAYPOINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, buf, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
#else
	mavlink_mav_waypoint_t packet;
	packet.chi_d = chi_d;
	packet.Va_d = Va_d;
	packet.chi_valid = chi_valid;
	packet.set_current = set_current;
	packet.clear_wp_list = clear_wp_list;
	mav_array_memcpy(packet.w, w, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, (const char *)&packet, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN, MAVLINK_MSG_ID_MAV_WAYPOINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, (const char *)&packet, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAV_WAYPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_waypoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *w, float chi_d, uint8_t chi_valid, float Va_d, uint8_t set_current, uint8_t clear_wp_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 12, chi_d);
	_mav_put_float(buf, 16, Va_d);
	_mav_put_uint8_t(buf, 20, chi_valid);
	_mav_put_uint8_t(buf, 21, set_current);
	_mav_put_uint8_t(buf, 22, clear_wp_list);
	_mav_put_float_array(buf, 0, w, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, buf, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN, MAVLINK_MSG_ID_MAV_WAYPOINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, buf, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
#else
	mavlink_mav_waypoint_t *packet = (mavlink_mav_waypoint_t *)msgbuf;
	packet->chi_d = chi_d;
	packet->Va_d = Va_d;
	packet->chi_valid = chi_valid;
	packet->set_current = set_current;
	packet->clear_wp_list = clear_wp_list;
	mav_array_memcpy(packet->w, w, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, (const char *)packet, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN, MAVLINK_MSG_ID_MAV_WAYPOINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_WAYPOINT, (const char *)packet, MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAV_WAYPOINT UNPACKING


/**
 * @brief Get field w from mav_waypoint message
 *
 * @return Waypoint in local NED (m)
 */
static inline uint16_t mavlink_msg_mav_waypoint_get_w(const mavlink_message_t* msg, float *w)
{
	return _MAV_RETURN_float_array(msg, w, 3,  0);
}

/**
 * @brief Get field chi_d from mav_waypoint message
 *
 * @return Desired course at this waypoint (rad)
 */
static inline float mavlink_msg_mav_waypoint_get_chi_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field chi_valid from mav_waypoint message
 *
 * @return Desired course valid (1 = valid, 0 = !valid)
 */
static inline uint8_t mavlink_msg_mav_waypoint_get_chi_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field Va_d from mav_waypoint message
 *
 * @return Desired airspeed (m/s)
 */
static inline float mavlink_msg_mav_waypoint_get_Va_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field set_current from mav_waypoint message
 *
 * @return Sets this waypoint to be executed now (1 = true, 0 = false)
 */
static inline uint8_t mavlink_msg_mav_waypoint_get_set_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field clear_wp_list from mav_waypoint message
 *
 * @return Removes all waypoints and returns to origin. The rest of this message will be ignored.
 */
static inline uint8_t mavlink_msg_mav_waypoint_get_clear_wp_list(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Decode a mav_waypoint message into a struct
 *
 * @param msg The message to decode
 * @param mav_waypoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_waypoint_decode(const mavlink_message_t* msg, mavlink_mav_waypoint_t* mav_waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_mav_waypoint_get_w(msg, mav_waypoint->w);
	mav_waypoint->chi_d = mavlink_msg_mav_waypoint_get_chi_d(msg);
	mav_waypoint->Va_d = mavlink_msg_mav_waypoint_get_Va_d(msg);
	mav_waypoint->chi_valid = mavlink_msg_mav_waypoint_get_chi_valid(msg);
	mav_waypoint->set_current = mavlink_msg_mav_waypoint_get_set_current(msg);
	mav_waypoint->clear_wp_list = mavlink_msg_mav_waypoint_get_clear_wp_list(msg);
#else
	memcpy(mav_waypoint, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAV_WAYPOINT_LEN);
#endif
}
