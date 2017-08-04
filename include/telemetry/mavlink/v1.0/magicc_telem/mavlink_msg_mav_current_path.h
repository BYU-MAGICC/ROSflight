// MESSAGE MAV_CURRENT_PATH PACKING

#define MAVLINK_MSG_ID_MAV_CURRENT_PATH 201

typedef struct __mavlink_mav_current_path_t
{
 float Va_d; /*< Desired airspeed (m/s)*/
 float r[3]; /*< Vector to origin of straight line path (m)*/
 float q[3]; /*< Unit vector, desired direction of travel for line path*/
 float c[3]; /*< Center of orbital path (m)*/
 float rho; /*< Radius of orbital path (m)*/
 uint8_t flag; /*< Indicates strait line or orbital path (1 = line, 0 = orbit)*/
 int8_t lambda; /*< Direction of orbital path (clockwise is 1, counterclockwise is -1)*/
} mavlink_mav_current_path_t;

#define MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN 46
#define MAVLINK_MSG_ID_201_LEN 46

#define MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC 218
#define MAVLINK_MSG_ID_201_CRC 218

#define MAVLINK_MSG_MAV_CURRENT_PATH_FIELD_R_LEN 3
#define MAVLINK_MSG_MAV_CURRENT_PATH_FIELD_Q_LEN 3
#define MAVLINK_MSG_MAV_CURRENT_PATH_FIELD_C_LEN 3

#define MAVLINK_MESSAGE_INFO_MAV_CURRENT_PATH { \
	"MAV_CURRENT_PATH", \
	7, \
	{  { "Va_d", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mav_current_path_t, Va_d) }, \
         { "r", NULL, MAVLINK_TYPE_FLOAT, 3, 4, offsetof(mavlink_mav_current_path_t, r) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_mav_current_path_t, q) }, \
         { "c", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_mav_current_path_t, c) }, \
         { "rho", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_mav_current_path_t, rho) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_mav_current_path_t, flag) }, \
         { "lambda", NULL, MAVLINK_TYPE_INT8_T, 0, 45, offsetof(mavlink_mav_current_path_t, lambda) }, \
         } \
}


/**
 * @brief Pack a mav_current_path message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flag Indicates strait line or orbital path (1 = line, 0 = orbit)
 * @param Va_d Desired airspeed (m/s)
 * @param r Vector to origin of straight line path (m)
 * @param q Unit vector, desired direction of travel for line path
 * @param c Center of orbital path (m)
 * @param rho Radius of orbital path (m)
 * @param lambda Direction of orbital path (clockwise is 1, counterclockwise is -1)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_current_path_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t flag, float Va_d, const float *r, const float *q, const float *c, float rho, int8_t lambda)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN];
	_mav_put_float(buf, 0, Va_d);
	_mav_put_float(buf, 40, rho);
	_mav_put_uint8_t(buf, 44, flag);
	_mav_put_int8_t(buf, 45, lambda);
	_mav_put_float_array(buf, 4, r, 3);
	_mav_put_float_array(buf, 16, q, 3);
	_mav_put_float_array(buf, 28, c, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#else
	mavlink_mav_current_path_t packet;
	packet.Va_d = Va_d;
	packet.rho = rho;
	packet.flag = flag;
	packet.lambda = lambda;
	mav_array_memcpy(packet.r, r, sizeof(float)*3);
	mav_array_memcpy(packet.q, q, sizeof(float)*3);
	mav_array_memcpy(packet.c, c, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_CURRENT_PATH;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN, MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
}

/**
 * @brief Pack a mav_current_path message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flag Indicates strait line or orbital path (1 = line, 0 = orbit)
 * @param Va_d Desired airspeed (m/s)
 * @param r Vector to origin of straight line path (m)
 * @param q Unit vector, desired direction of travel for line path
 * @param c Center of orbital path (m)
 * @param rho Radius of orbital path (m)
 * @param lambda Direction of orbital path (clockwise is 1, counterclockwise is -1)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_current_path_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t flag,float Va_d,const float *r,const float *q,const float *c,float rho,int8_t lambda)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN];
	_mav_put_float(buf, 0, Va_d);
	_mav_put_float(buf, 40, rho);
	_mav_put_uint8_t(buf, 44, flag);
	_mav_put_int8_t(buf, 45, lambda);
	_mav_put_float_array(buf, 4, r, 3);
	_mav_put_float_array(buf, 16, q, 3);
	_mav_put_float_array(buf, 28, c, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#else
	mavlink_mav_current_path_t packet;
	packet.Va_d = Va_d;
	packet.rho = rho;
	packet.flag = flag;
	packet.lambda = lambda;
	mav_array_memcpy(packet.r, r, sizeof(float)*3);
	mav_array_memcpy(packet.q, q, sizeof(float)*3);
	mav_array_memcpy(packet.c, c, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_CURRENT_PATH;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN, MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
}

/**
 * @brief Encode a mav_current_path struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_current_path C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_current_path_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_current_path_t* mav_current_path)
{
	return mavlink_msg_mav_current_path_pack(system_id, component_id, msg, mav_current_path->flag, mav_current_path->Va_d, mav_current_path->r, mav_current_path->q, mav_current_path->c, mav_current_path->rho, mav_current_path->lambda);
}

/**
 * @brief Encode a mav_current_path struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_current_path C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_current_path_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_current_path_t* mav_current_path)
{
	return mavlink_msg_mav_current_path_pack_chan(system_id, component_id, chan, msg, mav_current_path->flag, mav_current_path->Va_d, mav_current_path->r, mav_current_path->q, mav_current_path->c, mav_current_path->rho, mav_current_path->lambda);
}

/**
 * @brief Send a mav_current_path message
 * @param chan MAVLink channel to send the message
 *
 * @param flag Indicates strait line or orbital path (1 = line, 0 = orbit)
 * @param Va_d Desired airspeed (m/s)
 * @param r Vector to origin of straight line path (m)
 * @param q Unit vector, desired direction of travel for line path
 * @param c Center of orbital path (m)
 * @param rho Radius of orbital path (m)
 * @param lambda Direction of orbital path (clockwise is 1, counterclockwise is -1)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_current_path_send(mavlink_channel_t chan, uint8_t flag, float Va_d, const float *r, const float *q, const float *c, float rho, int8_t lambda)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN];
	_mav_put_float(buf, 0, Va_d);
	_mav_put_float(buf, 40, rho);
	_mav_put_uint8_t(buf, 44, flag);
	_mav_put_int8_t(buf, 45, lambda);
	_mav_put_float_array(buf, 4, r, 3);
	_mav_put_float_array(buf, 16, q, 3);
	_mav_put_float_array(buf, 28, c, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, buf, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN, MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, buf, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
#else
	mavlink_mav_current_path_t packet;
	packet.Va_d = Va_d;
	packet.rho = rho;
	packet.flag = flag;
	packet.lambda = lambda;
	mav_array_memcpy(packet.r, r, sizeof(float)*3);
	mav_array_memcpy(packet.q, q, sizeof(float)*3);
	mav_array_memcpy(packet.c, c, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, (const char *)&packet, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN, MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, (const char *)&packet, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_current_path_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t flag, float Va_d, const float *r, const float *q, const float *c, float rho, int8_t lambda)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, Va_d);
	_mav_put_float(buf, 40, rho);
	_mav_put_uint8_t(buf, 44, flag);
	_mav_put_int8_t(buf, 45, lambda);
	_mav_put_float_array(buf, 4, r, 3);
	_mav_put_float_array(buf, 16, q, 3);
	_mav_put_float_array(buf, 28, c, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, buf, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN, MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, buf, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
#else
	mavlink_mav_current_path_t *packet = (mavlink_mav_current_path_t *)msgbuf;
	packet->Va_d = Va_d;
	packet->rho = rho;
	packet->flag = flag;
	packet->lambda = lambda;
	mav_array_memcpy(packet->r, r, sizeof(float)*3);
	mav_array_memcpy(packet->q, q, sizeof(float)*3);
	mav_array_memcpy(packet->c, c, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, (const char *)packet, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN, MAVLINK_MSG_ID_MAV_CURRENT_PATH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_CURRENT_PATH, (const char *)packet, MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAV_CURRENT_PATH UNPACKING


/**
 * @brief Get field flag from mav_current_path message
 *
 * @return Indicates strait line or orbital path (1 = line, 0 = orbit)
 */
static inline uint8_t mavlink_msg_mav_current_path_get_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field Va_d from mav_current_path message
 *
 * @return Desired airspeed (m/s)
 */
static inline float mavlink_msg_mav_current_path_get_Va_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field r from mav_current_path message
 *
 * @return Vector to origin of straight line path (m)
 */
static inline uint16_t mavlink_msg_mav_current_path_get_r(const mavlink_message_t* msg, float *r)
{
	return _MAV_RETURN_float_array(msg, r, 3,  4);
}

/**
 * @brief Get field q from mav_current_path message
 *
 * @return Unit vector, desired direction of travel for line path
 */
static inline uint16_t mavlink_msg_mav_current_path_get_q(const mavlink_message_t* msg, float *q)
{
	return _MAV_RETURN_float_array(msg, q, 3,  16);
}

/**
 * @brief Get field c from mav_current_path message
 *
 * @return Center of orbital path (m)
 */
static inline uint16_t mavlink_msg_mav_current_path_get_c(const mavlink_message_t* msg, float *c)
{
	return _MAV_RETURN_float_array(msg, c, 3,  28);
}

/**
 * @brief Get field rho from mav_current_path message
 *
 * @return Radius of orbital path (m)
 */
static inline float mavlink_msg_mav_current_path_get_rho(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field lambda from mav_current_path message
 *
 * @return Direction of orbital path (clockwise is 1, counterclockwise is -1)
 */
static inline int8_t mavlink_msg_mav_current_path_get_lambda(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  45);
}

/**
 * @brief Decode a mav_current_path message into a struct
 *
 * @param msg The message to decode
 * @param mav_current_path C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_current_path_decode(const mavlink_message_t* msg, mavlink_mav_current_path_t* mav_current_path)
{
#if MAVLINK_NEED_BYTE_SWAP
	mav_current_path->Va_d = mavlink_msg_mav_current_path_get_Va_d(msg);
	mavlink_msg_mav_current_path_get_r(msg, mav_current_path->r);
	mavlink_msg_mav_current_path_get_q(msg, mav_current_path->q);
	mavlink_msg_mav_current_path_get_c(msg, mav_current_path->c);
	mav_current_path->rho = mavlink_msg_mav_current_path_get_rho(msg);
	mav_current_path->flag = mavlink_msg_mav_current_path_get_flag(msg);
	mav_current_path->lambda = mavlink_msg_mav_current_path_get_lambda(msg);
#else
	memcpy(mav_current_path, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAV_CURRENT_PATH_LEN);
#endif
}
