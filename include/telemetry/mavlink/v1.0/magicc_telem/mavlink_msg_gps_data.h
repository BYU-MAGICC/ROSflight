// MESSAGE GPS_DATA PACKING

#define MAVLINK_MSG_ID_GPS_DATA 204

typedef struct __mavlink_gps_data_t
{
 float latitude; /*< Latitude (deg)*/
 float longitude; /*< Longitude (deg)*/
 float altitude; /*< Altitude (m)*/
 float speed; /*< Speed (m/s)*/
 float ground_course; /*< Ground course (rad)*/
 float covariance; /*< Covariance (m)*/
 uint16_t NumSat; /*< Number of connected satellites*/
 uint8_t fix; /*< Status of gps fix (1 = true, 0 = false)*/
} mavlink_gps_data_t;

#define MAVLINK_MSG_ID_GPS_DATA_LEN 27
#define MAVLINK_MSG_ID_204_LEN 27

#define MAVLINK_MSG_ID_GPS_DATA_CRC 252
#define MAVLINK_MSG_ID_204_CRC 252



#define MAVLINK_MESSAGE_INFO_GPS_DATA { \
	"GPS_DATA", \
	8, \
	{  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps_data_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gps_data_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gps_data_t, altitude) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gps_data_t, speed) }, \
         { "ground_course", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gps_data_t, ground_course) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gps_data_t, covariance) }, \
         { "NumSat", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_gps_data_t, NumSat) }, \
         { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_gps_data_t, fix) }, \
         } \
}


/**
 * @brief Pack a gps_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fix Status of gps fix (1 = true, 0 = false)
 * @param NumSat Number of connected satellites
 * @param latitude Latitude (deg)
 * @param longitude Longitude (deg)
 * @param altitude Altitude (m)
 * @param speed Speed (m/s)
 * @param ground_course Ground course (rad)
 * @param covariance Covariance (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t fix, uint16_t NumSat, float latitude, float longitude, float altitude, float speed, float ground_course, float covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
	_mav_put_float(buf, 0, latitude);
	_mav_put_float(buf, 4, longitude);
	_mav_put_float(buf, 8, altitude);
	_mav_put_float(buf, 12, speed);
	_mav_put_float(buf, 16, ground_course);
	_mav_put_float(buf, 20, covariance);
	_mav_put_uint16_t(buf, 24, NumSat);
	_mav_put_uint8_t(buf, 26, fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#else
	mavlink_gps_data_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.speed = speed;
	packet.ground_course = ground_course;
	packet.covariance = covariance;
	packet.NumSat = NumSat;
	packet.fix = fix;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
}

/**
 * @brief Pack a gps_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fix Status of gps fix (1 = true, 0 = false)
 * @param NumSat Number of connected satellites
 * @param latitude Latitude (deg)
 * @param longitude Longitude (deg)
 * @param altitude Altitude (m)
 * @param speed Speed (m/s)
 * @param ground_course Ground course (rad)
 * @param covariance Covariance (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t fix,uint16_t NumSat,float latitude,float longitude,float altitude,float speed,float ground_course,float covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
	_mav_put_float(buf, 0, latitude);
	_mav_put_float(buf, 4, longitude);
	_mav_put_float(buf, 8, altitude);
	_mav_put_float(buf, 12, speed);
	_mav_put_float(buf, 16, ground_course);
	_mav_put_float(buf, 20, covariance);
	_mav_put_uint16_t(buf, 24, NumSat);
	_mav_put_uint8_t(buf, 26, fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#else
	mavlink_gps_data_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.speed = speed;
	packet.ground_course = ground_course;
	packet.covariance = covariance;
	packet.NumSat = NumSat;
	packet.fix = fix;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
}

/**
 * @brief Encode a gps_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_data_t* gps_data)
{
	return mavlink_msg_gps_data_pack(system_id, component_id, msg, gps_data->fix, gps_data->NumSat, gps_data->latitude, gps_data->longitude, gps_data->altitude, gps_data->speed, gps_data->ground_course, gps_data->covariance);
}

/**
 * @brief Encode a gps_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_data_t* gps_data)
{
	return mavlink_msg_gps_data_pack_chan(system_id, component_id, chan, msg, gps_data->fix, gps_data->NumSat, gps_data->latitude, gps_data->longitude, gps_data->altitude, gps_data->speed, gps_data->ground_course, gps_data->covariance);
}

/**
 * @brief Send a gps_data message
 * @param chan MAVLink channel to send the message
 *
 * @param fix Status of gps fix (1 = true, 0 = false)
 * @param NumSat Number of connected satellites
 * @param latitude Latitude (deg)
 * @param longitude Longitude (deg)
 * @param altitude Altitude (m)
 * @param speed Speed (m/s)
 * @param ground_course Ground course (rad)
 * @param covariance Covariance (m)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_data_send(mavlink_channel_t chan, uint8_t fix, uint16_t NumSat, float latitude, float longitude, float altitude, float speed, float ground_course, float covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
	_mav_put_float(buf, 0, latitude);
	_mav_put_float(buf, 4, longitude);
	_mav_put_float(buf, 8, altitude);
	_mav_put_float(buf, 12, speed);
	_mav_put_float(buf, 16, ground_course);
	_mav_put_float(buf, 20, covariance);
	_mav_put_uint16_t(buf, 24, NumSat);
	_mav_put_uint8_t(buf, 26, fix);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, buf, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
#else
	mavlink_gps_data_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.speed = speed;
	packet.ground_course = ground_course;
	packet.covariance = covariance;
	packet.NumSat = NumSat;
	packet.fix = fix;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)&packet, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)&packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GPS_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t fix, uint16_t NumSat, float latitude, float longitude, float altitude, float speed, float ground_course, float covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, latitude);
	_mav_put_float(buf, 4, longitude);
	_mav_put_float(buf, 8, altitude);
	_mav_put_float(buf, 12, speed);
	_mav_put_float(buf, 16, ground_course);
	_mav_put_float(buf, 20, covariance);
	_mav_put_uint16_t(buf, 24, NumSat);
	_mav_put_uint8_t(buf, 26, fix);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, buf, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
#else
	mavlink_gps_data_t *packet = (mavlink_gps_data_t *)msgbuf;
	packet->latitude = latitude;
	packet->longitude = longitude;
	packet->altitude = altitude;
	packet->speed = speed;
	packet->ground_course = ground_course;
	packet->covariance = covariance;
	packet->NumSat = NumSat;
	packet->fix = fix;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)packet, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GPS_DATA UNPACKING


/**
 * @brief Get field fix from gps_data message
 *
 * @return Status of gps fix (1 = true, 0 = false)
 */
static inline uint8_t mavlink_msg_gps_data_get_fix(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field NumSat from gps_data message
 *
 * @return Number of connected satellites
 */
static inline uint16_t mavlink_msg_gps_data_get_NumSat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field latitude from gps_data message
 *
 * @return Latitude (deg)
 */
static inline float mavlink_msg_gps_data_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from gps_data message
 *
 * @return Longitude (deg)
 */
static inline float mavlink_msg_gps_data_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field altitude from gps_data message
 *
 * @return Altitude (m)
 */
static inline float mavlink_msg_gps_data_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field speed from gps_data message
 *
 * @return Speed (m/s)
 */
static inline float mavlink_msg_gps_data_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ground_course from gps_data message
 *
 * @return Ground course (rad)
 */
static inline float mavlink_msg_gps_data_get_ground_course(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field covariance from gps_data message
 *
 * @return Covariance (m)
 */
static inline float mavlink_msg_gps_data_get_covariance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a gps_data message into a struct
 *
 * @param msg The message to decode
 * @param gps_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_data_decode(const mavlink_message_t* msg, mavlink_gps_data_t* gps_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_data->latitude = mavlink_msg_gps_data_get_latitude(msg);
	gps_data->longitude = mavlink_msg_gps_data_get_longitude(msg);
	gps_data->altitude = mavlink_msg_gps_data_get_altitude(msg);
	gps_data->speed = mavlink_msg_gps_data_get_speed(msg);
	gps_data->ground_course = mavlink_msg_gps_data_get_ground_course(msg);
	gps_data->covariance = mavlink_msg_gps_data_get_covariance(msg);
	gps_data->NumSat = mavlink_msg_gps_data_get_NumSat(msg);
	gps_data->fix = mavlink_msg_gps_data_get_fix(msg);
#else
	memcpy(gps_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
}
