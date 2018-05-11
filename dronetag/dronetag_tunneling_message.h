#pragma once
// MESSAGE DRONE TAG TUNNELING PACKING
#ifndef dronetag_tunneling_message_h
#define dronetag_tunneling_message_h
//#include "../mavlink_types.h"

#define MAVLINK_MSG_ID_DRONETAG_TUNNEL 240 // for the moment

typedef struct /*MAVLINK_PACKED*/__attribute__((__packed__)) __mavlink_dronetag_tunneling_t
{
    uint16_t target_system; /*< */
    uint16_t target_component; /*< */
    uint8_t  packet_to_wait;  /*< number of packets to wait to assemble a complete message, 0 by default */
    uint8_t  payload[250];   /*< can not exceed 255 maximum */
} mavlink_dronetag_tunneling_t;

#define MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN 255  /*< can not exceed 255 maximum */
#define MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN 17 //minimum payload length = 21;
#define MAVLINK_MSG_ID_240_LEN 255
#define MAVLINK_MSG_ID_240_MIN_LEN 17

#define MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC 197
#define MAVLINK_MSG_ID_240_CRC 197

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONETAG_TUNNEL { \
240, \
"DRONETAG_TUNNEL", \
2, \
{  { "target_system", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_dronetag_tunneling_t, target_system) }, \
{ "target_component", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_dronetag_tunneling_t, target_component) }, \
} \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONETAG_TUNNEL { \
"DRONETAG_TUNNEL", \
2, \
{  { "target_system", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_dronetag_tunneling_t, target_system) }, \
{ "target_component", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_dronetag_tunneling_t, target_component) }, \
} \
}
#endif

/**
 * @brief Pack a drone tag tunnel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system ID of destinatin system defined by dronetag radio
 * @param target_component ID of destination component defined by dronetag radio (e.g. 200 for IMU)
 * @param payload message to be tunneled as payload
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dronetag_tunneling_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                                     uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN];
    //_mav_put_uint8_t(buf, 2, target_network);
    _mav_put_uint8_t(buf, 3, target_system);
    _mav_put_uint8_t(buf, 4, target_component);
    _mav_put_uint8_t_array(buf, 5, payload, 267);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN);
#else
    mavlink_dronetag_tunneling_t packet;
    //packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*267);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN);
#endif
    
    msg->msgid = MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC);
}

/**
 * @brief Pack a drone tag tunnel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system ID of destinatin system defined by dronetag radio
 * @param target_component ID of destination component defined by dronetag radio (e.g. 200 for IMU)
 * @param payload message to be tunneled as payload
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dronetag_tunneling_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                                                          mavlink_message_t* msg,
                                                          uint8_t target_network,uint8_t target_system,uint8_t target_component,const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN];
    //_mav_put_uint8_t(buf, 2, target_network);
    _mav_put_uint8_t(buf, 3, target_system);
    _mav_put_uint8_t(buf, 4, target_component);
    _mav_put_uint8_t_array(buf, 5, payload, 249);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN);
#else
    mavlink_dronetag_tunneling_t packet;
    //packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*249);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN);
#endif
    
    msg->msgid = MAVLINK_MSG_ID_DRONETAG_TUNNEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC);
}

/**
 * @brief Encode a dronetag tunnel message struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tunnel_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dronetag_tunneling_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dronetag_tunneling_t* tunnel_message)
{
    return mavlink_msg_dronetag_tunneling_pack(system_id, component_id, msg, /*tunnel_message->target_network*/0, tunnel_message->target_system, tunnel_message->target_component, tunnel_message->payload);
}

/**
 * @brief Encode a dronetag tunneling struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v2_extension C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dronetag_tunneling_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dronetag_tunneling_t* tunnel_message)
{
    return mavlink_msg_dronetag_tunneling_pack_chan(system_id, component_id, chan, msg, /*dronetag_tunneling->target_network*/0, tunnel_message->target_system, tunnel_message->target_component, tunnel_message->payload);
}


/**
 * @brief Send a dronetag tunneling message
 * @param chan MAVLink channel to send the message
 *
 * @param target_network Network ID (0 for broadcast)
 * @param target_system System ID (0 for broadcast)
 * @param target_component Component ID (0 for broadcast)
 * @param
 * @param payload Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dronetag_tunneling_send(mavlink_channel_t chan, uint8_t target_network, uint8_t target_system, uint8_t target_component, uint8_t *payload, int payload_len)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN];
    //_mav_put_uint8_t(buf, 2, target_network);
    _mav_put_uint16_t(buf, 0, target_system);
    _mav_put_uint16_t(buf, 2, target_component);
    //_mav_put_uint8_t(buf, 4, target_component);
    //_mav_put_uint8_t_array(buf, 5, payload_msg, 249);
    _mav_finalize_tunneling_message_chan_send(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, buf, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, payload_len, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload);
#else
    mavlink_dronetag_tunneling_t packet;
    //packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    //mav_array_memcpy(packet.payload, payload_msg, payload_msg->len);
    _mav_finalize_tunneling_message_chan_send(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, (char*)&packet, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, payload_len, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload);
#endif
}

/**
 * @brief Tunnelize and serialize received wire steam in wire format to a buffer for sending
 * @param chan MAVLink channel to send the message
 * @param payload The payload bytes to send
 * @param payload_len payload byte length
 * Note: use this function to save multiple copies between internal buffers;
 */
static inline uint16_t mavlink_msg_dronetag_tunneling_send_payload_wire_to_buffer(uint8_t* ext_buf, mavlink_channel_t chan, uint8_t target_network, uint16_t target_system, uint16_t target_component, uint8_t *payload, int payload_len)
{
    uint8_t tunnel_header[5];
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    _mav_put_uint16_t(tunnel_header, 0, target_system);
    _mav_put_uint16_t(tunnel_header, 2, target_component);
    return _mav_finalize_tunneling_message_chan_to_buffer(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, tunnel_header, (char*)ext_buf, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, payload_len, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload);
#else
    memcpy(tunnel_header, (uint8_t*)&target_system, 2);
    memcpy(tunnel_header+2, (uint8_t*)&target_component+2, 2);
    
    return _mav_finalize_tunneling_message_chan_to_buffer(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, tunnel_header, (char*)ext_buf, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, payload_len, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload);
#endif
}

/**
 * @brief Send received byte steam in wire format in a drone tag tunneling message
 * @param chan MAVLink channel to send the message
 * @param payload The payload bytes to send
 * @param payload_len payload byte length
 *
 */
static inline void mavlink_msg_dronetag_tunneling_send_payload_wire(mavlink_channel_t chan, uint8_t target_network, uint8_t target_system, uint8_t target_component, uint8_t *payload, int payload_len)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dronetag_tunneling_send(chan, target_network, target_system, target_component, payload, payload_len);
#else
    mavlink_dronetag_tunneling_t packet;
    //packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    
    _mav_finalize_tunneling_message_chan_send(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, (char *)&packet, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, payload_len, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload);
#endif
}

/**
 * @brief Send a drone tag tunneling message
 * @param chan MAVLink channel to send the message
 * @param payload_msg The MAVLink message struct as payload to serialize
 * @param payload_msg_size size of payload message struct
 * Note the first 2 bytes of checksum in mavlink message struct needs to be ignored
 */
static inline void mavlink_msg_dronetag_tunneling_send_payload_msg(mavlink_channel_t chan, uint8_t target_network, uint8_t target_system, uint8_t target_component, mavlink_message_t *payload_msg, int payload_msg_size)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dronetag_tunneling_send(chan, target_network, target_system, target_component, (uint8_t*)(payload_msg+2), payload_msg_size-2);
#else
    mavlink_dronetag_tunneling_t packet;
    //packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_tunneling_message_chan_send(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, (char *)&packet, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, payload_msg_size-2, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, (uint8_t*)(payload_msg+2));
#endif
}

#if MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
 This varient of _send() can be used to save stack space by re-using
 memory from the receive buffer.  The caller provides a
 mavlink_message_t which is the size of a full mavlink message. This
 is usually the receive buffer for the channel, and allows a reply to an
 incoming message with minimum stack space usage.
 */
/*
static inline void mavlink_msg_dronetag_tunneling_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    //_mav_put_uint8_t(buf, 2, target_network);
    _mav_put_uint8_t(buf, 3, target_system);
    _mav_put_uint8_t(buf, 4, target_component);
    //_mav_put_uint8_t_array(buf, 5, payload, 249);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, buf, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload_msg);
#else
    mavlink_dronetag_tunnel_t *packet = (mavlink_dronetag_tunnel_t *)msgbuf;
    //packet->target_network = target_network;
    packet->target_system = target_system;
    packet->target_component = target_component;
    //mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*249);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONETAG_TUNNEL, (char *)packet, MAVLINK_MSG_ID_DRONETAG_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN, MAVLINK_MSG_ID_DRONETAG_TUNNEL_CRC, payload_msg);
#endif
}
 */
#endif

#endif //convenience functions


// MESSAGE DRONETAG_TUNNEL UNPACKING


/**
 * @brief Get field target_network from dronetag tunneling message
 *
 * @return Network ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_dronetag_tunnel_get_target_network(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_system from dronetag tunneling message
 *
 * @return System ID (0 for broadcast)
 */
static inline uint16_t mavlink_msg_dronetag_tunnel_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field target_component from dronetag tunneling message
 *
 * @return Component ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_dronetag_tunnel_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field message_type from dronetag tunneling message
 *
 * @return number of packets to wait;
 */
static inline uint16_t mavlink_msg_dronetag_tunnel_get_message_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4); //jump over the target system id and comp id;
}

/**
 * @brief Get field payload from dronetag tunneling message
 *
 * @return Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
static inline uint16_t mavlink_msg_dronetag_tunnel_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, msg->len,  5);
}

/**
 * @brief Get pointer to payload in dronetag tunneling message
 *
 * @return Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
static inline uint16_t mavlink_msg_dronetag_tunnel_get_chan_payload_non_copy(mavlink_channel_t chan, const uint8_t* msg_buf, uint8_t **payload)
{
    uint8_t wire_offset;
    if (1 == mavlink_get_proto_version(chan)) {
        wire_offset = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
    }
    else {
        wire_offset = MAVLINK_CORE_HEADER_LEN + 1;
    }

    *payload = (uint8_t*)(msg_buf + wire_offset + 5); //jump over the target system id and comp id;
    return *(msg_buf + 1) - 5;
}


/**
 * @brief Decode a dronetag tunneled message into a struct
 *
 * @param msg The message to decode
 * @param v2_extension C-struct to decode the message contents into
 */
static inline void mavlink_msg_dronetag_tunnel_decode(const mavlink_message_t* msg, mavlink_dronetag_tunneling_t* tunnel_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tunnel_message->message_type = mavlink_msg_dronetag_tunnel_get_message_type(msg);
    //tunnel_message->target_network = mavlink_msg_dronetag_tunnel_get_target_network(msg);
    tunnel_message->target_system = mavlink_msg_dronetag_tunnel_get_target_system(msg);
    tunnel_message->target_component = mavlink_msg_dronetag_tunnel_get_target_component(msg);
    mavlink_msg_dronetag_tunnel_get_payload(msg, tunnel_message->payload);
#else
    uint8_t len = msg->len < MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN? msg->len : MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN;
    memset(tunnel_message, 0, MAVLINK_MSG_ID_DRONETAG_TUNNEL_LEN);
    memcpy(tunnel_message, _MAV_PAYLOAD(msg), len);
#endif
}

#endif //dronetag_tunneling_message_h

