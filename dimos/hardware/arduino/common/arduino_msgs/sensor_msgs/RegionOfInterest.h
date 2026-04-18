/*
 * sensor_msgs/RegionOfInterest — Arduino-compatible LCM C encode/decode.
 * Wire format: int32_t + int32_t + int32_t + int32_t + boolean = 17 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_REGIONOFINTEREST_H
#define DIMOS_ARDUINO_MSG_REGIONOFINTEREST_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t x_offset;
    int32_t y_offset;
    int32_t height;
    int32_t width;
    int8_t do_rectify;
} dimos_msg__RegionOfInterest;

static inline int dimos_msg__RegionOfInterest__encoded_size(void) { return 17; }

static inline int dimos_msg__RegionOfInterest__encode(void *buf, int offset, int maxlen,
    const dimos_msg__RegionOfInterest *p)
{
    int pos = 0, thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->x_offset, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->y_offset, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->height, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->width, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &p->do_rectify, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__RegionOfInterest__decode(const void *buf, int offset,
    int maxlen, dimos_msg__RegionOfInterest *p)
{
    int pos = 0, thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->x_offset, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->y_offset, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->height, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->width, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &p->do_rectify, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ RegionOfInterest::getHash() */
static inline int64_t dimos_msg__RegionOfInterest__fingerprint(void) {
    return (int64_t)8292548831819628060LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__RegionOfInterest__type = {
    /* name */          "sensor_msgs.RegionOfInterest",
    /* fingerprint */   (int64_t)8292548831819628060LL,
    /* encoded_size */  17,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__RegionOfInterest__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
