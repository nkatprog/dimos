/*
 * vision_msgs/Point2D — Arduino-compatible LCM C encode/decode.
 * Wire format: double + double = 16 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_POINT2D_H
#define DIMOS_ARDUINO_MSG_POINT2D_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double x;
    double y;
} dimos_msg__Point2D;

static inline int dimos_msg__Point2D__encoded_size(void) { return 16; }

static inline int dimos_msg__Point2D__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Point2D *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Point2D__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Point2D *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Point2D::getHash() */
static inline int64_t dimos_msg__Point2D__fingerprint(void) {
    return (int64_t)-6579017587979939573LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Point2D__type = {
    /* name */          "vision_msgs.Point2D",
    /* fingerprint */   (int64_t)-6579017587979939573LL,
    /* encoded_size */  16,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Point2D__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
