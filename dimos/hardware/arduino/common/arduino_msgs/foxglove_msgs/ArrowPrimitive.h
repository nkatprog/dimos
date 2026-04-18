/*
 * foxglove_msgs/ArrowPrimitive — Arduino-compatible LCM C encode/decode.
 * Wire format: Pose(56) + double + double + double + double + Color(32) = 120 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_ARROWPRIMITIVE_H
#define DIMOS_ARDUINO_MSG_ARROWPRIMITIVE_H

#include "geometry_msgs/Pose.h"
#include "foxglove_msgs/Color.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Pose pose;
    double shaft_length;
    double shaft_diameter;
    double head_length;
    double head_diameter;
    dimos_msg__Color color;
} dimos_msg__ArrowPrimitive;

static inline int dimos_msg__ArrowPrimitive__encoded_size(void) { return 120; }

static inline int dimos_msg__ArrowPrimitive__encode(void *buf, int offset, int maxlen,
    const dimos_msg__ArrowPrimitive *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__encode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->shaft_length, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->shaft_diameter, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->head_length, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->head_diameter, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__encode(buf, offset + pos, maxlen - pos, &p->color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__ArrowPrimitive__decode(const void *buf, int offset,
    int maxlen, dimos_msg__ArrowPrimitive *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__decode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->shaft_length, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->shaft_diameter, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->head_length, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->head_diameter, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__decode(buf, offset + pos, maxlen - pos, &p->color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ ArrowPrimitive::getHash() */
static inline int64_t dimos_msg__ArrowPrimitive__fingerprint(void) {
    return (int64_t)1343453716438460264LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__ArrowPrimitive__type = {
    /* name */          "foxglove_msgs.ArrowPrimitive",
    /* fingerprint */   (int64_t)1343453716438460264LL,
    /* encoded_size */  120,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__ArrowPrimitive__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
