/*
 * foxglove_msgs/CylinderPrimitive — Arduino-compatible LCM C encode/decode.
 * Wire format: Pose(56) + Vector3(24) + double + double + Color(32) = 128 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_CYLINDERPRIMITIVE_H
#define DIMOS_ARDUINO_MSG_CYLINDERPRIMITIVE_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "foxglove_msgs/Color.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Pose pose;
    dimos_msg__Vector3 size;
    double bottom_scale;
    double top_scale;
    dimos_msg__Color color;
} dimos_msg__CylinderPrimitive;

static inline int dimos_msg__CylinderPrimitive__encoded_size(void) { return 128; }

static inline int dimos_msg__CylinderPrimitive__encode(void *buf, int offset, int maxlen,
    const dimos_msg__CylinderPrimitive *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__encode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->size);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->bottom_scale, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->top_scale, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__encode(buf, offset + pos, maxlen - pos, &p->color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__CylinderPrimitive__decode(const void *buf, int offset,
    int maxlen, dimos_msg__CylinderPrimitive *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__decode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->size);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->bottom_scale, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->top_scale, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__decode(buf, offset + pos, maxlen - pos, &p->color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ CylinderPrimitive::getHash() */
static inline int64_t dimos_msg__CylinderPrimitive__fingerprint(void) {
    return (int64_t)6192457399107281287LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__CylinderPrimitive__type = {
    /* name */          "foxglove_msgs.CylinderPrimitive",
    /* fingerprint */   (int64_t)6192457399107281287LL,
    /* encoded_size */  128,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__CylinderPrimitive__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
