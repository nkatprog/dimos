/*
 * foxglove_msgs/SpherePrimitive — Arduino-compatible LCM C encode/decode.
 * Wire format: Pose(56) + Vector3(24) + Color(32) = 112 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_SPHEREPRIMITIVE_H
#define DIMOS_ARDUINO_MSG_SPHEREPRIMITIVE_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "foxglove_msgs/Color.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Pose pose;
    dimos_msg__Vector3 size;
    dimos_msg__Color color;
} dimos_msg__SpherePrimitive;

static inline int dimos_msg__SpherePrimitive__encoded_size(void) { return 112; }

static inline int dimos_msg__SpherePrimitive__encode(void *buf, int offset, int maxlen,
    const dimos_msg__SpherePrimitive *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__encode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->size);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__encode(buf, offset + pos, maxlen - pos, &p->color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__SpherePrimitive__decode(const void *buf, int offset,
    int maxlen, dimos_msg__SpherePrimitive *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__decode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->size);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__decode(buf, offset + pos, maxlen - pos, &p->color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ SpherePrimitive::getHash() */
static inline int64_t dimos_msg__SpherePrimitive__fingerprint(void) {
    return (int64_t)-2630344587164280003LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__SpherePrimitive__type = {
    /* name */          "foxglove_msgs.SpherePrimitive",
    /* fingerprint */   (int64_t)-2630344587164280003LL,
    /* encoded_size */  112,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__SpherePrimitive__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
