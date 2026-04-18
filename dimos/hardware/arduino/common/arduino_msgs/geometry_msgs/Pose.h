/*
 * geometry_msgs/Pose — Arduino-compatible LCM C encode/decode.
 * Wire format: Point(24) + Quaternion(32) = 56 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_POSE_H
#define DIMOS_ARDUINO_MSG_POSE_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Point position;
    dimos_msg__Quaternion orientation;
} dimos_msg__Pose;

static inline int dimos_msg__Pose__encoded_size(void) { return 56; }

static inline int dimos_msg__Pose__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Pose *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Point__encode(buf, offset + pos, maxlen - pos, &p->position);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Quaternion__encode(buf, offset + pos, maxlen - pos, &p->orientation);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Pose__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Pose *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Point__decode(buf, offset + pos, maxlen - pos, &p->position);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Quaternion__decode(buf, offset + pos, maxlen - pos, &p->orientation);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Pose::getHash() */
static inline int64_t dimos_msg__Pose__fingerprint(void) {
    return (int64_t)2618338156007750518LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Pose__type = {
    /* name */          "geometry_msgs.Pose",
    /* fingerprint */   (int64_t)2618338156007750518LL,
    /* encoded_size */  56,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Pose__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
