/*
 * geometry_msgs/Twist — Arduino-compatible LCM C encode/decode.
 * Wire format: Vector3(24) + Vector3(24) = 48 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_TWIST_H
#define DIMOS_ARDUINO_MSG_TWIST_H

#include "geometry_msgs/Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Vector3 linear;
    dimos_msg__Vector3 angular;
} dimos_msg__Twist;

static inline int dimos_msg__Twist__encoded_size(void) { return 48; }

static inline int dimos_msg__Twist__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Twist *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->linear);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->angular);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Twist__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Twist *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->linear);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->angular);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Twist::getHash() */
static inline int64_t dimos_msg__Twist__fingerprint(void) {
    return (int64_t)3349560846311743527LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Twist__type = {
    /* name */          "geometry_msgs.Twist",
    /* fingerprint */   (int64_t)3349560846311743527LL,
    /* encoded_size */  48,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Twist__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
