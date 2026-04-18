/*
 * geometry_msgs/Wrench — Arduino-compatible LCM C encode/decode.
 * Wire format: Vector3(24) + Vector3(24) = 48 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_WRENCH_H
#define DIMOS_ARDUINO_MSG_WRENCH_H

#include "geometry_msgs/Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Vector3 force;
    dimos_msg__Vector3 torque;
} dimos_msg__Wrench;

static inline int dimos_msg__Wrench__encoded_size(void) { return 48; }

static inline int dimos_msg__Wrench__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Wrench *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->force);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->torque);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Wrench__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Wrench *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->force);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->torque);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Wrench::getHash() */
static inline int64_t dimos_msg__Wrench__fingerprint(void) {
    return (int64_t)-3676337769631967292LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Wrench__type = {
    /* name */          "geometry_msgs.Wrench",
    /* fingerprint */   (int64_t)-3676337769631967292LL,
    /* encoded_size */  48,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Wrench__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
