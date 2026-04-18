/*
 * geometry_msgs/Transform — Arduino-compatible LCM C encode/decode.
 * Wire format: Vector3(24) + Quaternion(32) = 56 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_TRANSFORM_H
#define DIMOS_ARDUINO_MSG_TRANSFORM_H

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Vector3 translation;
    dimos_msg__Quaternion rotation;
} dimos_msg__Transform;

static inline int dimos_msg__Transform__encoded_size(void) { return 56; }

static inline int dimos_msg__Transform__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Transform *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->translation);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Quaternion__encode(buf, offset + pos, maxlen - pos, &p->rotation);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Transform__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Transform *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->translation);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Quaternion__decode(buf, offset + pos, maxlen - pos, &p->rotation);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Transform::getHash() */
static inline int64_t dimos_msg__Transform__fingerprint(void) {
    return (int64_t)-1270028124645539951LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Transform__type = {
    /* name */          "geometry_msgs.Transform",
    /* fingerprint */   (int64_t)-1270028124645539951LL,
    /* encoded_size */  56,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Transform__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
