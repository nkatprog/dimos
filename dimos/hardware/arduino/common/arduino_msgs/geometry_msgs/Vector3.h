/*
 * geometry_msgs/Vector3 — Arduino-compatible LCM C encode/decode.
 * Wire format: double + double + double = 24 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_VECTOR3_H
#define DIMOS_ARDUINO_MSG_VECTOR3_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double x;
    double y;
    double z;
} dimos_msg__Vector3;

static inline int dimos_msg__Vector3__encoded_size(void) { return 24; }

static inline int dimos_msg__Vector3__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Vector3 *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Vector3__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Vector3 *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Vector3::getHash() */
static inline int64_t dimos_msg__Vector3__fingerprint(void) {
    return (int64_t)-5873151609983426274LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Vector3__type = {
    /* name */          "geometry_msgs.Vector3",
    /* fingerprint */   (int64_t)-5873151609983426274LL,
    /* encoded_size */  24,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Vector3__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
