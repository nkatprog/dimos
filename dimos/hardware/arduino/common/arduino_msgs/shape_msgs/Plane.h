/*
 * shape_msgs/Plane — Arduino-compatible LCM C encode/decode.
 * Wire format: 4x double(32) = 32 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_PLANE_H
#define DIMOS_ARDUINO_MSG_PLANE_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double coef[4];
} dimos_msg__Plane;

static inline int dimos_msg__Plane__encoded_size(void) { return 32; }

static inline int dimos_msg__Plane__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Plane *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, p->coef, 4);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Plane__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Plane *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, p->coef, 4);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Plane::getHash() */
static inline int64_t dimos_msg__Plane__fingerprint(void) {
    return (int64_t)-264984081144330268LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Plane__type = {
    /* name */          "shape_msgs.Plane",
    /* fingerprint */   (int64_t)-264984081144330268LL,
    /* encoded_size */  32,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Plane__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
