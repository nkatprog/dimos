/*
 * geometry_msgs/Quaternion — Arduino-compatible LCM C encode/decode.
 * Wire format: double + double + double + double = 32 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_QUATERNION_H
#define DIMOS_ARDUINO_MSG_QUATERNION_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double x;
    double y;
    double z;
    double w;
} dimos_msg__Quaternion;

static inline int dimos_msg__Quaternion__encoded_size(void) { return 32; }

static inline int dimos_msg__Quaternion__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Quaternion *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->w, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Quaternion__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Quaternion *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->w, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Quaternion::getHash() */
static inline int64_t dimos_msg__Quaternion__fingerprint(void) {
    return (int64_t)3907960351325948459LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Quaternion__type = {
    /* name */          "geometry_msgs.Quaternion",
    /* fingerprint */   (int64_t)3907960351325948459LL,
    /* encoded_size */  32,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Quaternion__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
