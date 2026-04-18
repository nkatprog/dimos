/*
 * geometry_msgs/Inertia — Arduino-compatible LCM C encode/decode.
 * Wire format: double + Vector3(24) + double + double + double + double + double + double = 80 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_INERTIA_H
#define DIMOS_ARDUINO_MSG_INERTIA_H

#include "geometry_msgs/Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double m;
    dimos_msg__Vector3 com;
    double ixx;
    double ixy;
    double ixz;
    double iyy;
    double iyz;
    double izz;
} dimos_msg__Inertia;

static inline int dimos_msg__Inertia__encoded_size(void) { return 80; }

static inline int dimos_msg__Inertia__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Inertia *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->m, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->com);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->ixx, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->ixy, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->ixz, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->iyy, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->iyz, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->izz, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Inertia__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Inertia *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->m, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->com);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->ixx, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->ixy, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->ixz, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->iyy, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->iyz, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->izz, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Inertia::getHash() */
static inline int64_t dimos_msg__Inertia__fingerprint(void) {
    return (int64_t)-2715402529235359748LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Inertia__type = {
    /* name */          "geometry_msgs.Inertia",
    /* fingerprint */   (int64_t)-2715402529235359748LL,
    /* encoded_size */  80,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Inertia__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
