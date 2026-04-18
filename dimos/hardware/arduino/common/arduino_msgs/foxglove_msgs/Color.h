/*
 * foxglove_msgs/Color — Arduino-compatible LCM C encode/decode.
 * Wire format: double + double + double + double = 32 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_COLOR_H
#define DIMOS_ARDUINO_MSG_COLOR_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double r;
    double g;
    double b;
    double a;
} dimos_msg__Color;

static inline int dimos_msg__Color__encoded_size(void) { return 32; }

static inline int dimos_msg__Color__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Color *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->r, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->g, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->b, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->a, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Color__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Color *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->r, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->g, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->b, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->a, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Color::getHash() */
static inline int64_t dimos_msg__Color__fingerprint(void) {
    return (int64_t)3675619187199805571LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Color__type = {
    /* name */          "foxglove_msgs.Color",
    /* fingerprint */   (int64_t)3675619187199805571LL,
    /* encoded_size */  32,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Color__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
