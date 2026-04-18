/*
 * std_msgs/ColorRGBA — Arduino-compatible LCM C encode/decode.
 * Wire format: float + float + float + float = 16 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_COLORRGBA_H
#define DIMOS_ARDUINO_MSG_COLORRGBA_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float r;
    float g;
    float b;
    float a;
} dimos_msg__ColorRGBA;

static inline int dimos_msg__ColorRGBA__encoded_size(void) { return 16; }

static inline int dimos_msg__ColorRGBA__encode(void *buf, int offset, int maxlen,
    const dimos_msg__ColorRGBA *p)
{
    int pos = 0, thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->r, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->g, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->b, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->a, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__ColorRGBA__decode(const void *buf, int offset,
    int maxlen, dimos_msg__ColorRGBA *p)
{
    int pos = 0, thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->r, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->g, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->b, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->a, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ ColorRGBA::getHash() */
static inline int64_t dimos_msg__ColorRGBA__fingerprint(void) {
    return (int64_t)7940549966018511412LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__ColorRGBA__type = {
    /* name */          "std_msgs.ColorRGBA",
    /* fingerprint */   (int64_t)7940549966018511412LL,
    /* encoded_size */  16,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__ColorRGBA__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
