/*
 * sensor_msgs/JoyFeedback — Arduino-compatible LCM C encode/decode.
 * Wire format: byte + byte + float = 6 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_JOYFEEDBACK_H
#define DIMOS_ARDUINO_MSG_JOYFEEDBACK_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t type;
    uint8_t id;
    float intensity;
} dimos_msg__JoyFeedback;

static inline int dimos_msg__JoyFeedback__encoded_size(void) { return 6; }

static inline int dimos_msg__JoyFeedback__encode(void *buf, int offset, int maxlen,
    const dimos_msg__JoyFeedback *p)
{
    int pos = 0, thislen;
    thislen = __byte_encode_array(buf, offset + pos, maxlen - pos, &p->type, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __byte_encode_array(buf, offset + pos, maxlen - pos, &p->id, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->intensity, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__JoyFeedback__decode(const void *buf, int offset,
    int maxlen, dimos_msg__JoyFeedback *p)
{
    int pos = 0, thislen;
    thislen = __byte_decode_array(buf, offset + pos, maxlen - pos, &p->type, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __byte_decode_array(buf, offset + pos, maxlen - pos, &p->id, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->intensity, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ JoyFeedback::getHash() */
static inline int64_t dimos_msg__JoyFeedback__fingerprint(void) {
    return (int64_t)5230447590260245546LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__JoyFeedback__type = {
    /* name */          "sensor_msgs.JoyFeedback",
    /* fingerprint */   (int64_t)5230447590260245546LL,
    /* encoded_size */  6,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__JoyFeedback__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
