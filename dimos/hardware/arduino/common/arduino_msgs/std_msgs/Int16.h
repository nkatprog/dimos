/*
 * std_msgs/Int16 — Arduino-compatible LCM C encode/decode.
 * Wire format: int16_t = 2 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_INT16_H
#define DIMOS_ARDUINO_MSG_INT16_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t data;
} dimos_msg__Int16;

static inline int dimos_msg__Int16__encoded_size(void) { return 2; }

static inline int dimos_msg__Int16__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Int16 *p)
{
    return __int16_t_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Int16__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Int16 *p)
{
    return __int16_t_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ Int16::getHash() */
static inline int64_t dimos_msg__Int16__fingerprint(void) {
    return (int64_t)3223726259432391230LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Int16__type = {
    /* name */          "std_msgs.Int16",
    /* fingerprint */   (int64_t)3223726259432391230LL,
    /* encoded_size */  2,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Int16__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
