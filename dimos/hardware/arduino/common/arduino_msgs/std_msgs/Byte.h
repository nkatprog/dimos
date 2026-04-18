/*
 * std_msgs/Byte — Arduino-compatible LCM C encode/decode.
 * Wire format: int8_t = 1 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_BYTE_H
#define DIMOS_ARDUINO_MSG_BYTE_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t data;
} dimos_msg__Byte;

static inline int dimos_msg__Byte__encoded_size(void) { return 1; }

static inline int dimos_msg__Byte__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Byte *p)
{
    return __int8_t_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Byte__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Byte *p)
{
    return __int8_t_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ Byte::getHash() */
static inline int64_t dimos_msg__Byte__fingerprint(void) {
    return (int64_t)2437365516348376085LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Byte__type = {
    /* name */          "std_msgs.Byte",
    /* fingerprint */   (int64_t)2437365516348376085LL,
    /* encoded_size */  1,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Byte__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
