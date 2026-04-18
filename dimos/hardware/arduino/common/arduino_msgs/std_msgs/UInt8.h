/*
 * std_msgs/UInt8 — Arduino-compatible LCM C encode/decode.
 * Wire format: byte = 1 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_UINT8_H
#define DIMOS_ARDUINO_MSG_UINT8_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t data;
} dimos_msg__UInt8;

static inline int dimos_msg__UInt8__encoded_size(void) { return 1; }

static inline int dimos_msg__UInt8__encode(void *buf, int offset, int maxlen,
    const dimos_msg__UInt8 *p)
{
    return __byte_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__UInt8__decode(const void *buf, int offset,
    int maxlen, dimos_msg__UInt8 *p)
{
    return __byte_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ UInt8::getHash() */
static inline int64_t dimos_msg__UInt8__fingerprint(void) {
    return (int64_t)-1654270087181739132LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__UInt8__type = {
    /* name */          "std_msgs.UInt8",
    /* fingerprint */   (int64_t)-1654270087181739132LL,
    /* encoded_size */  1,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__UInt8__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
