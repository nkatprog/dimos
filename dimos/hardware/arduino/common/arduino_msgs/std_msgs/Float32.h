/*
 * std_msgs/Float32 — Arduino-compatible LCM C encode/decode.
 * Wire format: float = 4 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_FLOAT32_H
#define DIMOS_ARDUINO_MSG_FLOAT32_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float data;
} dimos_msg__Float32;

static inline int dimos_msg__Float32__encoded_size(void) { return 4; }

static inline int dimos_msg__Float32__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Float32 *p)
{
    return __float_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Float32__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Float32 *p)
{
    return __float_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ Float32::getHash() */
static inline int64_t dimos_msg__Float32__fingerprint(void) {
    return (int64_t)782543011003526611LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Float32__type = {
    /* name */          "std_msgs.Float32",
    /* fingerprint */   (int64_t)782543011003526611LL,
    /* encoded_size */  4,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Float32__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
