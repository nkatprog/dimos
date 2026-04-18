/*
 * std_msgs/Float64 — Arduino-compatible LCM C encode/decode.
 * Wire format: double = 8 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_FLOAT64_H
#define DIMOS_ARDUINO_MSG_FLOAT64_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double data;
} dimos_msg__Float64;

static inline int dimos_msg__Float64__encoded_size(void) { return 8; }

static inline int dimos_msg__Float64__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Float64 *p)
{
    return __double_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Float64__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Float64 *p)
{
    return __double_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ Float64::getHash() */
static inline int64_t dimos_msg__Float64__fingerprint(void) {
    return (int64_t)2440178057091310101LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Float64__type = {
    /* name */          "std_msgs.Float64",
    /* fingerprint */   (int64_t)2440178057091310101LL,
    /* encoded_size */  8,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Float64__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
