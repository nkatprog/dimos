/*
 * std_msgs/Int64 — Arduino-compatible LCM C encode/decode.
 * Wire format: int64_t = 8 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_INT64_H
#define DIMOS_ARDUINO_MSG_INT64_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t data;
} dimos_msg__Int64;

static inline int dimos_msg__Int64__encoded_size(void) { return 8; }

static inline int dimos_msg__Int64__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Int64 *p)
{
    return __int64_t_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Int64__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Int64 *p)
{
    return __int64_t_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ Int64::getHash() */
static inline int64_t dimos_msg__Int64__fingerprint(void) {
    return (int64_t)3223726302314955326LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Int64__type = {
    /* name */          "std_msgs.Int64",
    /* fingerprint */   (int64_t)3223726302314955326LL,
    /* encoded_size */  8,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Int64__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
