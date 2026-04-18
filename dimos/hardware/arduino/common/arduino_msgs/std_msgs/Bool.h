/*
 * std_msgs/Bool — Arduino-compatible LCM C encode/decode.
 * Wire format: boolean = 1 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_BOOL_H
#define DIMOS_ARDUINO_MSG_BOOL_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t data;
} dimos_msg__Bool;

static inline int dimos_msg__Bool__encoded_size(void) { return 1; }

static inline int dimos_msg__Bool__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Bool *p)
{
    return __int8_t_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Bool__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Bool *p)
{
    return __int8_t_decode_array(buf, offset, maxlen, &p->data, 1);
}

/* LCM fingerprint hash — matches C++ Bool::getHash() */
static inline int64_t dimos_msg__Bool__fingerprint(void) {
    return (int64_t)2215472406122008126LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Bool__type = {
    /* name */          "std_msgs.Bool",
    /* fingerprint */   (int64_t)2215472406122008126LL,
    /* encoded_size */  1,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Bool__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
