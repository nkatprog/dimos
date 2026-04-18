/*
 * std_msgs/Empty — Arduino-compatible LCM C encode/decode.
 * Wire format:  = 0 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_EMPTY_H
#define DIMOS_ARDUINO_MSG_EMPTY_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
} dimos_msg__Empty;

static inline int dimos_msg__Empty__encoded_size(void) { return 0; }

static inline int dimos_msg__Empty__encode(void *buf, int offset, int maxlen,
    const dimos_msg__Empty *p)
{
    int pos = 0, thislen;
    return pos;
}

static inline int dimos_msg__Empty__decode(const void *buf, int offset,
    int maxlen, dimos_msg__Empty *p)
{
    int pos = 0, thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ Empty::getHash() */
static inline int64_t dimos_msg__Empty__fingerprint(void) {
    return (int64_t)610839792LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__Empty__type = {
    /* name */          "std_msgs.Empty",
    /* fingerprint */   (int64_t)610839792LL,
    /* encoded_size */  0,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__Empty__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
