/*
 * sensor_msgs/NavSatStatus — Arduino-compatible LCM C encode/decode.
 * Wire format: int8_t + int16_t = 3 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_NAVSATSTATUS_H
#define DIMOS_ARDUINO_MSG_NAVSATSTATUS_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t status;
    int16_t service;
} dimos_msg__NavSatStatus;

static inline int dimos_msg__NavSatStatus__encoded_size(void) { return 3; }

static inline int dimos_msg__NavSatStatus__encode(void *buf, int offset, int maxlen,
    const dimos_msg__NavSatStatus *p)
{
    int pos = 0, thislen;
    thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &p->status, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &p->service, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__NavSatStatus__decode(const void *buf, int offset,
    int maxlen, dimos_msg__NavSatStatus *p)
{
    int pos = 0, thislen;
    thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &p->status, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &p->service, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ NavSatStatus::getHash() */
static inline int64_t dimos_msg__NavSatStatus__fingerprint(void) {
    return (int64_t)-1340827276200410186LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__NavSatStatus__type = {
    /* name */          "sensor_msgs.NavSatStatus",
    /* fingerprint */   (int64_t)-1340827276200410186LL,
    /* encoded_size */  3,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__NavSatStatus__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
