/*
 * shape_msgs/MeshTriangle — Arduino-compatible LCM C encode/decode.
 * Wire format: 3x int32_t(12) = 12 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_MESHTRIANGLE_H
#define DIMOS_ARDUINO_MSG_MESHTRIANGLE_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t vertex_indices[3];
} dimos_msg__MeshTriangle;

static inline int dimos_msg__MeshTriangle__encoded_size(void) { return 12; }

static inline int dimos_msg__MeshTriangle__encode(void *buf, int offset, int maxlen,
    const dimos_msg__MeshTriangle *p)
{
    int pos = 0, thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, p->vertex_indices, 3);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__MeshTriangle__decode(const void *buf, int offset,
    int maxlen, dimos_msg__MeshTriangle *p)
{
    int pos = 0, thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, p->vertex_indices, 3);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ MeshTriangle::getHash() */
static inline int64_t dimos_msg__MeshTriangle__fingerprint(void) {
    return (int64_t)6912568532040753946LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__MeshTriangle__type = {
    /* name */          "shape_msgs.MeshTriangle",
    /* fingerprint */   (int64_t)6912568532040753946LL,
    /* encoded_size */  12,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__MeshTriangle__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
