/*
 * vision_msgs/BoundingBox3D — Arduino-compatible LCM C encode/decode.
 * Wire format: Pose(56) + Vector3(24) = 80 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_BOUNDINGBOX3D_H
#define DIMOS_ARDUINO_MSG_BOUNDINGBOX3D_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Pose center;
    dimos_msg__Vector3 size;
} dimos_msg__BoundingBox3D;

static inline int dimos_msg__BoundingBox3D__encoded_size(void) { return 80; }

static inline int dimos_msg__BoundingBox3D__encode(void *buf, int offset, int maxlen,
    const dimos_msg__BoundingBox3D *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__encode(buf, offset + pos, maxlen - pos, &p->center);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->size);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__BoundingBox3D__decode(const void *buf, int offset,
    int maxlen, dimos_msg__BoundingBox3D *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__decode(buf, offset + pos, maxlen - pos, &p->center);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->size);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ BoundingBox3D::getHash() */
static inline int64_t dimos_msg__BoundingBox3D__fingerprint(void) {
    return (int64_t)7478515651293570543LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__BoundingBox3D__type = {
    /* name */          "vision_msgs.BoundingBox3D",
    /* fingerprint */   (int64_t)7478515651293570543LL,
    /* encoded_size */  80,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__BoundingBox3D__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
