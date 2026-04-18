/*
 * nav_msgs/MapMetaData — Arduino-compatible LCM C encode/decode.
 * Wire format: Time(8) + float + int32_t + int32_t + Pose(56) = 76 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_MAPMETADATA_H
#define DIMOS_ARDUINO_MSG_MAPMETADATA_H

#include "std_msgs/Time.h"
#include "geometry_msgs/Pose.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Time map_load_time;
    float resolution;
    int32_t width;
    int32_t height;
    dimos_msg__Pose origin;
} dimos_msg__MapMetaData;

static inline int dimos_msg__MapMetaData__encoded_size(void) { return 76; }

static inline int dimos_msg__MapMetaData__encode(void *buf, int offset, int maxlen,
    const dimos_msg__MapMetaData *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Time__encode(buf, offset + pos, maxlen - pos, &p->map_load_time);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->resolution, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->width, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->height, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Pose__encode(buf, offset + pos, maxlen - pos, &p->origin);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__MapMetaData__decode(const void *buf, int offset,
    int maxlen, dimos_msg__MapMetaData *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Time__decode(buf, offset + pos, maxlen - pos, &p->map_load_time);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->resolution, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->width, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->height, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Pose__decode(buf, offset + pos, maxlen - pos, &p->origin);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ MapMetaData::getHash() */
static inline int64_t dimos_msg__MapMetaData__fingerprint(void) {
    return (int64_t)2714794841705235764LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__MapMetaData__type = {
    /* name */          "nav_msgs.MapMetaData",
    /* fingerprint */   (int64_t)2714794841705235764LL,
    /* encoded_size */  76,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__MapMetaData__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
