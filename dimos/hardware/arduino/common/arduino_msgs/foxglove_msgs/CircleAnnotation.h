/*
 * foxglove_msgs/CircleAnnotation — Arduino-compatible LCM C encode/decode.
 * Wire format: Time(8) + Point2(16) + double + double + Color(32) + Color(32) = 104 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_CIRCLEANNOTATION_H
#define DIMOS_ARDUINO_MSG_CIRCLEANNOTATION_H

#include "builtin_interfaces/Time.h"
#include "foxglove_msgs/Point2.h"
#include "foxglove_msgs/Color.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Time timestamp;
    dimos_msg__Point2 position;
    double diameter;
    double thickness;
    dimos_msg__Color fill_color;
    dimos_msg__Color outline_color;
} dimos_msg__CircleAnnotation;

static inline int dimos_msg__CircleAnnotation__encoded_size(void) { return 104; }

static inline int dimos_msg__CircleAnnotation__encode(void *buf, int offset, int maxlen,
    const dimos_msg__CircleAnnotation *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Time__encode(buf, offset + pos, maxlen - pos, &p->timestamp);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Point2__encode(buf, offset + pos, maxlen - pos, &p->position);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->diameter, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->thickness, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__encode(buf, offset + pos, maxlen - pos, &p->fill_color);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__encode(buf, offset + pos, maxlen - pos, &p->outline_color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__CircleAnnotation__decode(const void *buf, int offset,
    int maxlen, dimos_msg__CircleAnnotation *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Time__decode(buf, offset + pos, maxlen - pos, &p->timestamp);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Point2__decode(buf, offset + pos, maxlen - pos, &p->position);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->diameter, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->thickness, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__decode(buf, offset + pos, maxlen - pos, &p->fill_color);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Color__decode(buf, offset + pos, maxlen - pos, &p->outline_color);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

/* LCM fingerprint hash — matches C++ CircleAnnotation::getHash() */
static inline int64_t dimos_msg__CircleAnnotation__fingerprint(void) {
    return (int64_t)7120207774344483495LL;
}

/* Type descriptor for dimos_lcm_pubsub — include dimos_lcm_pubsub.h first */
#ifdef DIMOS_LCM_PUBSUB_H
static const dimos_lcm_type_t dimos_msg__CircleAnnotation__type = {
    /* name */          "foxglove_msgs.CircleAnnotation",
    /* fingerprint */   (int64_t)7120207774344483495LL,
    /* encoded_size */  104,
    /* decode */        (int (*)(const void *, int, int, void *))dimos_msg__CircleAnnotation__decode
};
#endif

#ifdef __cplusplus
}
#endif

#endif
