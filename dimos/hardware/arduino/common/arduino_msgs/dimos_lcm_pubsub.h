/*
 * dimos_lcm_pubsub.h
 *
 * Transport-agnostic LCM publish/subscribe layer for embedded systems.
 *
 * This header provides channel-based message routing without any transport
 * dependencies (no serial, no network, no Arduino.h).  It pairs with
 * lcm_coretypes_arduino.h for encode/decode primitives and generated
 * per-type headers for message structs.
 *
 * Design constraints:
 *   - No malloc / no dynamic allocation — all fixed-size buffers
 *   - Works on AVR 8-bit (2 KB SRAM) with reduced defaults
 *   - All functions are static inline (single-header, no .c file)
 *   - Transport layer calls dimos_lcm_dispatch() for inbound messages
 *   - Transport layer drains outbox via dimos_lcm_pop_outbound()
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

#ifndef DIMOS_LCM_PUBSUB_H
#define DIMOS_LCM_PUBSUB_H

#include <stdint.h>
#include <string.h>
#include "lcm_coretypes_arduino.h"

/* ------------------------------------------------------------------ */
/*  Compile-time configuration                                        */
/* ------------------------------------------------------------------ */

/** Maximum number of active subscriptions. */
#ifndef DIMOS_LCM_MAX_SUBS
  #ifdef __AVR__
    #define DIMOS_LCM_MAX_SUBS 4
  #else
    #define DIMOS_LCM_MAX_SUBS 16
  #endif
#endif

/** Maximum number of pending outbound messages in the outbox. */
#ifndef DIMOS_LCM_MAX_PENDING
  #ifdef __AVR__
    #define DIMOS_LCM_MAX_PENDING 2
  #else
    #define DIMOS_LCM_MAX_PENDING 8
  #endif
#endif

/**
 * Maximum encoded message payload size (excluding the 8-byte fingerprint).
 * Also used as the decode staging buffer size, so it must be at least as
 * large as the biggest decoded struct you will handle.
 *
 * On AVR, default to 64 bytes to conserve SRAM.  Override via
 * -DDIMOS_LCM_MAX_MSG_SIZE=<N> for chips with more SRAM (e.g. Mega 2560).
 */
#ifndef DIMOS_LCM_MAX_MSG_SIZE
  #ifdef __AVR__
    #define DIMOS_LCM_MAX_MSG_SIZE 64
  #else
    #define DIMOS_LCM_MAX_MSG_SIZE 512
  #endif
#endif

/** Size of the fingerprint prefix on the wire (always 8 bytes). */
#define DIMOS_LCM_FINGERPRINT_SIZE 8

/* ------------------------------------------------------------------ */
/*  Type descriptor                                                   */
/* ------------------------------------------------------------------ */

/**
 * Per-message-type descriptor.  One of these is generated for each LCM
 * message type and referenced by subscriptions and publish calls.
 *
 * Fields:
 *   name           Human-readable type name, e.g. "geometry_msgs.Twist"
 *   fingerprint    8-byte LCM structural hash (compared big-endian on wire)
 *   encoded_size   Fixed encoded payload size in bytes (we only support
 *                  fixed-size types on embedded targets)
 *   decode         Function that decodes `encoded_size` bytes from `buf`
 *                  starting at `offset` into the struct pointed to by `out`.
 *                  Returns number of bytes consumed on success, or <0 on error.
 */
typedef struct {
    const char *name;
    int64_t     fingerprint;
    int         encoded_size;
    int       (*decode)(const void *buf, int offset, int maxlen, void *out);
} dimos_lcm_type_t;

/* ------------------------------------------------------------------ */
/*  Callback signature                                                */
/* ------------------------------------------------------------------ */

/**
 * Subscription handler callback.
 *
 * @param channel      Channel name the message arrived on.
 * @param decoded_msg  Pointer to the decoded message struct (lives in the
 *                     lcm instance's decode_buf — valid only for the
 *                     duration of this callback).
 * @param user_data    Opaque pointer supplied at subscribe time.
 */
typedef void (*dimos_lcm_handler_t)(const char *channel,
                                     const void *decoded_msg,
                                     void *user_data);

/* ------------------------------------------------------------------ */
/*  Internal data structures                                          */
/* ------------------------------------------------------------------ */

/** Subscription table entry. */
typedef struct {
    const char            *channel;
    const dimos_lcm_type_t *type;
    dimos_lcm_handler_t    handler;
    void                  *user_data;
    uint8_t                active;
} dimos_lcm_sub_t;

/** Pending outbound message (queued by publish, drained by transport). */
typedef struct {
    const char *channel;
    int64_t     fingerprint;
    uint8_t     data[DIMOS_LCM_MAX_MSG_SIZE]; /* encoded payload (no fingerprint) */
    uint16_t    data_len;
    uint8_t     pending;
} dimos_lcm_outmsg_t;

/** Main LCM pubsub instance — allocate one of these (typically global). */
typedef struct {
    dimos_lcm_sub_t    subs[DIMOS_LCM_MAX_SUBS];
    uint8_t            num_subs;
    dimos_lcm_outmsg_t outbox[DIMOS_LCM_MAX_PENDING];
    uint8_t            num_pending;
    /** Temporary staging buffer for decoded structs during dispatch. */
    uint8_t            decode_buf[DIMOS_LCM_MAX_MSG_SIZE];
} dimos_lcm_t;

/* ------------------------------------------------------------------ */
/*  API — all static inline                                           */
/* ------------------------------------------------------------------ */

/**
 * Initialise an LCM pubsub instance.  Must be called before any other
 * function.  Zeroes all subscriptions and the outbox.
 */
static inline void dimos_lcm_init(dimos_lcm_t *lcm)
{
    memset(lcm, 0, sizeof(*lcm));
}

/**
 * Subscribe to messages on `channel` of the given `type`.
 *
 * @param lcm        LCM instance.
 * @param channel    Channel name (must point to storage that outlives the
 *                   subscription — typically a string literal or generated
 *                   constant).
 * @param type       Type descriptor for the expected message type.
 * @param handler    Callback invoked when a matching message arrives.
 * @param user_data  Opaque pointer forwarded to the handler.
 * @return           Subscription index (>= 0) on success, -1 if the
 *                   subscription table is full.
 */
static inline int dimos_lcm_subscribe(dimos_lcm_t *lcm,
                                       const char *channel,
                                       const dimos_lcm_type_t *type,
                                       dimos_lcm_handler_t handler,
                                       void *user_data)
{
    /* Look for a free slot (either beyond num_subs, or a deactivated entry). */
    int slot = -1;

    for (int i = 0; i < DIMOS_LCM_MAX_SUBS; i++) {
        if (!lcm->subs[i].active) {
            slot = i;
            break;
        }
    }
    if (slot < 0) return -1;

    lcm->subs[slot].channel   = channel;
    lcm->subs[slot].type      = type;
    lcm->subs[slot].handler   = handler;
    lcm->subs[slot].user_data = user_data;
    lcm->subs[slot].active    = 1;

    if ((uint8_t)(slot + 1) > lcm->num_subs)
        lcm->num_subs = (uint8_t)(slot + 1);

    return slot;
}

/**
 * Unsubscribe a previously registered subscription.
 *
 * @param lcm        LCM instance.
 * @param sub_index  Index returned by dimos_lcm_subscribe().
 */
static inline void dimos_lcm_unsubscribe(dimos_lcm_t *lcm, int sub_index)
{
    if (sub_index < 0 || sub_index >= DIMOS_LCM_MAX_SUBS) return;
    lcm->subs[sub_index].active = 0;
}

/* ------------------------------------------------------------------ */
/*  Inbound dispatch                                                  */
/* ------------------------------------------------------------------ */

/**
 * Read an int64_t from a big-endian byte buffer (matching LCM wire order).
 */
static inline int64_t dimos_lcm__read_fingerprint(const uint8_t *buf)
{
    int64_t v = 0;
    for (int i = 0; i < 8; i++) {
        v = (v << 8) | buf[i];
    }
    return v;
}

/**
 * Dispatch an inbound message to matching subscriber(s).
 *
 * The transport layer calls this after it has received a complete, framed
 * message.  The payload layout is:
 *
 *     [ 8-byte fingerprint (big-endian) ][ encoded message data ]
 *
 * Processing steps:
 *   1. Extract the 8-byte fingerprint from the payload.
 *   2. Walk the subscription table for entries matching `channel`.
 *   3. Verify the fingerprint matches the subscription's expected type.
 *   4. Decode the message into the instance's staging buffer.
 *   5. Invoke the handler callback.
 *
 * Multiple subscriptions on the same channel are supported (each is
 * dispatched independently).
 *
 * @param lcm          LCM instance.
 * @param channel      Channel name for this message.
 * @param payload      Raw payload: [fingerprint][encoded data].
 * @param payload_len  Total length of `payload` in bytes.
 * @return             Number of handlers invoked, or -1 on framing error.
 */
static inline int dimos_lcm_dispatch(dimos_lcm_t *lcm,
                                      const char *channel,
                                      const uint8_t *payload,
                                      uint16_t payload_len)
{
    if (payload_len < DIMOS_LCM_FINGERPRINT_SIZE)
        return -1;

    int64_t wire_fp = dimos_lcm__read_fingerprint(payload);
    const uint8_t *data     = payload + DIMOS_LCM_FINGERPRINT_SIZE;
    uint16_t       data_len = payload_len - DIMOS_LCM_FINGERPRINT_SIZE;

    int dispatched = 0;

    for (uint8_t i = 0; i < lcm->num_subs; i++) {
        dimos_lcm_sub_t *sub = &lcm->subs[i];
        if (!sub->active) continue;
        if (strcmp(sub->channel, channel) != 0) continue;
        if (sub->type->fingerprint != wire_fp) continue;

        /* Verify payload size. */
        if (data_len < (uint16_t)sub->type->encoded_size) continue;
        if ((uint16_t)sub->type->encoded_size > DIMOS_LCM_MAX_MSG_SIZE) continue;

        /* Decode into staging buffer. */
        int rc = sub->type->decode(data, 0, (int)data_len, lcm->decode_buf);
        if (rc < 0) continue;

        /* Deliver. */
        sub->handler(channel, lcm->decode_buf, sub->user_data);
        dispatched++;
    }

    return dispatched;
}

/* ------------------------------------------------------------------ */
/*  Outbound publish                                                  */
/* ------------------------------------------------------------------ */

/**
 * Write an int64_t to a buffer in big-endian byte order.
 */
static inline void dimos_lcm__write_fingerprint(uint8_t *buf, int64_t fp)
{
    for (int i = 7; i >= 0; i--) {
        buf[i] = (uint8_t)(fp & 0xFF);
        fp >>= 8;
    }
}

/**
 * Queue an encoded message for outbound transmission.
 *
 * The caller supplies already-encoded message data (produced by the
 * generated encode function).  The pubsub layer stores the fingerprint
 * alongside the data so the transport can prepend it when sending.
 *
 * @param lcm           LCM instance.
 * @param channel       Channel name to publish on.
 * @param type          Type descriptor (used for the fingerprint).
 * @param encoded_data  Encoded message bytes (no fingerprint prefix).
 * @param data_len      Length of encoded_data.
 * @return              0 on success, -1 if the outbox is full or the
 *                      message is too large.
 */
static inline int dimos_lcm_publish(dimos_lcm_t *lcm,
                                     const char *channel,
                                     const dimos_lcm_type_t *type,
                                     const uint8_t *encoded_data,
                                     uint16_t data_len)
{
    if (data_len > DIMOS_LCM_MAX_MSG_SIZE)
        return -1;

    /* Find a free outbox slot. */
    for (uint8_t i = 0; i < DIMOS_LCM_MAX_PENDING; i++) {
        if (!lcm->outbox[i].pending) {
            lcm->outbox[i].channel     = channel;
            lcm->outbox[i].fingerprint = type->fingerprint;
            memcpy(lcm->outbox[i].data, encoded_data, data_len);
            lcm->outbox[i].data_len    = data_len;
            lcm->outbox[i].pending     = 1;
            lcm->num_pending++;
            return 0;
        }
    }
    return -1; /* outbox full */
}

/**
 * Check whether there are any pending outbound messages.
 *
 * @return Non-zero if at least one message is queued.
 */
static inline int dimos_lcm_has_outbound(const dimos_lcm_t *lcm)
{
    return lcm->num_pending > 0;
}

/**
 * Pop the next pending outbound message from the outbox.
 *
 * The output buffer receives the full wire payload:
 *
 *     [ 8-byte fingerprint (big-endian) ][ encoded message data ]
 *
 * @param lcm          LCM instance.
 * @param out_channel  Receives a pointer to the channel name string.
 * @param out_buf      Destination buffer for the wire payload.
 * @param out_buf_max  Size of out_buf in bytes.
 * @return             Total bytes written (8 + data_len), or 0 if nothing
 *                     is pending or the output buffer is too small.
 */
static inline uint16_t dimos_lcm_pop_outbound(dimos_lcm_t *lcm,
                                               const char **out_channel,
                                               uint8_t *out_buf,
                                               uint16_t out_buf_max)
{
    for (uint8_t i = 0; i < DIMOS_LCM_MAX_PENDING; i++) {
        if (!lcm->outbox[i].pending) continue;

        uint16_t total = DIMOS_LCM_FINGERPRINT_SIZE + lcm->outbox[i].data_len;
        if (total > out_buf_max)
            return 0;

        /* Write fingerprint in big-endian. */
        dimos_lcm__write_fingerprint(out_buf, lcm->outbox[i].fingerprint);

        /* Copy encoded payload. */
        memcpy(out_buf + DIMOS_LCM_FINGERPRINT_SIZE,
               lcm->outbox[i].data,
               lcm->outbox[i].data_len);

        *out_channel = lcm->outbox[i].channel;

        /* Mark slot as free. */
        lcm->outbox[i].pending = 0;
        lcm->num_pending--;

        return total;
    }
    return 0;
}

#endif /* DIMOS_LCM_PUBSUB_H */
