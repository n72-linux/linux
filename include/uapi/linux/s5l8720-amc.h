/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * S5L8720 AMC audio coprocessor (iPod touch 2G): hardware audio decoding.
 *
 * Open /dev/amc (one user at a time), AMC_IOC_START a stream, then hand it
 * whole compressed packets with AMC_IOC_DECODE; every packet comes back as
 * one frame of 16-bit little-endian interleaved PCM at the stream's rate.
 * With AMC_CODEC_AACENC the direction is reversed: AMC_IOC_ENCODE takes
 * 1024-frame blocks of that PCM and returns raw AAC-LC access units.
 *
 * Several opens share the coprocessor (each is saved and restored around
 * the others), but the encoder and a decoder do not fit together in real
 * time, so one gives way with EBUSY (also mid-stream) and its caller goes
 * on on the CPU: AAC-LC and MP3 decoders give way to the encoder; the
 * encoder gives way to HE-AAC, ALAC and Audible decoders (no CPU decoder
 * of the same quality on the device). Decoders share with each other.
 * Packets are raw: AAC access units without ADTS/LATM headers, MP3 frames
 * with their header, ALAC packets as stored in the MP4 file.
 */
#ifndef _UAPI_LINUX_S5L8720_AMC_H
#define _UAPI_LINUX_S5L8720_AMC_H

#include <linux/ioctl.h>
#include <linux/types.h>

enum amc_codec {
	AMC_CODEC_MP3 = 0,	/* MPEG-1/2 Layer III */
	AMC_CODEC_AAC = 1,	/* AAC-LC */
	AMC_CODEC_HEAAC = 2,	/* HE-AAC (SBR), HE-AACv2 with AMC_START_PS */
	AMC_CODEC_ALAC = 3,	/* Apple Lossless, 16-bit */
	AMC_CODEC_AUDB = 4,	/* Audible, 8 or 16 kHz */
	AMC_CODEC_AACENC = 5,	/* AAC-LC encoder: 16-bit PCM in, raw AAC out */
};

#define AMC_START_PS	(1 << 0)	/* HE-AAC: parametric stereo (aacp) */

struct amc_start {
	__u32 codec;		/* enum amc_codec */
	__u32 rate;		/* output sample rate in Hz (HE-AAC: the SBR rate) */
	__u32 channels;		/* output channels, 1 or 2 */
	__u32 flags;		/* AMC_START_* */
	__u8 cookie[4];		/* ALAC: bitDepth, pb, mb, kb from the 'alac' atom */
	/* returned */
	__u32 frames_per_packet;	/* PCM frames per packet */
	__u32 bitrate;		/* AACENC: bits per second (0 = 128000) */
	__u32 reserved[3];
};

#define AMC_DECODE_DRAIN	(1 << 0)	/* last call of the stream: pad the input */

struct amc_decode {
	__u64 in;		/* packets back to back */
	__u64 sizes;		/* __u32 sizes[npackets], bytes */
	__u64 out;		/* PCM */
	__u32 npackets;
	__u32 out_size;		/* bytes available at out */
	__u32 flags;		/* AMC_DECODE_* */
	/* returned */
	__u32 packets_done;
	__u32 out_bytes;
	__s32 error;		/* 0, or the coprocessor's error code */
	__u32 reserved[4];
};

/* limits of one AMC_IOC_DECODE call */
#define AMC_MAX_INPUT		0x8000
#define AMC_MAX_PACKETS		256

/*
 * AMC_IOC_ENCODE: in_bytes = a whole number of 1024-frame blocks (at most
 * AMC_MAX_ENC_BLOCKS). Each block yields at most one access unit (the
 * encoder's start-up delay yields none); out gets them back to back,
 * sizes[] their lengths. At the end of a stream, encode three blocks of
 * silence to get the last samples out.
 */
#define AMC_MAX_ENC_BLOCKS	8
#define AMC_MAX_AU		1536	/* bytes: 6144 bits per channel, 2 channels */

struct amc_encode {
	__u64 in;		/* PCM, S16_LE interleaved */
	__u64 out;		/* access units */
	__u64 sizes;		/* __u32 sizes[AMC_MAX_ENC_BLOCKS] */
	__u32 in_bytes;
	__u32 out_size;
	__u32 flags;		/* none yet: 0 */
	/* returned */
	__u32 blocks_done;
	__u32 packets;
	__u32 out_bytes;
	__s32 error;		/* 0, or why the encoder stopped (call START again) */
	__u32 notes;		/* blocks with a status code the encoder went on after */
	__u32 last_note;	/* the last such code */
	__u32 reserved[2];
};

#define AMC_IOC_START	_IOWR(0xA6, 0x00, struct amc_start)
#define AMC_IOC_DECODE	_IOWR(0xA6, 0x01, struct amc_decode)
#define AMC_IOC_ENCODE	_IOWR(0xA6, 0x02, struct amc_encode)

#endif
