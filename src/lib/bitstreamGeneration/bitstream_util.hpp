/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024, Tampere University, ITU/ISO/IEC, project contributors
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of the Tampere University or ITU/ISO/IEC nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

#pragma once

#include <cassert>
#include <cstring>

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

#define BITSTREAM_DEBUG false

/* Size of data chunks */
#define UVG_DATA_CHUNK_SIZE 4096

/* A linked list of chunks of data, used for returning the encoded data */
typedef struct uvg_data_chunk {
    /* Buffer for the data */
    uint8_t data[UVG_DATA_CHUNK_SIZE];

    /* Number of bytes filled in this chunk */
    uint32_t len;

    /* Next chunk in the list */
    struct uvg_data_chunk *next;
} uvg_data_chunk;

/* A stream of bits */
typedef struct bitstream_t {
    /* Total number of complete bytes */
    uint32_t len;

    /* Pointer to the first chunk, or NULL */
    uvg_data_chunk *first;

    /* Pointer to the last chunk, or NULL */
    uvg_data_chunk *last;

    /* The incomplete byte */
    uint8_t data;

    /* Number of bits in the incomplete byte */
    uint8_t cur_bit;
} bitstream_t;

/* Initialize a new bitstream */
void uvg_bitstream_init(bitstream_t *const stream);

/* Allocates a new bitstream chunk */
uvg_data_chunk *uvg_bitstream_alloc_chunk();

/* Free a list of chunks */
void uvg_bitstream_free_chunks(uvg_data_chunk *chunk);

/* Write a byte to a byte-aligned bitstream */
void uvg_bitstream_writebyte(bitstream_t *const stream, const uint8_t byte);

/* Write bits to bitstream. Buffers individual bits until they make a full byte */
void uvg_bitstream_put(bitstream_t *const stream, const uint32_t data, uint8_t bits);

/* Take chunks from a byte-aligned bitstream. Moves ownership of the chunks to the caller and clears the bitstream */
uvg_data_chunk *uvg_bitstream_take_chunks(bitstream_t *stream);

/* Free resources used by a bitstream */
void uvg_bitstream_finalize(bitstream_t *stream);

/* Reset stream */
void uvg_bitstream_clear(bitstream_t *const stream);

/* Get the number of bits written */
uint64_t uvg_bitstream_tell(const bitstream_t *stream);

/* Write unsigned Exp-Golomb bit string */
void uvg_bitstream_put_ue(bitstream_t *stream, uint32_t code_num);

/* Calculate amount of bits required to represent a number in Exp-Golomb format */
std::size_t uvg_calculate_ue_len(uint32_t number);

/* Add rbsp_trailing_bits syntax element, which aligns the bitstream */
void uvg_bitstream_add_rbsp_trailing_bits(bitstream_t *const stream);

/* Align the bitstream, unless it's already aligned */
void uvg_bitstream_align(bitstream_t *const stream);

/* Move data from one stream to another. Destination stream must be byte-aligned. Source stream will be cleared */
void uvg_bitstream_move(bitstream_t *const dst, bitstream_t *const src);

/**
 * \brief Copy array of bytes to a byte-aligned bitstream
 * \param stream  pointer bitstream to put the data
 * \param bytes   bytes to copy
 * \param len     length of bytes array
 */
void uvg_bitstream_copy_bytes(bitstream_t *const stream, const uint8_t *bytes, uint32_t len);

/* Get the last (possibly incomplete) byte of the bitstream */
uint32_t uvg_bitstream_peek_last_byte(bitstream_t *const stream);

/* In debug mode print out some extra info */
#if BITSTREAM_DEBUG
/* Counter to keep up with bits written */
#define WRITE_U(stream, data, bits, name)               \
    {                                                   \
        printf("%-50s u(%d) : %d\n", name, bits, data); \
        uvg_bitstream_put(stream, data, bits);          \
    }
#define WRITE_UE(stream, data, name)             \
    {                                            \
        printf("%-50s ue(v): %d\n", name, data); \
        uvg_bitstream_put_ue(stream, data);      \
    }
#else
#define WRITE_U(stream, data, bits, name)      \
    {                                          \
        uvg_bitstream_put(stream, data, bits); \
    }
#define WRITE_UE(stream, data, name)        \
    {                                       \
        uvg_bitstream_put_ue(stream, data); \
    }
#endif
