/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024-present, Tampere University, ITU/ISO/IEC, project contributors
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

#include "bitstream_util.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>

// NOLINTBEGIN(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc) //
// TODO(gg) : lf : Currently we manually handle most of the memory object in the bitstream generation. Consider reducing alloc use to minimum

const std::array<uint32_t, 32> uvg_bit_set_mask = {
    0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080, 0x00000100, 0x00000200, 0x00000400,
    0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000, 0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000,
    0x00400000, 0x00800000, 0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000};

namespace {

unsigned uvg_math_floor_log2(unsigned value) {
    assert(value > 0);

    unsigned result = 0;

    for (size_t i = 4;; --i) {
        const unsigned bits = 1ULL << i;
        const unsigned shift = value >= (1U << bits) ? bits : 0;
        result += shift;
        value >>= shift;
        if (i == 0) {
            break;
        }
    }

    return result;
}

}  // anonymous namespace

void uvg_bitstream_init(bitstream_t *const stream) { memset(stream, 0, sizeof(bitstream_t)); }

uvg_data_chunk *uvg_bitstream_alloc_chunk() {
    uvg_data_chunk *chunk = static_cast<uvg_data_chunk *>(malloc(sizeof(uvg_data_chunk)));
    if (chunk != nullptr) {
        chunk->len = 0;
        chunk->next = nullptr;
    }
    return chunk;
}

void uvg_bitstream_free_chunks(uvg_data_chunk *chunk) {
    while (chunk != nullptr) {
        uvg_data_chunk *next = chunk->next;
        free(chunk);
        chunk = next;
    }
}

/**
 * \brief Write a byte to bitstream
 *
 * The stream must be byte-aligned.
 *
 * \param stream  pointer bitstream to put the data
 * \param byte    byte to write
 */
void uvg_bitstream_writebyte(bitstream_t *const stream, const uint8_t byte) {
    assert(stream->cur_bit == 0);

    if (stream->last == nullptr || stream->last->len == UVG_DATA_CHUNK_SIZE) {
        // Need to allocate a new chunk.
        uvg_data_chunk *new_chunk = uvg_bitstream_alloc_chunk();
        assert(new_chunk);

        if (stream->first == nullptr) {
            stream->first = new_chunk;
        }
        if (stream->last != nullptr) {
            stream->last->next = new_chunk;
        }
        stream->last = new_chunk;
    }
    assert(stream->last->len < UVG_DATA_CHUNK_SIZE);

    stream->last->data[stream->last->len] = byte;
    stream->last->len += 1;
    stream->len += 1;
}

void uvg_bitstream_put(bitstream_t *const stream, const uint32_t data, uint8_t bits) {
    while (bits-- != 0) {
        stream->data <<= 1U;

        if ((data & uvg_bit_set_mask[bits]) != 0U) {
            stream->data |= 1U;
        }
        stream->cur_bit++;

        // write byte to output
        if (stream->cur_bit == 8) {
            stream->cur_bit = 0;
            uvg_bitstream_writebyte(stream, stream->data);
        }
    }
}

uvg_data_chunk *uvg_bitstream_take_chunks(bitstream_t *const stream) {
    assert(stream->cur_bit == 0);
    uvg_data_chunk *chunks = stream->first;
    stream->first = stream->last = nullptr;
    stream->len = 0;
    return chunks;
}

void uvg_bitstream_finalize(bitstream_t *const stream) {
    // uvg_bitstream_clear(stream); // TODO(lf): to delete
    uvg_bitstream_free_chunks(stream->first);
    delete stream;
}

/**
 * Reset stream.
 */
void uvg_bitstream_clear(bitstream_t *const stream) {
    uvg_bitstream_free_chunks(stream->first);
    uvg_bitstream_init(stream);
}

uint64_t uvg_bitstream_tell(const bitstream_t *const stream) {
    const uint64_t position = stream->len;
    return position * 8 + stream->cur_bit;
}

void uvg_bitstream_put_ue(bitstream_t *stream, uint32_t code_num) {
    const unsigned code_num_log2 = uvg_math_floor_log2(code_num + 1);
    const unsigned prefix = 1U << code_num_log2;
    const unsigned suffix = code_num + 1 - prefix;
    const unsigned num_bits = code_num_log2 * 2 + 1;
    const unsigned value = prefix | suffix;

    uvg_bitstream_put(stream, value, num_bits);
}

// TODO(lf): rename this function
size_t uvg_calculate_ue_len(uint32_t number) {
    const unsigned code_num_log2 = uvg_math_floor_log2(number + 1);
    //   unsigned prefix = 1 << code_num_log2;
    //   unsigned suffix = number + 1 - prefix;
    return code_num_log2 * 2 + 1;
}

void uvg_bitstream_add_rbsp_trailing_bits(bitstream_t *const stream) {
    uvg_bitstream_put(stream, 1, 1);
    if ((stream->cur_bit & 7U) != 0) {
        uvg_bitstream_put(stream, 0, 8 - (stream->cur_bit & 7U));
    }
}

void uvg_bitstream_align(bitstream_t *const stream) {
    if ((stream->cur_bit & 7U) != 0) {
        uvg_bitstream_add_rbsp_trailing_bits(stream);
    }
}

void uvg_bitstream_move(bitstream_t *const dst, bitstream_t *const src) {
    assert(dst->cur_bit == 0);

    if (src->len > 0) {
        if (dst->first == nullptr) {
            dst->first = src->first;
            dst->last = src->last;
            dst->len = src->len;
        } else {
            dst->last->next = src->first;
            dst->last = src->last;
            dst->len += src->len;
        }
    }

    // Move the leftover bits.
    dst->data = src->data;
    dst->cur_bit = src->cur_bit;

    src->first = src->last = nullptr;
    uvg_bitstream_clear(src);
}

void uvg_bitstream_copy_bytes(bitstream_t *const stream, const uint8_t *bytes, uint32_t len) {
    assert(stream->cur_bit == 0);
    uint32_t ptr = 0;
    uint32_t data_left = len;
    // std::cout << "- copy_bytes, len: " << len << std::endl;

    while (ptr < len) {
        const uint32_t space_left_in_chunk = UVG_DATA_CHUNK_SIZE - stream->last->len;
        // std::cout << "--- data_left: " << data_left << std::endl;
        // std::cout << "--- ptr: " << ptr << ", space_left_in_chunk: " << space_left_in_chunk << std::endl;

        if (data_left < space_left_in_chunk) {
            // std::cout << "--- fits, from src, copy bytes: " << ptr << " to " << ptr + data_left << std::endl;
            memcpy(&stream->last->data[stream->last->len], &bytes[ptr], data_left);
            ptr += data_left;
            stream->last->len += data_left;
            stream->len += data_left;
            data_left -= len - ptr;
            // std::cout << "------ end, data_left: " << data_left << std::endl;
        } else {
            // std::cout << "--- doesnt fit, from src, copy bytes: " << ptr << " to " << ptr + space_left_in_chunk << std::endl;
            memcpy(&stream->last->data[stream->last->len], &bytes[ptr], space_left_in_chunk);
            ptr += space_left_in_chunk;
            stream->last->len += space_left_in_chunk;
            stream->len += space_left_in_chunk;
            data_left -= space_left_in_chunk;

            if (stream->last == nullptr || stream->last->len == UVG_DATA_CHUNK_SIZE) {
                // Need to allocate a new chunk.
                uvg_data_chunk *new_chunk = uvg_bitstream_alloc_chunk();
                assert(new_chunk);

                if (stream->first == nullptr) {
                    stream->first = new_chunk;
                }
                if (stream->last != nullptr) {
                    stream->last->next = new_chunk;
                }
                stream->last = new_chunk;
            }
            assert(stream->last->len < UVG_DATA_CHUNK_SIZE);
        }
    }
}

uint32_t uvg_bitstream_peek_last_byte(bitstream_t *const stream) { return stream->data; }

// NOLINTEND(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)
