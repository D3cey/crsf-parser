#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr size_t BUFFER_SIZE = 64;
static constexpr uint8_t MIN_PAYLOAD_LEN = 2;
static constexpr uint8_t MAX_PAYLOAD_LEN = BUFFER_SIZE - 2;
static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

namespace
{
    constexpr uint8_t crc8tab[256] = {
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
        0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
        0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
        0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
        0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
        0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
        0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
        0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
        0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
        0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
        0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
        0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
        0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
        0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
        0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
        0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};
}

class CRSFParser
{
private:
    enum class ParserState
    {
        PS_SYNC,
        PS_LENGTH,
        PS_PAYLOAD_AND_CRC
    };

    ParserState m_state = ParserState::PS_SYNC;
    size_t m_buffer_index = 0;
    uint8_t m_buffer[BUFFER_SIZE];
    uint8_t m_expected_length = 0;

    bool parse_byte(uint8_t byte)
    {
        switch (m_state)
        {
        case ParserState::PS_SYNC:
            if (byte == CRSF_SYNC_BYTE)
            {
                m_state = ParserState::PS_LENGTH;
                m_buffer[0] = byte;
                m_buffer_index = 1;
                m_expected_length = 0;
            }
            break;

        case ParserState::PS_LENGTH:
            if (byte >= MIN_PAYLOAD_LEN && byte <= MAX_PAYLOAD_LEN)
            {
                m_state = ParserState::PS_PAYLOAD_AND_CRC;
                m_expected_length = byte;
                m_buffer[1] = byte;
                m_buffer_index = 2;
                break;
            }

            if (byte == CRSF_SYNC_BYTE)
            {
                m_buffer[0] = byte;
                m_buffer_index = 1;
                m_expected_length = 0;
                m_state = ParserState::PS_LENGTH;
                break;
            }

            m_buffer_index = 0;
            m_state = ParserState::PS_SYNC;

            break;

        case ParserState::PS_PAYLOAD_AND_CRC:
            m_buffer[m_buffer_index++] = byte;

            if (m_buffer_index == m_expected_length + 2)
            {
                if (crc_check())
                {
                    m_state = ParserState::PS_SYNC;
                    return true;
                }

                m_state = ParserState::PS_SYNC;

                for (size_t i = 1; i < m_buffer_index; i++)
                {
                    if (m_buffer[i] == CRSF_SYNC_BYTE)
                    {
                        size_t remaining = m_buffer_index - i;

                        if (remaining == 1)
                        {
                            m_buffer[0] = m_buffer[i];
                            m_buffer_index = 1;
                            m_state = ParserState::PS_LENGTH;
                            break;
                        }

                        uint8_t potential_length = m_buffer[i + 1];
                        if (potential_length >= MIN_PAYLOAD_LEN && potential_length <= MAX_PAYLOAD_LEN)
                        {
                            memmove(m_buffer, &m_buffer[i], remaining);
                            m_buffer_index = remaining;
                            m_expected_length = potential_length;
                            m_state = ParserState::PS_PAYLOAD_AND_CRC;
                            break;
                        }
                    }
                }

                if (m_state == ParserState::PS_SYNC)
                {
                    m_buffer_index = 0;
                }
            }
            break;
        }
        return false;
    }

    bool crc_check()
    {
        uint8_t crc = 0;
        size_t CRC_START_BYTE = 2;
        size_t CRC_END_BYTE = m_expected_length + 2 - 1; // +2 because of sync and length bytes, -1 because of crc byte, how can I write it cleaner?

        for (size_t i = CRC_START_BYTE; i < CRC_END_BYTE; i++)
        {
            crc = crc8tab[crc ^ m_buffer[i]];
        }

        return crc == m_buffer[CRC_END_BYTE];
    }
};

extern "C" void app_main(void)
{
}