#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class CRSFParser
{
private:
    enum class ParserState
    {
        PS_WAITING_FOR_SYNC,
        PS_WAITING_FOR_LENGTH,
        PS_WAITING_FOR_PAYLOAD_AND_CRC
    };

    ParserState m_state;
    size_t m_buffer_index;
    uint8_t m_buffer[64];
    uint8_t m_expected_length;

    bool parse_byte(uint8_t byte)
    {
        switch (m_state)
        {
        case ParserState::PS_WAITING_FOR_SYNC:
            if (byte == 0xC8)
            {
                m_state = ParserState::PS_WAITING_FOR_LENGTH;
                m_buffer[0] = byte;
                m_buffer_index = 1;
                m_expected_length = 0;
            }
            break;

        case ParserState::PS_WAITING_FOR_LENGTH:
            if (byte <= 62 && byte >= 2)
            {
                m_state = ParserState::PS_WAITING_FOR_PAYLOAD_AND_CRC;
                m_expected_length = byte;
                m_buffer[1] = byte;
                m_buffer_index = 2;
            }
            else
            {
                if (byte == 0xC8)
                {
                    m_state = ParserState::PS_WAITING_FOR_LENGTH;
                    m_buffer[0] = byte;
                    m_buffer_index = 1;
                    m_expected_length = 0;
                    break;
                }
                else
                {
                    m_state = ParserState::PS_WAITING_FOR_SYNC;
                }
            }
            break;

        case ParserState::PS_WAITING_FOR_PAYLOAD_AND_CRC:
            m_buffer[m_buffer_index++] = byte;

            if (m_buffer_index == (m_expected_length + 2)) // add constexpr for min_payload_len, max_payload_len and buffer_size
            {
                if (crc_check() != 1)
                {
                    m_state = ParserState::PS_WAITING_FOR_SYNC;

                    bool found_sync = false;
                    for (size_t i = 1; i < m_buffer_index; i++)
                    {
                        if (m_buffer[i] == 0xC8)
                        {
                            size_t remaining = m_buffer_index - i;
                            memmove(m_buffer, &m_buffer[i], remaining);
                            m_buffer_index = remaining;
                            found_sync == true;

                            if (m_buffer_index == 1)
                            {
                                m_state = ParserState::PS_WAITING_FOR_LENGTH;
                            }
                            else
                            {
                                m_expected_length = m_buffer[1];
                                if (m_expected_length >= 2 && m_expected_length <= 62)
                                {
                                    m_state = ParserState::PS_WAITING_FOR_PAYLOAD_AND_CRC;
                                    break;
                                }
                                else 
                                {
                                    continue;
                                }

                            }
                        }
                    }
                    break;
                }
                else
                {
                    m_state = ParserState::PS_WAITING_FOR_SYNC;
                    return 1;
                }
            }
            break;
        }
        return false;
    }

    bool crc_check();
    bool look_for_sync_byte_in_collected_bytes();
};

extern "C" void app_main(void)
{
}