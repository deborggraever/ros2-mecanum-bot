
#ifndef __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__
#define __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__

#include <string>
#include <vector>

#define MECANUMBOT_SERIAL_BUFFER_MAX_SIZE           200
#define MECANUMBOT_SERIAL_SERIAL_FRAME_MAX_SIZE     100

namespace debict
{
    namespace mecanumbot
    {
        namespace hardware
        {
            enum class return_type : std::uint8_t
            {
                SUCCESS = 0,
                ERROR = 1
            };

            struct SerialHdlcFrame
            {
                uint8_t data[MECANUMBOT_SERIAL_SERIAL_FRAME_MAX_SIZE];
                size_t length;
            };

            class MecanumbotSerialPort
            {
            public:
                MecanumbotSerialPort();
                ~MecanumbotSerialPort();
                
                return_type open(const std::string & port_name);
                return_type close();
                return_type read_frames(std::vector<SerialHdlcFrame>& frames);
                return_type write_frame(const uint8_t* data, size_t size);
                bool is_open() const;

            protected:
                void encode_byte(uint8_t data);
                void decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames);
                uint16_t crc_update(uint16_t crc, uint8_t data);

            private:
                int serial_port_;
                uint8_t rx_buffer_[MECANUMBOT_SERIAL_BUFFER_MAX_SIZE];
                uint8_t rx_frame_buffer_[MECANUMBOT_SERIAL_SERIAL_FRAME_MAX_SIZE];
                size_t rx_frame_length_;
                uint16_t rx_frame_crc_;
                bool rx_frame_escape_;
                uint8_t tx_frame_buffer_[MECANUMBOT_SERIAL_SERIAL_FRAME_MAX_SIZE];
                size_t tx_frame_length_;
                uint16_t tx_frame_crc_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__
