
#ifndef __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__
#define __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__

#include <string>
#include <vector>

#define SERIAL_BUFFER_MAX_SIZE      1024
#define SERIAL_FRAME_MAX_SIZE       100

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
                uint8_t data[100];
                size_t length;
            };

            class MecanumbotSerialPort
            {
            public:
                MecanumbotSerialPort();
                ~MecanumbotSerialPort();
                
                return_type open(const std::string & port_name);
                return_type close();
                return_type read(std::vector<SerialHdlcFrame>& frames);
                return_type write(const uint8_t* data, size_t size);
                bool is_open() const;

            private:
                int serial_port_;
                uint8_t buffer_data_[SERIAL_BUFFER_MAX_SIZE];
                size_t buffer_size_;
                size_t buffer_offset_;
                size_t frame_offset_;
                uint16_t frame_crc_;
                bool is_escape_byte_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__
