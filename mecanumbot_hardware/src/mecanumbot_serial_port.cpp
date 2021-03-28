
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>


#include "mecanumbot_hardware/mecanumbot_serial_port.hpp"

#define HDLC_FRAME_BOUNDRY_FLAG     0x7E
#define HDLC_ESCAPE_FLAG            0x7D
#define HDLC_ESCAPE_XOR             0x20
#define HDLC_CRC_INIT_VALUE         0xFFFF

using namespace debict::mecanumbot::hardware;

MecanumbotSerialPort::MecanumbotSerialPort()
    : serial_port_(-1)
    , buffer_size_(0)
    , buffer_offset_(0)
    , frame_offset_(0)
    , frame_crc_(0)
    , is_escape_byte_(false)
{

}

MecanumbotSerialPort::~MecanumbotSerialPort()
{
    close();
}

return_type MecanumbotSerialPort::open(const std::string & port_name)
{
    serial_port_ = ::open(port_name.c_str(), O_RDWR);

    if (serial_port_ < 0) {
        fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    struct termios tty_config{};
    if (::tcgetattr(serial_port_, &tty_config) != 0) {
        fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    tty_config.c_iflag &= ~(INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
    tty_config.c_iflag |= IGNBRK | IGNPAR;
    tty_config.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL | NLDLY | VTDLY);
    tty_config.c_oflag |= NL0 | VT0;
    tty_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
    tty_config.c_cflag |= CS8 | CREAD | CLOCAL;
    tty_config.c_lflag &= ~(ISIG | ICANON | ECHO | TOSTOP | IEXTEN);

    if (::cfsetispeed(&tty_config, B9600) != 0 || ::cfsetospeed(&tty_config, B9600) != 0) {
        fprintf(stderr, "Failed to set serial port speed: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
        fprintf(stderr, "Failed to set serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

return_type MecanumbotSerialPort::close()
{
    if (is_open()) {
        ::close(serial_port_);
        serial_port_ = -1;
    }
    return return_type::SUCCESS;
}

return_type MecanumbotSerialPort::read(std::vector<SerialHdlcFrame>& frames)
{
    // Read data from the serial port
    uint8_t buffer[256];
    const auto num_bytes = ::read(serial_port_, buffer, 256);
    if (num_bytes == -1) {
        fprintf(stderr, "Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    // Check if we can store this data
    auto bytes_available = (SERIAL_BUFFER_MAX_SIZE - (buffer_offset_ + buffer_size_));
    if (bytes_available < num_bytes) {
        // Discard all data
        fprintf(stderr, "Serial port read buffer overflow: Flushing data\n");
        buffer_offset_ = 0;
        buffer_size_ = 0;
    }

    // Store the read data
    memcpy(&buffer_data_[buffer_offset_], buffer, num_bytes);
    buffer_offset_ = buffer_size_;
    buffer_size_ += num_bytes;

    // Process the frames
    uint8_t frame_buffer[100];
    size_t frame_position = 0;
    for (size_t i = 0; i < buffer_size_; i++) {
        uint8_t frame_data = buffer_data_[i];
        if (frame_data == HDLC_FRAME_BOUNDRY_FLAG) {
            if (is_escape_byte_) {
                is_escape_byte_ = false;
            }
            else if (i >= 2) {
                uint16_t crc_value = ((uint16_t)buffer_data_[i - 2] << 8) | (uint16_t)buffer_data_[i - 1];
                if (crc_value == frame_crc_) {
                    SerialHdlcFrame frame;
                    frame.length = i - frame_offset_ - 2;
                    memcpy(frame.data, &buffer_data_[frame_offset_], frame.length);
                    frames.push_back(frame);
                }
                else {
                    fprintf(stderr, "Serial port frame incorrect frame crc: Discard data\n");
                }
            }
            frame_offset_ = i;
            frame_crc_ = HDLC_FRAME_BOUNDRY_FLAG;
            continue;
        }

        if (is_escape_byte_) {
            is_escape_byte_ = false;
            frame_data ^= HDLC_ESCAPE_XOR;
        }
        else if (frame_data == HDLC_ESCAPE_FLAG) {
            is_escape_byte_ = true;
            continue;
        }

        frame_buffer[frame_position] = frame_data;
        if (i - frame_position >= 2) {
            uint8_t crc_data = frame_buffer[frame_position - 2];
            crc_data ^= (uint8_t)(frame_crc_ & 0xFF);
            crc_data ^= (crc_data << 4);
            crc_value = ((((uint16_t)crc_data << 8) | ((uint8_t)(crc_value >> 8) & 0xFF)) ^ (uint8_t)(crc_data >> 4) ^ ((uint16_t)crc_data << 3));

        }
        frame_position++;
    }

    // Copy unprocessed data or reset buffer offset
}

return_type MecanumbotSerialPort::write(const uint8_t* data, size_t size)
{
    if (!is_open()) {
        return return_type::ERROR;
    }

    size_t buffer_offset = 0;
    uint8_t buffer[1024];

    // Set the frame header
    buffer[buffer_offset++] = HDLC_FRAME_BOUNDRY_FLAG;

    // Calculate the checksum and add the bytes to the buffer
    uint16_t crc_value = 0xFFFF;
    for (size_t i = 0; i < size; i++) {
        uint8_t crc_data = data[i];
        crc_data ^= (uint8_t)(crc_value & 0xFF);
        crc_data ^= (crc_data << 4);
        crc_value = ((((uint16_t)crc_data << 8) | ((uint8_t)(crc_value >> 8) & 0xFF)) ^ (uint8_t)(crc_data >> 4) ^ ((uint16_t)crc_data << 3));

        if ((crc_data == HDLC_FRAME_BOUNDRY_FLAG || crc_data == HDLC_ESCAPE_FLAG)) {
            buffer[buffer_offset++] = HDLC_ESCAPE_FLAG;
            crc_data ^= HDLC_ESCAPE_XOR;
        }
        buffer[buffer_offset++] = crc_data;
    }

    // Set the checksum low byte
    uint8_t crc_lb = (uint8_t)(crc_value & 0xFF);
    if ((crc_lb == HDLC_FRAME_BOUNDRY_FLAG || crc_lb == HDLC_ESCAPE_FLAG)) {
        buffer[buffer_offset++] = HDLC_ESCAPE_FLAG;
        crc_lb ^= HDLC_ESCAPE_XOR;
    }
    buffer[buffer_offset++] = crc_lb;

    // Set the checksum high byte
    uint8_t crc_hb = (uint8_t)((crc_value >> 8) && 0xFF);
    if ((crc_hb == HDLC_FRAME_BOUNDRY_FLAG || crc_hb == HDLC_ESCAPE_FLAG)) {
        buffer[buffer_offset++] = HDLC_ESCAPE_FLAG;
        crc_hb ^= HDLC_ESCAPE_XOR;
    }
    buffer[buffer_offset++] = crc_hb;

    // Set the frame footer
    buffer[buffer_offset++] = HDLC_FRAME_BOUNDRY_FLAG;    

    if (::write(serial_port_, buffer, buffer_offset) == -1) {
        fprintf(stderr, "Failed to write serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

bool MecanumbotSerialPort::is_open() const
{
    return serial_port_ >= 0;
}
