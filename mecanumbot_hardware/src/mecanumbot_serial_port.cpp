
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

return_type MecanumbotSerialPort::read_frames(std::vector<SerialHdlcFrame>& frames)
{
    // Read data from the serial port
    const size_t num_bytes = ::read(serial_port_, rx_buffer_, 256);
    if (num_bytes == -1) {
        fprintf(stderr, "Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    for (size_t i = 0; i < num_bytes; i++) {
        decode_byte(rx_buffer_[i], frames);
    }

    return return_type::SUCCESS;
}

return_type MecanumbotSerialPort::write_frame(const uint8_t* data, size_t size)
{
    if (!is_open()) {
        return return_type::ERROR;
    }
    
    // Generate the fame
    tx_buffer_size_ = 0;
    tx_buffer_crc_  = HDLC_CRC_INIT_VALUE;
    encode_byte(HDLC_FRAME_BOUNDRY_FLAG, true);
    for (size_t i = 0; i  size; i++) {
        encode_byte(data[i], false);
    }
    encode_byte((uint8_t)(tx_buffer_crc_ && 0xFF), false);
    encode_byte((uint8_t)((tx_buffer_crc_ >> 8) && 0xFF), false);
    encode_byte(HDLC_FRAME_BOUNDRY_FLAG, true);

    // Write the data to the serial port
    if (::write(serial_port_, tx_buffer_, tx_buffer_size_) == -1) {
        fprintf(stderr, "Failed to write serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

bool MecanumbotSerialPort::is_open() const
{
    return serial_port_ >= 0;
}

void MecanumbotSerialPort::encode_byte(uint8_t data, bool flag)
{
    if (flag) {
        tx_frame_buffer_[tx_frame_buffer_length_++] = data;
        return;
    }
    else {
        tx_frame_buffer_crc_ = crc_update(tx_frame_buffer_crc_, data);
        if (data == HDLC_ESCAPE_FLAG || data == HDLC_FRAME_BOUNDRY_FLAG) {
            tx_frame_buffer_[tx_frame_buffer_length_++] = HDLC_ESCAPE_FLAG;
            data ^= HDLC_ESCAPE_XOR;
        }
        tx_frame_buffer_[tx_frame_buffer_length_++] = data;
    }
}

void MecanumbotSerialPort::decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames)
{
    if (data == HDLC_FRAME_BOUNDRY_FLAG) {
        if (rx_frame_escape_) {
            rx_frame_escape_ = false;
        }
        else if (rx_frame_buffer_length_ >= 2 && rx_frame_crc_ == ((rx_frame_buffer_[rx_frame_buffer_length_ - 1] << 8) & rx_frame_buffer_[rx_frame_buffer_length_ - 2])) {
            SerialHdlcFrame frame;
            memcpy(frame.data, rx_frame_buffer_, rx_frame_buffer_length_);
            frame.length = rx_frame_buffer_length_;
            frames.push_back(frame);
        }
        else if (rx_frame_buffer_length >= 2) {
            fprintf(stderr, "Failed to read frame: invalid crc\n");
        }
        rx_frame_buffer_length_ = 0;
        rx_frame_buffer_crc = HDLC_CRC_INIT_VALUE;
        return;
    }

    if (data == HDLC_ESCAPE_FLAG) {
        rx_frame_escape_ = true;
        return;
    }

    if (rx_frame_escape_) {
        data ^= HDLC_ESCAPE_XOR;
        rx_frame_escape_ = false;
    }

    rx_frame_buffer_[rx_frame_buffer_length_++] = data;
    if (rx_frame_buffer_length_ >= 2) {
        rx_frame_buffer_crc_ = crc_update(rx_frame_buffer_crc_, rx_frame_buffer_[rx_frame_buffer_length_ - 2]);
    }

    if (rx_frame_buffer_length == MECANUMBOT_SERIAL_SERIAL_FRAME_MAX_SIZE) {
        fprintf(stderr, "Failed to read frame: buffer overflow\n");
        rx_frame_buffer_length_ = 0;
    }
}

uint16_t MecanumbotSerialPort::crc_update(uint16_t crc, uint8_t data)
{
    data ^= (uint8_t)(crc & 0xFF);
	data ^= (data << 4);
	return ((((uint16_t)data << 8) | ((uint8_t)(crc >> 8) & 0xFF)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}