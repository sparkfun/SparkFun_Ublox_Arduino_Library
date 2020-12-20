#ifndef Wire_h
#define Wire_h

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "Common.h"

class TwoWire
{
public:
  TwoWire() : m_address(0x42), i2c_fd(0) { init(); }
  ~TwoWire() { close(i2c_fd); }

  bool init() {
    char filename[20] = I2C_DEV;
    //snprintf(filename, sizeof(filename), "/dev/i2c-%u", bus_);

    if ((i2c_fd = open(filename, O_RDWR)) < 0) {
      printf("I2C: Failed to open bus '%s': %i-%s",
                filename, errno, strerror(errno));
      return false;
    }

    printf("I2C: Successfully initialized with default address [%s 0x%02x]", filename, m_address);
    return true;
  }

  bool beginTransmission(uint8_t address) { 
    m_address = address;
    if (ioctl(i2c_fd, I2C_SLAVE, m_address) < 0) {
      printf("I2C: Failed to acquire bus access and/or talk to slave: %i-%s",
                errno, strerror(errno));
      return false;
    }
  }
  uint8_t endTransmission(void) { return 0; }
  uint8_t endTransmission(bool stop) { return 0; }

  bool available() {
    return (read(m_recvdByte));
  }

  size_t write(uint8_t byte) {
    m_IsAvailable = false;
    return ::write(i2c_fd, (void *)&byte, sizeof(byte));
  }

  size_t write(uint8_t *byte, size_t num_of_bytes) {
    m_IsAvailable = false;
    return ::write(i2c_fd, byte, num_of_bytes);
  }

  uint8_t read() {
    return m_recvdByte;
  }

  uint8_t requestFrom(uint8_t address, uint8_t quantity) {
    m_address = address;
    m_quantity = quantity;
    return (read(quantity));
  }

private:
  uint8_t m_address;
  bool m_IsAvailable;
  bool m_IsConnected;
  uint8_t m_recvdByte;
  uint8_t m_quantity;
  int i2c_fd;

  size_t read(uint8_t byte) {
    int iRxLen = 0;
    iRxLen = ::read(i2c_fd, (void *)&byte, sizeof(byte));
    if (iRxLen > 0) {
      m_IsAvailable = true;
    } else {
      m_IsAvailable = false;
    }
    return m_IsAvailable;
  }
};

extern TwoWire Wire;

#endif //Wire_h