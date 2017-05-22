#include "roboclaw.h"

#define UART_ERROR(data) (data >> 8)

uint16_t crc;

void update_crc(uint8_t data) {
  crc = crc ^ (((uint16_t) data) << 8);
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
}

uint16_t crc16(char *packet, uint8_t nBytes) {
  uint16_t _crc = 0;
  for (uint8_t byte = 0; byte < nBytes; byte++) {
    _crc = _crc ^ (((uint16_t) packet[byte]) << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (_crc & 0x8000)
        _crc = (_crc << 1) ^ 0x1021;
      else
        _crc <<= 1;
    }
  }
  return _crc;
}

void rc_init(void) {
  uint16_t baudrate = UART_BAUD_SELECT(BAUD, F_CPU);
  uart1_init(baudrate);
}

bool check_crc() {
  uint16_t crcc = 0;
  uint16_t data = uart1_getc();
  if (! UART_ERROR(data)) {
    crcc = (int16_t)data << 8;
    data = uart1_getc();
    if (! UART_ERROR(data)) {
      crcc |= (data & 0xFF);
      return (crc == crcc);
    }
  }
  return false;
}

void send_cmd(uint8_t cmd) {
  crc = 0;
  uart1_putc(RC_ADDRESS);
  uart1_putc(cmd);
  // uart_send(RC_ADDRESS);
  // uart_send(cmd);
  update_crc(RC_ADDRESS);
  update_crc(cmd);
}

bool write_n(uint8_t cnt, ...) {
  uint16_t tcrc, rcrc;

  va_list args; // Variable list
  va_start(args, cnt); // Initialise variable arguments

  char packet[cnt + 1] = {RC_ADDRESS};
  for (uint8_t i = 1; i <= cnt; i++) {
    packet[i] = va_arg(args, uint8_t);
  }

  uart1_puts(packet);

  tcrc = crc16(packet, cnt + 1);

  uart1_putc(tcrc >> 8);
  uart1_putc(tcrc & 0xFF);

  rcrc = uart1_getc();
  if (!UART_ERROR(rcrc) && (rcrc & 0xFF) == 0xFF)
    return true;

  return false;

  // uart_send(RC_ADDRESS);
  // update_crc(RC_ADDRESS);

  // while (cnt--) {
  //   uint8_t data = va_arg(args, int);
  //   uart_send(data);
  //   update_crc(data);
  // }



  // uart_send(crc >> 8);
  // uart_send(crc & 0xFF);

  // if (uart_recv(TIMEOUT) == 0xFF)
  //   return true;

  // return false;
}

template <typename T> bool read_n(uint8_t cnt, uint8_t cmd, ...) {
  bool valid = false;
  uint8_t data = 0;

  uart1_flush();

  send_cmd(cmd);

  va_list args;
  va_start(args, cmd);

  while (cnt--) {
    T *ptr = va_arg(args, T*);
    *ptr = read<T>(&valid);

    if (!valid)
      break;
  }

  uint16_t _crc = (uart1_getc() << 8) | (uart1_getc() & 0xFF);

  if (valid)
    return _crc == crc;
  else
    return false;
}


template <typename T> T read(bool *valid) {
  T value = 0;
  uint8_t bytes = (sizeof(T) / sizeof(uint8_t));
  uint16_t data = 0;

  uart1_flush();

  while (bytes--) {
    data = uart1_getc();
    if (!UART_ERROR(data)) {
      *valid = true;
      update_crc(data);
      value |= ((T) data) << ((bytes - 1) * 8);
    } else {
      *valid = false;
      break;
    }

  }
}


bool ReadCurrents(int16_t &current1, int16_t &current2) {
  bool valid = read_n<int16_t>(2, RC_GETCURRENTS, current1, current2);
  return valid;
}


// bool ReadVersion(char *version){
//   uint8_t data = 0;
//   crc = 0;

//   send_cmd(RC_GETVERSION);

//   for (uint8_t i = 0; i < 48; i++) {
//     data = uart_recv(TIMEOUT);
//     version[i] = data;
//     update_crc(version[i]);
//     if (version[i] == 0)
//       break;
//   }
//   return check_crc();
// }