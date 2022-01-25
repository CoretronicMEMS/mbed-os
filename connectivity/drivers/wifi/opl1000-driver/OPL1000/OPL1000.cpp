/* 
 * MIT License
 *
 * Copyright (c) 2022 CoretronicMEMS
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#define MBED_CONF_CMDLINE_ENABLE_INTERNAL_TRACES 1
#if DEVICE_SERIAL && DEVICE_INTERRUPTIN &&                                   \
    defined(MBED_CONF_EVENTS_PRESENT) && defined(MBED_CONF_NSAPI_PRESENT) && \
    defined(MBED_CONF_RTOS_API_PRESENT)
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include "OPL1000.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "PinNames.h"
#include "mbed_trace.h"
#include "netsocket/nsapi_types.h"
#include "platform/Callback.h"
#include "platform/mbed_error.h"
#include "rtos/Kernel.h"

#define TRACE_GROUP "OPLA"  // OPL1000 AT layer

#define OPL1000_ALL_SOCKET_IDS -1

#define OPL1000_DEFAULT_SERIAL_BAUDRATE 115200

using namespace mbed;
using namespace std::chrono;
using std::milli;

OPL1000::OPL1000(PinName tx, PinName rx, bool debug, PinName rts, PinName cts)
    : _sdk_v(-1),
      _at_v(-1, -1),
      _callback(),
      _serial(tx, rx, MBED_CONF_OPL1000_SERIAL_BAUDRATE),
      _serial_rts(rts),
      _serial_cts(cts),
      _parser(&_serial),
      _packets(0),
      _packets_end(&_packets),
      _sock_active_id(-1),
      _heap_usage(0),
      _connect_error(0),
      _disconnect(false),
      _fail(false),
      _sock_already(false),
      _closed(false),
      _error(false),
      _busy(false),
      _reset_done(false),
      _sock_sending_id(-1),
      _conn_status(NSAPI_STATUS_DISCONNECTED) {
  _serial.set_baud(OPL1000_DEFAULT_SERIAL_BAUDRATE);
  _parser.debug_on(debug);
  _parser.set_delimiter("\r\n");
  _parser.oob("+IPD", callback(this, &OPL1000::_oob_packet_hdlr));
  // Note: espressif at command document says that this should be
  // +CWJAP_CUR:<error code> but seems that at least current version is not
  // sending it
  // https://www.espressif.com/sites/default/files/documentation/4a-esp8266_at_instruction_set_en.pdf
  // Also seems that ERROR is not sent, but FAIL instead
  _parser.oob("0,CLOSED", callback(this, &OPL1000::_oob_socket0_closed));
  _parser.oob("1,CLOSED", callback(this, &OPL1000::_oob_socket1_closed));
  _parser.oob("2,CLOSED", callback(this, &OPL1000::_oob_socket2_closed));
  _parser.oob("3,CLOSED", callback(this, &OPL1000::_oob_socket3_closed));
  _parser.oob("4,CLOSED", callback(this, &OPL1000::_oob_socket4_closed));
  _parser.oob("+CWJAP:", callback(this, &OPL1000::_oob_connect_err));
  _parser.oob("WIFI ", callback(this, &OPL1000::_oob_connection_status));
  _parser.oob("UNLINK", callback(this, &OPL1000::_oob_socket_close_err));
  _parser.oob("ALREADY CONNECTED", callback(this, &OPL1000::_oob_conn_already));
  _parser.oob("ERROR", callback(this, &OPL1000::_oob_err));
  _parser.oob("ready", callback(this, &OPL1000::_oob_ready));
  _parser.oob("+CWLAP:", callback(this, &OPL1000::_oob_scan_results));
  // Don't expect to find anything about the watchdog reset in official
  // documentation
  // https://techtutorialsx.com/2017/01/21/esp8266-watchdog-functions/
  _parser.oob("wdt reset", callback(this, &OPL1000::_oob_watchdog_reset));
  // Don't see a reason to make distiction between software(Software WDT reset)
  // and hardware(wdt reset) watchdog treatment
  // https://github.com/esp8266/Arduino/blob/4897e0006b5b0123a2fa31f67b14a3fff65ce561/doc/faq/a02-my-esp-crashes.md#watchdog
  _parser.oob("Soft WDT reset", callback(this, &OPL1000::_oob_watchdog_reset));
  _parser.oob("busy ", callback(this, &OPL1000::_oob_busy));
  // NOTE: documentation v3.0 says '+CIPRECVDATA:<data_len>,' but it's not how
  // the FW responds...
  _parser.oob("+CIPRECVDATA,", callback(this, &OPL1000::_oob_tcp_data_hdlr));
  // Register 'SEND OK'/'SEND FAIL' oobs here. Don't get involved in oob
  // management with send status because ESP8266 modem possibly doesn't reply
  // these packets on error case.
  _parser.oob("SEND OK", callback(this, &OPL1000::_oob_send_ok_received));
  _parser.oob("SEND FAIL", callback(this, &OPL1000::_oob_send_fail_received));

  for (int i = 0; i < SOCKET_COUNT; i++) {
    _sock_i[i].open = false;
    _sock_i[i].proto = NSAPI_UDP;
    _sock_i[i].tcp_data = NULL;
    _sock_i[i].tcp_data_avbl = 0;
    _sock_i[i].tcp_data_rcvd = 0;
    _sock_i[i].send_fail = false;
  }

  _scan_r.res = NULL;
  _scan_r.limit = 0;
  _scan_r.cnt = 0;
}

bool OPL1000::at_available() {
  bool ready = false;

  _smutex.lock();
  // Might take a while to respond after HW reset
  for (int i = 0; i < 5; i++) {
    ready = _parser.send("AT") && _parser.recv("OK\n");
    if (ready) {
      break;
    }
    tr_debug("at_available(): Waiting AT response.");
  }
  // Switch baud-rate from default one to assigned one
  if (MBED_CONF_OPL1000_SERIAL_BAUDRATE != OPL1000_DEFAULT_SERIAL_BAUDRATE) {
    ready &= _parser.send("AT+UART_CUR=%u,8,1,0,0",
                          MBED_CONF_OPL1000_SERIAL_BAUDRATE) &&
             _parser.recv("OK\n");
    _serial.set_baud(MBED_CONF_OPL1000_SERIAL_BAUDRATE);
    ready &= _parser.send("AT") && _parser.recv("OK\n");
  }
  _smutex.unlock();

  return ready;
}

bool OPL1000::echo_off() {
  _smutex.lock();
  bool ready = _parser.send("ATE0") && _parser.recv("OK\n");
  _smutex.unlock();

  return ready;
}

int OPL1000::sdk_version() {
  int version;

  _smutex.lock();
  bool done = _parser.send("AT+GMR") &&
              _parser.recv("SDK version:%d", &version) && _parser.recv("OK\n");
  _smutex.unlock();

  if (!done) {
    return -1;
  }
  return version;
}

struct OPL1000::fw_at_version OPL1000::at_version() {
  int major;
  int minor;

  _smutex.lock();
  bool done = _parser.send("AT+GMR") &&
              _parser.recv("AT version:%d.%d", &major, &minor) &&
              _parser.recv("OK\n");
  _smutex.unlock();

  if (done) {
    _at_v.major = major;
    _at_v.minor = minor;
  }
  return _at_v;
}

bool OPL1000::stop_uart_hw_flow_ctrl(void) {
  bool done = true;
#if DEVICE_SERIAL_FC

  if (_serial_rts != NC || _serial_cts != NC) {
    // Stop board's flow control
    _serial.set_flow_control(SerialBase::Disabled, _serial_rts, _serial_cts);

    // Stop OPL1000's flow control
    done = _parser.send("AT+UART_CUR=%u,8,1,0,0",
                        MBED_CONF_OPL1000_SERIAL_BAUDRATE) &&
           _parser.recv("OK\n");
  }

#endif
  return done;
}

bool OPL1000::start_uart_hw_flow_ctrl(void) {
  bool done = true;

#if DEVICE_SERIAL_FC
  _smutex.lock();
  if (_serial_rts != NC && _serial_cts != NC) {
    // Start OPL1000's flow control
    done = _parser.send("AT+UART_CUR=%u,8,1,0,3",
                        MBED_CONF_OPL1000_SERIAL_BAUDRATE) &&
           _parser.recv("OK\n");

    if (done) {
      // Start board's flow control
      _serial.set_flow_control(SerialBase::RTSCTS, _serial_rts, _serial_cts);
    }

  } else if (_serial_rts != NC) {
    _serial.set_flow_control(SerialBase::RTS, _serial_rts, NC);

    // Enable OPL1000's CTS pin
    done = _parser.send("AT+UART_CUR=%u,8,1,0,2",
                        MBED_CONF_OPL1000_SERIAL_BAUDRATE);
  } else if (_serial_cts != NC) {
    // Enable OPL1000's RTS pin
    done = _parser.send("AT+UART_CUR=%u,8,1,0,1",
                        MBED_CONF_OPL1000_SERIAL_BAUDRATE);

    if (done) {
      _serial.set_flow_control(SerialBase::CTS, NC, _serial_cts);
    }
  }
  _smutex.unlock();

  if (!done) {
    tr_debug("start_uart_hw_flow_ctrl(): Enable UART HW flow control: FAIL.");
  }
#else
  if (_serial_rts != NC || _serial_cts != NC) {
    done = false;
  }
#endif
  return done;
}

bool OPL1000::startup() {
  _smutex.lock();
  set_timeout(OPL1000_CONNECT_TIMEOUT);
  bool done = _parser.send("AT+CWMODE=1\r\n") && _parser.recv("OK\n") &&
              _parser.send("AT+CIPMUX=1") && _parser.recv("OK\n") &&
              _parser.send("AT+CWAUTOCONN=0,3\r\n") && _parser.recv("OK\n");

  set_timeout();  // Restore default
  _smutex.unlock();

  return done;
}

bool OPL1000::reset(void) {
  static const auto OPL1000_BOOTTIME = 10s;
  bool done = false;

  _smutex.lock();

  auto start_time = rtos::Kernel::Clock::now();
  _reset_done = false;
  set_timeout(OPL1000_RECV_TIMEOUT);
  for (int i = 0; i < 2; i++) {
    done = _parser.send("AT+RST") && _parser.recv("OK\n");
    if (!done) {
      continue;
    }
    while (done) {
      if (rtos::Kernel::Clock::now() - start_time >= OPL1000_BOOTTIME) {
        _reset_done = true;
        break;
      }
      rtos::ThisThread::sleep_for(100ms);
    }
    if (_reset_done) {
      break;
    }
  }
  tr_debug("reset(): Done: %s.", done ? "OK" : "FAIL");

  _clear_socket_packets(OPL1000_ALL_SOCKET_IDS);
  _sock_sending_id = -1;
  set_timeout();
  _smutex.unlock();

  return done;
}

nsapi_error_t OPL1000::connect(const char *ap, const char *passPhrase) {
  nsapi_error_t ret = NSAPI_ERROR_OK;
  bool res_2 = false;
  _smutex.lock();
  set_timeout(OPL1000_RECV_TIMEOUT);
  _parser.send("AT+CWQAP") && _parser.recv("OK\n");
  _parser.send("AT+CWLAP") && _parser.recv("OK\n");
  for (int i = 0; i < 3; i++) {
    bool res = _parser.send("AT+CWJAP=\"%s\",\"%s\"\r\n", ap, passPhrase);
    res_2 = res && _parser.recv("OK\n");
    if (!res_2) {
      printf("Retry-connect...\n");
      continue;
    } else {
      break;
    }
  }

  if (!res_2) {
    if (_fail) {
      if (_connect_error == 1) {
        ret = NSAPI_ERROR_CONNECTION_TIMEOUT;
      } else if (_connect_error == 2) {
        ret = NSAPI_ERROR_AUTH_FAILURE;
      } else if (_connect_error == 3) {
        ret = NSAPI_ERROR_NO_SSID;
      } else {
        ret = NSAPI_ERROR_NO_CONNECTION;
      }
      _fail = false;
      _connect_error = 0;
    }
  }

  set_timeout();
  _smutex.unlock();

  return ret;
}

bool OPL1000::disconnect(void) {
  rtos::ThisThread::sleep_for(1000ms);  // OPL1000 need delay 1 sec.
  _smutex.lock();
  _disconnect = true;
  bool done = _parser.send("AT+CWQAP") && _parser.recv("OK\n");
  _smutex.unlock();
  return done;
}

bool OPL1000::ip_info_print(int enable) {
  _smutex.lock();
  _disconnect = true;
  bool done = _parser.send("AT+CIPDINFO=%d", enable) && _parser.recv("OK\n");
  _smutex.unlock();

  return done;
}

const char *OPL1000::ip_addr(void) {
  _smutex.lock();
  set_timeout(OPL1000_CONNECT_TIMEOUT);
  if (!(_parser.send("AT+CIFSR") &&
        _parser.recv("+CIFSR:STAIP,\"%15[^\"]\"", _ip_buffer) &&
        _parser.recv("OK\n"))) {
    _smutex.unlock();
    return 0;
  }
  set_timeout();
  _smutex.unlock();

  return _ip_buffer;
}

bool OPL1000::set_ip_addr(const char *ip, const char *gateway,
                          const char *netmask) {  // Not Supported
  return false;
}

const char *OPL1000::mac_addr(void) {
  _smutex.lock();
  if (!(_parser.send("AT+CIFSR") &&
        _parser.recv("+CIFSR:STAMAC,\"%17[^\"]\"", _mac_buffer) &&
        _parser.recv("OK\n"))) {
    _smutex.unlock();
    return 0;
  }
  _smutex.unlock();

  return _mac_buffer;
}

nsapi_error_t OPL1000::open_udp(int id, const char *addr, int port,
                                int local_port, int udp_mode) {
  static const char *type = "UDP";
  bool done = false;

  ip_info_print(1);

  _smutex.lock();

  // process OOB so that _sock_i reflects the correct state of the socket
  _process_oob(OPL1000_SEND_TIMEOUT, true);

  // Previous close() can fail with busy in sending. Usually, user will ignore
  // the close() error code and cause 'spurious close', in which case user has
  // closed the socket but ESP8266 modem hasn't yet. Because we don't know how
  // long ESP8266 modem will trap in busy, enlarge retry count or timeout in
  // close() isn't a nice way. Here, we actively re-call close() in open() to
  // let the modem close the socket. User can re-try open() on failure.
  // Without this active close(), open() can fail forever with previous
  // 'spurious close', unless peer closes the socket and so ESP8266 modem
  // closes it accordingly.
  if (id >= SOCKET_COUNT) {
    _smutex.unlock();
    return NSAPI_ERROR_PARAMETER;
  } else if (_sock_i[id].open) {
    close(id);
  }

  for (int i = 0; i < 2; i++) {
    if (local_port) {
      done = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d,%d,%d", id, type,
                          addr, port, local_port, udp_mode);
    } else {
      done =
          _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d", id, type, addr, port);
    }

    if (done) {
      if (!_parser.recv("OK\n")) {
        if (_sock_already) {
          _sock_already = false;  // To be raised again by OOB msg
          done = close(id);
          if (!done) {
            break;
          }
        }
        if (_error) {
          _error = false;
          done = false;
        }
        continue;
      }
      _sock_i[id].open = true;
      _sock_i[id].proto = NSAPI_UDP;
      break;
    }
  }
  _clear_socket_packets(id);

  _smutex.unlock();

  tr_debug("open_udp(): UDP socket %d opened: %s.", id,
           (_sock_i[id].open ? "true" : "false"));
  return done ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
}

nsapi_error_t OPL1000::open_tcp(int id, const char *addr, int port,
                                int keepalive) {
  static const char *type = "TCP";
  bool done = false;

  ip_info_print(1);

  if (!addr) {
    return NSAPI_ERROR_PARAMETER;
  }
  _smutex.lock();

  // process OOB so that _sock_i reflects the correct state of the socket
  _process_oob(OPL1000_SEND_TIMEOUT, true);

  // See the reason above with close()
  if (id >= SOCKET_COUNT) {
    _smutex.unlock();
    return NSAPI_ERROR_PARAMETER;
  } else if (_sock_i[id].open) {
    close(id);
  }

  for (int i = 0; i < 2; i++) {
    if (keepalive) {
      done = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d,%d", id, type, addr,
                          port, keepalive);
    } else {
      done =
          _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d", id, type, addr, port);
    }

    if (done) {
      if (!_parser.recv("OK\n")) {
        if (_sock_already) {
          _sock_already = false;  // To be raised again by OOB msg
          done = close(id);
          if (!done) {
            break;
          }
        }
        if (_error) {
          _error = false;
          done = false;
        }
        continue;
      }
      _sock_i[id].open = true;
      _sock_i[id].proto = NSAPI_TCP;
      break;
    }
  }
  _clear_socket_packets(id);

  _smutex.unlock();

  tr_debug("open_tcp: TCP socket %d opened: %s . ", id,
           (_sock_i[id].open ? "true" : "false"));

  return done ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
}

bool OPL1000::dns_lookup(const char *name, char *ip) {
  _smutex.lock();
  set_timeout(OPL1000_DNS_TIMEOUT);
  bool done = _parser.send("AT+CIPDOMAIN=\"%s\"", name) &&
              _parser.recv("+CIPDOMAIN:%15[^\n]\n", ip) && _parser.recv("OK\n");
  set_timeout();
  _smutex.unlock();

  return done;
}

nsapi_size_or_error_t OPL1000::send(int id, const void *data, uint32_t amount) {
  if (_sock_i[id].proto == NSAPI_TCP || _sock_i[id].proto == NSAPI_UDP) {
    if (_sock_sending_id >= 0 && _sock_sending_id < SOCKET_COUNT) {
      if (!_sock_i[id].send_fail) {
        tr_debug(
            "send(): Previous packet (socket %d) was not yet ACK-ed with "
            "SEND "
            "OK.",
            _sock_sending_id);
        return NSAPI_ERROR_WOULD_BLOCK;
      } else {
        tr_debug("send(): Previous packet (socket %d) failed.", id);
        return NSAPI_ERROR_DEVICE_ERROR;
      }
    }
  }

  nsapi_error_t ret = NSAPI_ERROR_DEVICE_ERROR;
  int bytes_confirmed = 0;

  // +CIPSEND supports up to 2048 bytes at a time
  // Data stream can be truncated
  if (amount > 2048 && _sock_i[id].proto == NSAPI_TCP) {
    amount = 2048;
    // Datagram must stay intact
  } else if (amount > 2048 && _sock_i[id].proto == NSAPI_UDP) {
    tr_debug("send(): UDP datagram maximum size is 2048 .");
    return NSAPI_ERROR_PARAMETER;
  }

  _smutex.lock();
  // Mark this socket is sending. We allow only one actively sending socket
  // because:
  // 1. OPL1000 AT packets 'SEND OK'/'SEND FAIL' are not associated with
  // socket ID. No way to tell them.
  // 2. In original implementation, OPL1000::send() is synchronous, which
  // implies only one actively sending socket.
  _sock_sending_id = id;
  set_timeout(OPL1000_SEND_TIMEOUT);
  _busy = false;
  _error = false;
  if (!_parser.send("AT+CIPSEND=%d,%d", id, amount)) {
    tr_debug("send(): AT+CIPSEND failed.");
    goto END;
  }
  // rtos::ThisThread::sleep_for(10);
  if (!_parser.recv(">")) {
    // This means OPL1000 hasn't even started to receive data
    tr_debug("send(): Didn't get \">\"");
    if (_sock_i[id].proto == NSAPI_TCP) {
      ret = NSAPI_ERROR_WOULD_BLOCK;  // Not necessarily critical error.
    } else if (_sock_i[id].proto == NSAPI_UDP) {
      ret = NSAPI_ERROR_NO_MEMORY;
    }
    goto END;
  }
  if (_parser.write((char *)data, (int)amount) < 0) {
    tr_debug("send(): Failed to write serial data");
    // Serial is not working, serious error, reset needed.
    ret = NSAPI_ERROR_DEVICE_ERROR;
    goto END;
  }

  // The "Recv X bytes" is not documented.
  if (!_parser.recv("Recv %d bytes", &bytes_confirmed)) {
    tr_debug("send(): Bytes not confirmed.");
    if (_sock_i[id].proto == NSAPI_TCP) {
      ret = NSAPI_ERROR_WOULD_BLOCK;
    } else if (_sock_i[id].proto == NSAPI_UDP) {
      ret = NSAPI_ERROR_NO_MEMORY;
    }
  } else if (bytes_confirmed != (int)amount && _sock_i[id].proto == NSAPI_UDP) {
    tr_debug("send(): Error: confirmed %d bytes, but expected %d.",
             bytes_confirmed, amount);
    ret = NSAPI_ERROR_DEVICE_ERROR;
  } else {
    // TCP can accept partial writes(if they ever happen)
    ret = bytes_confirmed;
  }

END:
  _process_oob(OPL1000_RECV_TIMEOUT,
               true);  // Drain USART receive register to avoid data overrun

  // error hierarchy, from low to high
  // NOTE: We cannot return NSAPI_ERROR_WOULD_BLOCK when "Recv X bytes" has
  // reached, otherwise duplicate data send.
  if (_busy && ret < 0) {
    ret = NSAPI_ERROR_WOULD_BLOCK;
    tr_debug("send(): Modem busy.");
  }

  if (_error) {
    // FIXME: Not sure clear or not of _error. See it as device error and it
    // can recover only via reset?
    _sock_sending_id = -1;
    ret = NSAPI_ERROR_CONNECTION_LOST;
    tr_debug("send(): Connection disrupted.");
  }

  if (_sock_i[id].send_fail) {
    _sock_sending_id = -1;
    if (_sock_i[id].proto == NSAPI_TCP) {
      ret = NSAPI_ERROR_DEVICE_ERROR;
    } else {
      ret = NSAPI_ERROR_NO_MEMORY;
    }
    tr_debug("send(): SEND FAIL received.");
  }

  if (!_sock_i[id].open && ret < 0) {
    _sock_sending_id = -1;
    ret = NSAPI_ERROR_CONNECTION_LOST;
    tr_debug("send(): Socket %d closed abruptly.", id);
  }

  set_timeout();
  _smutex.unlock();
  return ret;
}

void OPL1000::_oob_packet_hdlr() {
  int id;
  int port;
  int amount;
  int pdu_len;

  // Get socket id
  if (!_parser.scanf(",%d,", &id)) {
    return;
  }

  if (_tcp_passive && _sock_i[id].open == true &&
      _sock_i[id].proto == NSAPI_TCP) {
    // For TCP +IPD return only id and amount and it is independent on
    // AT+CIPDINFO settings Unfortunately no information about that in ESP
    // manual but it has sense.
    if (_parser.recv("%d\n", &amount)) {
      _sock_i[id].tcp_data_avbl = amount;

      // notify data is available
      if (_callback) {
        _callback();
      }
    }
    return;
  } else {
    if (!(_parser.scanf("%d,", &amount) &&
          _parser.scanf("%15[^,],", _ip_buffer) &&
          _parser.scanf("%d:", &port))) {
      return;
    }
  }

  pdu_len = sizeof(struct packet) + amount;

  if ((_heap_usage + pdu_len) > MBED_CONF_OPL1000_SOCKET_BUFSIZE) {
    tr_debug("\"opl1000.socket-bufsize\"-limit exceeded, packet dropped");
    return;
  }

  struct packet *packet = (struct packet *)malloc(pdu_len);
  if (!packet) {
    tr_debug(
        "_oob_packet_hdlr(): Out of memory, unable to allocate memory for "
        "packet.");
    return;
  }
  _heap_usage += pdu_len;

  packet->id = id;
  if (_sock_i[id].proto == NSAPI_UDP) {
    packet->remote_port = port;
    memcpy(packet->remote_ip, _ip_buffer, 16);
  }
  packet->len = amount;
  packet->alloc_len = amount;
  packet->next = 0;

  if (_parser.read((char *)(packet + 1), amount) < amount) {
    free(packet);
    _heap_usage -= pdu_len;
    return;
  }

  // append to packet list
  *_packets_end = packet;
  _packets_end = &packet->next;
}

void OPL1000::_process_oob(duration<uint32_t, milli> timeout, bool all) {
  set_timeout(timeout);
  // Poll for inbound packets
  while (_parser.process_oob() && all) {
  }
  set_timeout();
}

void OPL1000::bg_process_oob(duration<uint32_t, milli> timeout, bool all) {
  _smutex.lock();
  _process_oob(timeout, all);
  _smutex.unlock();
}

int32_t OPL1000::_recv_tcp_passive(int id, void *data, uint32_t amount,
                                   duration<uint32_t, milli> timeout) {
  int32_t ret = NSAPI_ERROR_WOULD_BLOCK;

  _smutex.lock();

  _process_oob(timeout, true);

  if (_sock_i[id].tcp_data_avbl != 0) {
    _sock_i[id].tcp_data = (char *)data;
    _sock_i[id].tcp_data_rcvd = NSAPI_ERROR_WOULD_BLOCK;
    _sock_active_id = id;

    // +CIPRECVDATA supports up to 2048 bytes at a time
    amount = amount > 2048 ? 2048 : amount;

    // NOTE: documentation v3.0 says '+CIPRECVDATA:<data_len>,' but it's not
    // how the FW responds...
    bool done = _parser.send("AT+CIPRECVDATA=%d,%" PRIu32, id, amount) &&
                _parser.recv("OK\n");

    _sock_i[id].tcp_data = NULL;
    _sock_active_id = -1;

    if (!done) {
      goto BUSY;
    }

    // update internal variable tcp_data_avbl to reflect the remaining data
    if (_sock_i[id].tcp_data_rcvd > 0) {
      if (_sock_i[id].tcp_data_rcvd > (int32_t)amount) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_EBADMSG),
                   "OPL1000::_recv_tcp_passive() too much data from modem\n");
      }
      if (_sock_i[id].tcp_data_avbl > _sock_i[id].tcp_data_rcvd) {
        _sock_i[id].tcp_data_avbl -= _sock_i[id].tcp_data_rcvd;
      } else {
        _sock_i[id].tcp_data_avbl = 0;
      }
    }

    ret = _sock_i[id].tcp_data_rcvd;
  }

  if (!_sock_i[id].open && ret == NSAPI_ERROR_WOULD_BLOCK) {
    ret = 0;
  }

  _smutex.unlock();
  return ret;

BUSY:
  _process_oob(OPL1000_RECV_TIMEOUT, true);
  if (_busy) {
    tr_debug("_recv_tcp_passive(): Modem busy.");
    ret = NSAPI_ERROR_WOULD_BLOCK;
  } else {
    tr_error("_recv_tcp_passive(): Unknown state.");
    ret = NSAPI_ERROR_DEVICE_ERROR;
  }
  _smutex.unlock();
  return ret;
}

int32_t OPL1000::recv_tcp(int id, void *data, uint32_t amount,
                          duration<uint32_t, milli> timeout) {
  if (_tcp_passive) {
    return _recv_tcp_passive(id, data, amount, timeout);
  }

  _smutex.lock();

  // No flow control, drain the USART receive register ASAP to avoid data
  // overrun
  if (_serial_rts == NC) {
    _process_oob(timeout, true);
  }

  // check if any packets are ready for us
  for (struct packet **p = &_packets; *p; p = &(*p)->next) {
    if ((*p)->id == id) {
      struct packet *q = *p;

      if (q->len <= amount) {  // Return and remove full packet
        memcpy(data, q + 1, q->len);

        if (_packets_end == &(*p)->next) {
          _packets_end = p;
        }
        *p = (*p)->next;

        _smutex.unlock();

        uint32_t pdu_len = sizeof(struct packet) + q->alloc_len;
        uint32_t len = q->len;
        free(q);
        _heap_usage -= pdu_len;
        return len;
      } else {  // return only partial packet
        memcpy(data, q + 1, amount);

        q->len -= amount;
        memmove(q + 1, (uint8_t *)(q + 1) + amount, q->len);

        _smutex.unlock();
        return amount;
      }
    }
  }
  if (!_sock_i[id].open) {
    _smutex.unlock();
    return 0;
  }

  // Flow control, read from USART receive register only when no more data is
  // buffered, and as little as possible
  if (_serial_rts != NC) {
    _process_oob(timeout, false);
  }
  _smutex.unlock();

  return NSAPI_ERROR_WOULD_BLOCK;
}

int32_t OPL1000::recv_udp(struct opl1000_socket *socket, void *data,
                          uint32_t amount, duration<uint32_t, milli> timeout) {
  _smutex.lock();
  set_timeout(timeout);

  // Process OOB data since this is
  // how UDP packets are received
  _process_oob(timeout, true);

  set_timeout();

  // check if any packets are ready for us
  for (struct packet **p = &_packets; *p; p = &(*p)->next) {
    if ((*p)->id == socket->id) {
      struct packet *q = *p;

      socket->addr.set_ip_address((*p)->remote_ip);
      socket->addr.set_port((*p)->remote_port);

      // Return and remove packet (truncated if necessary)
      uint32_t len = q->len < amount ? q->len : amount;
      memcpy(data, q + 1, len);

      if (_packets_end == &(*p)->next) {
        _packets_end = p;
      }
      *p = (*p)->next;
      _smutex.unlock();

      uint32_t pdu_len = sizeof(struct packet) + q->alloc_len;
      free(q);
      _heap_usage -= pdu_len;
      return len;
    }
  }

  // Flow control, read from USART receive register only when no more data is
  // buffered, and as little as possible
  if (_serial_rts != NC) {
    _process_oob(timeout, false);
  }

  _smutex.unlock();

  return NSAPI_ERROR_WOULD_BLOCK;
}

void OPL1000::_clear_socket_packets(int id) {
  struct packet **p = &_packets;

  while (*p) {
    if ((*p)->id == id || id == OPL1000_ALL_SOCKET_IDS) {
      struct packet *q = *p;
      int pdu_len = sizeof(struct packet) + q->alloc_len;

      if (_packets_end == &(*p)->next) {
        _packets_end = p;  // Set last packet next field/_packets
      }
      *p = (*p)->next;
      free(q);
      _heap_usage -= pdu_len;
    } else {
      // Point to last packet next field
      p = &(*p)->next;
    }
  }
  if (id == OPL1000_ALL_SOCKET_IDS) {
    for (int id = 0; id < 5; id++) {
      _sock_i[id].tcp_data_avbl = 0;
    }
  } else {
    _sock_i[id].tcp_data_avbl = 0;
  }
}

void OPL1000::_clear_socket_sending(int id) {
  if (id == _sock_sending_id) {
    _sock_sending_id = -1;
  }
  _sock_i[id].send_fail = false;
}

bool OPL1000::close(int id) {
  // May take a second try if device is busy
  for (unsigned i = 0; i < 2; i++) {
    _smutex.lock();
    if (_parser.send("AT+CIPCLOSE=%d", id)) {
      if (!_parser.recv("OK\n")) {
        if (_closed) {  // UNLINK ERROR
          _closed = false;
          _sock_i[id].open = false;
          _clear_socket_packets(id);
          // Closed, so this socket escapes from SEND FAIL status.
          _clear_socket_sending(id);
          _smutex.unlock();
          // OPL1000 has a habit that it might close a socket on its own.
          tr_debug("close(%d): socket close OK with UNLINK ERROR", id);
          return true;
        }
      } else {
        // _sock_i[id].open set to false with an OOB
        _clear_socket_packets(id);
        // Closed, so this socket escapes from SEND FAIL status
        _clear_socket_sending(id);
        _smutex.unlock();
        tr_debug("close(%d): socket close OK with AT+CIPCLOSE OK", id);
        return true;
      }
    }
    _smutex.unlock();
  }
  tr_debug("close(%d): socket close FAIL'ed (spurious close)", id);

  return false;
}

void OPL1000::set_timeout(duration<uint32_t, milli> timeout) {
  _parser.set_timeout(timeout.count());
}

bool OPL1000::readable() { return _serial.FileHandle::readable(); }

bool OPL1000::writeable() { return _serial.FileHandle::writable(); }

void OPL1000::sigio(Callback<void()> func) {
  _serial.sigio(func);
  _callback = func;
}

void OPL1000::attach(Callback<void()> status_cb) { _conn_stat_cb = status_cb; }

bool OPL1000::_recv_ap(nsapi_wifi_ap_t *ap) { return false; }  // Not Supported

void OPL1000::_oob_watchdog_reset() {
  MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ETIME),
             "_oob_watchdog_reset() modem watchdog reset triggered\n");
}

void OPL1000::_oob_ready() {
  _reset_done = true;

  for (int i = 0; i < SOCKET_COUNT; i++) {
    _sock_i[i].open = false;
  }

  // Makes possible to reinitialize
  _conn_status = NSAPI_STATUS_ERROR_UNSUPPORTED;
  _conn_stat_cb();

  tr_debug("_oob_reset(): Reset detected.");
}

void OPL1000::_oob_busy() {
  char status;
  if (_parser.scanf("%c...\n", &status)) {
    if (status == 's') {
      tr_debug("_oob_busy(): Busy s...");
    } else if (status == 'p') {
      tr_debug("_oob_busy(): Busy p...");
    } else {
      tr_error("_oob_busy(): unrecognized busy state '%c...'", status);
      MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_EBADMSG),
                 "OPL1000::_oob_busy() unrecognized busy state\n");
    }
  } else {
    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMSG),
               "OPL1000::_oob_busy() AT timeout\n");
  }
  _busy = true;
}

void OPL1000::_oob_tcp_data_hdlr() {
  int32_t len;

  MBED_ASSERT(_sock_active_id >= 0 && _sock_active_id < 5);

  if (!_parser.scanf("%" SCNd32 ":", &len)) {
    return;
  }

  if (_parser.read(_sock_i[_sock_active_id].tcp_data, len) == -1) {
    return;
  }

  _sock_i[_sock_active_id].tcp_data_rcvd = len;
}

void OPL1000::_oob_scan_results() {
  nsapi_wifi_ap_t ap;

  if (_recv_ap(&ap)) {
    if (_scan_r.res && _scan_r.cnt < _scan_r.limit) {
      _scan_r.res[_scan_r.cnt] = WiFiAccessPoint(ap);
    }

    _scan_r.cnt++;
  }
}

void OPL1000::_oob_connect_err() {
  _fail = false;
  _connect_error = 0;

  if (_parser.scanf("%d", &_connect_error) && _parser.scanf("FAIL")) {
    _fail = true;
    _parser.abort();
  }
}

void OPL1000::_oob_conn_already() {
  _sock_already = true;
  _parser.abort();
}

void OPL1000::_oob_err() {
  _error = true;
  _parser.abort();
}

void OPL1000::_oob_socket_close_err() {
  if (_error) {
    _error = false;
  }
  _closed = true;  // Not possible to pinpoint to a certain socket
}

void OPL1000::_oob_socket0_closed() {
  static const int id = 0;
  _sock_i[id].open = false;
  // Closed, so this socket escapes from SEND FAIL status
  _clear_socket_sending(id);
  tr_debug("_oob_socket0_closed(): Socket %d closed.", id);
}

void OPL1000::_oob_socket1_closed() {
  static const int id = 1;
  _sock_i[id].open = false;
  // Closed, so this socket escapes from SEND FAIL status
  _clear_socket_sending(id);
  tr_debug("_oob_socket1_closed(): Socket %d closed.", id);
}

void OPL1000::_oob_socket2_closed() {
  static const int id = 2;
  _sock_i[id].open = false;
  _clear_socket_sending(id);
  tr_debug("_oob_socket2_closed(): Socket %d closed.", id);
}

void OPL1000::_oob_socket3_closed() {
  static const int id = 3;
  _sock_i[id].open = false;
  _clear_socket_sending(id);
  tr_debug("_oob_socket3_closed(): %d closed.", id);
}

void OPL1000::_oob_socket4_closed() {
  static const int id = 4;
  _sock_i[id].open = false;
  // Closed, so this socket escapes from SEND FAIL status
  _clear_socket_sending(id);
  tr_debug("_oob_socket0_closed(): Socket %d closed.", id);
}

void OPL1000::_oob_connection_status() {
  char status[13];
  if (_parser.recv("%12[^\"]\n", status)) {
    if (strcmp(status, "GOT IP\n") == 0) {
      _conn_status = NSAPI_STATUS_GLOBAL_UP;
    } else if (strcmp(status, "DISCONNECT\n") == 0) {
      if (_disconnect) {
        _conn_status = NSAPI_STATUS_DISCONNECTED;
        _disconnect = false;
      } else {
        _conn_status = NSAPI_STATUS_CONNECTING;
      }
    } else if (strcmp(status, "CONNECTED\n") == 0) {
      _conn_status = NSAPI_STATUS_CONNECTING;
    } else {
      tr_error("_oob_connection_status(): Invalid AT cmd \'%s\' .", status);
      MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_EBADMSG),
                 "OPL1000::_oob_connection_status: invalid AT cmd\n");
    }
  } else {
    tr_error(
        "_oob_connection_status(): Network status timeout, disconnecting.");
    if (!disconnect()) {
      tr_warning(
          "_oob_connection_status(): Driver initiated disconnect failed.");
    } else {
      tr_debug("_oob_connection_status(): Disconnected.");
    }
    _conn_status = NSAPI_STATUS_ERROR_UNSUPPORTED;
  }

  MBED_ASSERT(_conn_stat_cb);
  _conn_stat_cb();
}

void OPL1000::_oob_send_ok_received() {
  tr_debug("_oob_send_ok_received called for socket %d", _sock_sending_id);
  _sock_sending_id = -1;
}

void OPL1000::_oob_send_fail_received() {
  tr_debug("_oob_send_fail_received called for socket %d", _sock_sending_id);
  if (_sock_sending_id >= 0 && _sock_sending_id < SOCKET_COUNT) {
    _sock_i[_sock_sending_id].send_fail = true;
  }
  _sock_sending_id = -1;
}

void OPL1000::flush() {
  _smutex.lock();
  _parser.flush();
  _smutex.unlock();
}

nsapi_connection_status_t OPL1000::connection_status() const {
  return _conn_status;
}

int OPL1000::uart_enable_input(bool enabled) {
  return _serial.enable_input(enabled);
}

#endif
