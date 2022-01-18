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

#ifndef OPL1000_H
#define OPL1000_H

#if DEVICE_SERIAL && DEVICE_INTERRUPTIN &&                                   \
    defined(MBED_CONF_EVENTS_PRESENT) && defined(MBED_CONF_NSAPI_PRESENT) && \
    defined(MBED_CONF_RTOS_API_PRESENT)
#include <stdint.h>

#include <ctime>

#include "PinNames.h"
#include "drivers/BufferedSerial.h"
#include "netsocket/SocketAddress.h"
#include "netsocket/WiFiAccessPoint.h"
#include "netsocket/nsapi_types.h"
#include "platform/ATCmdParser.h"
#include "platform/Callback.h"
#include "platform/mbed_chrono.h"
#include "platform/mbed_error.h"
#include "rtos/Mutex.h"
#include "rtos/ThisThread.h"

// Various timeouts for different OPL1000 operations
// (some of these can't use literal form as they're needed for defaults in this
// header, where we shouldn't add a using directive for them. Defines only used
// in the C++ file can use literals).
#ifndef OPL1000_CONNECT_TIMEOUT
#define OPL1000_CONNECT_TIMEOUT 15s
#endif
#ifndef OPL1000_SEND_TIMEOUT
#define OPL1000_SEND_TIMEOUT 2s
#endif
#ifndef OPL1000_RECV_TIMEOUT
#define OPL1000_RECV_TIMEOUT std::chrono::seconds(5)
#endif
#ifndef OPL1000_MISC_TIMEOUT
#define OPL1000_MISC_TIMEOUT std::chrono::seconds(2)
#endif
#ifndef OPL1000_DNS_TIMEOUT
#define OPL1000_DNS_TIMEOUT 15s
#endif

#define OPL1000_SCAN_TIME_MIN 0ms
#define OPL1000_SCAN_TIME_MAX 1500ms
#define OPL1000_SCAN_TIME_MIN_DEFAULT 120ms
#define OPL1000_SCAN_TIME_MAX_DEFAULT 360ms

// Firmware version
#define OPL1000_SDK_VERSION 2000000
#define OPL1000_SDK_VERSION_MAJOR OPL1000_SDK_VERSION / 1000000

#define OPL1000_AT_VERSION 1000000
#define OPL1000_AT_VERSION_MAJOR OPL1000_AT_VERSION / 1000000
#define EOPL1000_AT_VERSION_TCP_PASSIVE_MODE 1070000
#define OPL1000_AT_VERSION_WIFI_SCAN_CHANGE 1060000

#define FW_AT_LEAST_VERSION(MAJOR, MINOR, PATCH, NUSED /*Not used*/, REF) \
  (((MAJOR)*1000000 + (MINOR)*10000 + (PATCH)*100) >= REF ? true : false)

struct opl1000_socket {
  int id;
  nsapi_protocol_t proto;
  bool connected;
  bool bound;
  SocketAddress addr;
  int keepalive;  // TCP
};

/** OPL1000Interface class.
    This is an interface to a OPL1000 radio.
 */
class OPL1000 {
 public:
  OPL1000(PinName tx, PinName rx, bool debug = false, PinName rts = NC,
          PinName cts = NC);

  /**
   * OPL1000 firmware AT version
   *
   * @param major Major version number
   * @param minor Minor version number
   * @param patch Patch version number
   */
  struct fw_at_version {
    int major;
    int minor;
    fw_at_version(int major, int minor) : major(major), minor(minor) {}
  };

  /**
   * Check AT command interface of OPL1000
   *
   * @return true if ready to respond on AT commands
   */
  bool at_available(void);

  /**
   * Disable echo - required for OOB processing to work
   *
   * @return true if echo was successfully disabled
   */
  bool echo_off(void);

  /**
   * Check sdk version from which firmware is created
   *
   * @return sdk_version which tells sdk version
   */
  int sdk_version(void);

  /**
   * Check AT instruction set version from which firmware is created
   *
   * @return fw_at_version which tells major, minor and patch version
   */
  struct fw_at_version at_version(void);

  /**
   * Startup the OPL1000
   *
   * @return true only if OPL1000 was setup correctly
   */
  bool startup();

  /**
   * Reset OPL1000
   *
   * @return true only if OPL1000 resets successfully
   */
  bool reset(void);

  /**
   * Connect OPL1000 to AP
   *
   * @param ap the name of the AP
   * @param passPhrase the password of AP
   * @return NSAPI_ERROR_OK in success, negative error code in failure
   */
  nsapi_error_t connect(const char *ap, const char *passPhrase);

  /**
   * Disconnect OPL1000 from AP
   *
   * @return true only if OPL1000 is disconnected successfully
   */
  bool disconnect(void);

  /**
   * Enable or disable Remote IP and Port printing with +IPD
   *
   * @param enable, 1 on, 0 off
   * @return true only if OPL1000 is disconnected successfully
   */
  bool ip_info_print(int enable);

  /**
   * Get the IP address of OPL1000
   *
   * @return null-teriminated IP address or null if no IP address is assigned
   */
  const char *ip_addr(void);

  /**
   * Set static IP address, gateway and netmask
   *
   * @param ip IP address to set
   * @param gateway (optional) gateway to set
   * @param netmask (optional) netmask to set
   *
   * @return true if operation was successful and flase otherwise
   */
  bool set_ip_addr(const char *ip, const char *gateway, const char *netmask);

  /**
   * Get the MAC address of OPL1000
   *
   * @return null-terminated MAC address or null if no MAC address is assigned
   */
  const char *mac_addr(void);

  /** Get the local gateway
   *
   *  @return         Null-terminated representation of the local gateway
   *                  or null if no network mask has been recieved
   */
  const char *gateway();

  /** Get the local network mask
   *
   *  @return         Null-terminated representation of the local network mask
   *                  or null if no network mask has been recieved
   */
  const char *netmask();

  /* Return RSSI for active connection
   *
   * @return      Measured RSSI
   */
  int8_t rssi();

  /** Scan mode
   */

  /**Perform a dns query
   *
   * @param name Hostname to resolve
   * @param ip   Buffer to store IP address
   * @return 0 true on success, false on failure
   */
  bool dns_lookup(const char *name, char *ip);

  /**
   * Open a socketed connection
   *
   * @param type the type of socket to open "UDP" or "TCP"
   * @param id id to give the new socket, valid 0-4
   * @param port port to open connection with
   * @param addr the IP address of the destination
   * @param port the port on the destination
   * @param local_port UDP socket's local port, zero means any
   * @param udp_mode UDP socket's mode, zero means can't change remote, 1 can
   * change once, 2 can change multiple times
   * @return NSAPI_ERROR_OK in success, negative error code in failure
   */
  nsapi_error_t open_udp(int id, const char *addr, int port, int local_port = 0,
                         int udp_mode = 0);

  /**
   * Open a socketed connection
   *
   * @param type the type of socket to open "UDP" or "TCP"
   * @param id id to give the new socket, valid 0-4
   * @param port port to open connection with
   * @param addr the IP address of the destination
   * @param port the port on the destination
   * @param tcp_keepalive TCP connection's keep alive time, zero means disabled
   * @return NSAPI_ERROR_OK in success, negative error code in failure
   */
  nsapi_error_t open_tcp(int id, const char *addr, int port, int keepalive = 0);

  /**
   * Sends data to an open socket
   *
   * @param id id of socket to send to
   * @param data data to be sent
   * @param amount amount of data to be sent - max 2048
   * @return number of bytes on success, negative error code in failure
   */
  nsapi_size_or_error_t send(int id, const void *data, uint32_t amount);

  /**
   * Receives datagram from an open UDP socket
   *
   * @param id id to receive from
   * @param data placeholder for returned information
   * @param amount number of bytes to be received
   * @return the number of bytes received
   */
  int32_t recv_udp(
      struct opl1000_socket *socket, void *data, uint32_t amount,
      mbed::chrono::milliseconds_u32 timeout = OPL1000_RECV_TIMEOUT);

  /**
   * Receives stream data from an open TCP socket
   *
   * @param id id to receive from
   * @param data placeholder for returned information
   * @param amount number of bytes to be received
   * @return the number of bytes received
   */
  int32_t recv_tcp(
      int id, void *data, uint32_t amount,
      mbed::chrono::milliseconds_u32 timeout = OPL1000_RECV_TIMEOUT);

  /**
   * Closes a socket
   *
   * @param id id of socket to close, valid only 0-4
   * @return true only if socket is closed successfully
   */
  bool close(int id);

  /**
   * Allows timeout to be changed between commands
   *
   * @param timeout_ms timeout of the connection
   */
  void set_timeout(
      mbed::chrono::milliseconds_u32 timeout = OPL1000_MISC_TIMEOUT);

  /**
   * Checks if data is available
   */
  bool readable();

  /**
   * Checks if data can be written
   */
  bool writeable();

  /**
   * Attach a function to call whenever sigio happens in the serial
   *
   * @param func A pointer to a void function, or 0 to set as none
   */
  void sigio(mbed::Callback<void()> func);

  /**
   * Attach a function to call whenever sigio happens in the serial
   *
   * @param obj pointer to the object to call the member function on
   * @param method pointer to the member function to call
   */
  template <typename T, typename M>
  void sigio(T *obj, M method) {
    sigio(mbed::Callback<void()>(obj, method));
  }

  /**
   * Attach a function to call whenever network state has changed.
   *
   * @param func A pointer to a void function, or 0 to set as none
   */
  void attach(mbed::Callback<void()> status_cb);

  template <typename T, typename M>
  void attach(T *obj, M method) {
    attach(mbed::Callback<void()>(obj, method));
  }

  /** Get the connection status
   *
   *  @return         The connection status according to ConnectionStatusType
   */
  nsapi_connection_status_t connection_status() const;

  /**
   * Start board's and OPL1000's UART flow control
   *
   * @return true if started
   */
  bool start_uart_hw_flow_ctrl();

  /**
   * Stop board's and OPL1000's UART flow control
   *
   * @return true if started
   */
  bool stop_uart_hw_flow_ctrl();

  /**
   * For executing OOB processing on background
   *
   * @param timeout AT parser receive timeout
   * @param if TRUE, process all OOBs instead of only one
   */
  void bg_process_oob(std::chrono::duration<uint32_t, std::milli> timeout,
                      bool all);

  /**
   * Flush the serial port input buffers.
   *
   * If you do HW reset for ESP module, you should
   * flush the input buffers from existing responses
   * from the device.
   */
  void flush();

  static const int8_t WIFIMODE_STATION = 1;
  static const int8_t WIFIMODE_SOFTAP = 2;
  static const int8_t WIFIMODE_STATION_SOFTAP = 3;
  static const int8_t SOCKET_COUNT = 5;

  /**
   * Enables or disables uart input and deep sleep
   *
   * @param lock if TRUE, uart input is enabled and  deep sleep is locked
   * if FALSE, uart input is disabled and  deep sleep is unlocked
   */
  int uart_enable_input(bool lock);

 private:
  // FW version
  int _sdk_v;
  struct fw_at_version _at_v;

  // FW version specific settings and functionalities
  bool _tcp_passive;
  int32_t _recv_tcp_passive(
      int id, void *data, uint32_t amount,
      std::chrono::duration<uint32_t, std::milli> timeout);
  mbed::Callback<void()> _callback;

  // UART settings
  mbed::BufferedSerial _serial;
  PinName _serial_rts;
  PinName _serial_cts;
  rtos::Mutex _smutex;  // Protect serial port access

  // AT Command Parser
  mbed::ATCmdParser _parser;

  // Wifi scan result handling
  bool _recv_ap(nsapi_wifi_ap_t *ap);

  // Socket data buffer
  struct packet {
    struct packet *next;
    int id;
    char remote_ip[16];
    int remote_port;
    uint32_t len;        // Remaining length
    uint32_t alloc_len;  // Original length
                         // data follows
  } * _packets, **_packets_end;
  void _clear_socket_packets(int id);
  void _clear_socket_sending(int id);
  int _sock_active_id;

  // Memory statistics
  size_t _heap_usage;  // (Socket data buffer usage)

  // OOB processing
  void _process_oob(std::chrono::duration<uint32_t, std::milli> timeout,
                    bool all);

  // OOB message handlers
  void _oob_packet_hdlr();
  void _oob_connect_err();
  void _oob_conn_already();
  void _oob_err();
  void _oob_socket0_closed();
  void _oob_socket1_closed();
  void _oob_socket2_closed();
  void _oob_socket3_closed();
  void _oob_socket4_closed();
  void _oob_connection_status();
  void _oob_socket_close_err();
  void _oob_watchdog_reset();
  void _oob_busy();
  void _oob_tcp_data_hdlr();
  void _oob_ready();
  void _oob_scan_results();
  void _oob_send_ok_received();
  void _oob_send_fail_received();

  // OOB state variables
  int _connect_error;
  bool _disconnect;
  bool _fail;
  bool _sock_already;
  bool _closed;
  bool _error;
  bool _busy;
  bool _reset_done;
  int _sock_sending_id;

  // Modem's address info
  char _ip_buffer[16];
  char _gateway_buffer[16];
  char _netmask_buffer[16];
  char _mac_buffer[18];

  // Modem's socket info
  struct _sock_info {
    bool open;
    nsapi_protocol_t proto;
    char *tcp_data;
    int32_t tcp_data_avbl;  // Data waiting on modem
    int32_t tcp_data_rcvd;
    bool send_fail;  // Received 'SEND FAIL'. Expect user will close the socket.
  };
  struct _sock_info _sock_i[SOCKET_COUNT];

  // Scan results
  struct _scan_results {
    WiFiAccessPoint *res;
    unsigned limit;
    unsigned cnt;
  };
  struct _scan_results _scan_r;

  // Connection state reporting
  nsapi_connection_status_t _conn_status;
  mbed::Callback<void()> _conn_stat_cb;  // OPL1000Interface registered
};
#endif
#endif
