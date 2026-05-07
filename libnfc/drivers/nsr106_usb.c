/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 */

/**
 * @file nsr106_usb.c
 * @brief Driver for NSR106 NFC readers (wCopy NSR106 / 2SING family)
 *
 * These devices expose a USB HID interface. Each 65-byte HID report wraps
 * a standard PN532 TamaCommand in a proprietary framing. The PN532 chip layer
 * handles all NFC protocol logic; this driver only implements the transport.
 *
 * TX frame layout (65 bytes, zero-padded):
 *   [0]        0x00            HID Report ID
 *   [1]        0x01            Frame type: host→device command
 *   [2]        N+12            Length field (bytes [1]..[N+12] inclusive)
 *   [3]        seq_lo          Sequence counter, low byte
 *   [4]        seq_hi          Sequence counter, high byte
 *   [5]        0xFF            PN532 preamble marker
 *   [6..8]     0x00 0x00 0x00  Reserved padding
 *   [9]        N+1             PN532 content length (D4 + N command bytes)
 *   [10]       0xD4            PN532 TFI (host → device direction)
 *   [11..10+N] cmd + params    PN532 command byte and parameters (N bytes)
 *   [11+N]     checksum        XOR(0xFF, sum(bytes [1..10+N]))
 *   [12+N]     0xFE            End marker
 *
 * RX frame layout:
 *   [0]        0x00            HID Report ID (must be 0)
 *   [1]        0x02            Frame type: device→host response
 *   [2]        len             Length field (bytes [1]..[len] inclusive)
 *   [3]        seq_lo          Echo of request sequence counter
 *   [4]        seq_hi
 *   [5]        0xD5            PN532 TFI (device → host direction)
 *   [6]        cmd+1           PN532 response command (= sent cmd + 1)
 *   [7..len-2] response data   (len-8 bytes)
 *   [len-1]    checksum        XOR(0xFF, sum(bytes [1..len-2]))
 *   [len]      0xFD            End marker
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <inttypes.h>

#include <nfc/nfc.h>

#include "nfc-internal.h"
#include "buses/usbbus.h"
#include "chips/pn53x.h"
#include "chips/pn53x-internal.h"
#include "drivers/nsr106_usb.h"

#define NSR106_USB_DRIVER_NAME "nsr106_usb"

#define LOG_GROUP     NFC_LOG_GROUP_DRIVER
#define LOG_CATEGORY "libnfc.driver.nsr106_usb"

#define USB_INFINITE_TIMEOUT   0
#define USB_TIMEOUT_PER_PASS   200

/* HID report size (including Report ID byte) */
#define NSR106_HID_REPORT_SIZE  65

/*
 * The PN532 max frame data is 264 bytes, but the HID report is only 65 bytes.
 * Usable PN532 payload per frame: 65 - 13 = 52 bytes (cmd + params).
 */
#define NSR106_MAX_PN532_PAYLOAD  52

struct nsr106_supported_device {
  uint16_t vendor_id;
  uint16_t product_id;
  const char *name;
};

static const struct nsr106_supported_device nsr106_supported_devices[] = {
  { 0x0416, 0xB008, "WYD Technology NSR106 (B008)" },
  { 0x0416, 0xB029, "WYD Technology NSR106 (B029)" },
  { 0x0416, 0xB030, "WYD Technology NSR106 (B030)" },
  { 0x0416, 0xB058, "WYD Technology NSR106 (B058)" },
  { 0x2518, 0x6018, "2SING NSR106 (6018)"          },
  { 0x2518, 0x6022, "2SING NSR106 (6022)"          },
};

struct nsr106_data {
  usb_dev_handle *dev;
  uint8_t         ep_in;
  uint8_t         ep_out;
  uint16_t        seq;          /* sequence counter, incremented by 2 per command */
  volatile bool   abort_flag;
};

#define DRIVER_DATA(pnd)  ((struct nsr106_data *)(pnd)->driver_data)

/* ---- endpoint discovery ---- */

static void
nsr106_get_end_points(struct usb_device *dev, struct nsr106_data *data)
{
  if (!dev->config || !dev->config->interface || !dev->config->interface->altsetting)
    return;

  struct usb_interface_descriptor *iface = dev->config->interface->altsetting;

  for (uint32_t i = 0; i < iface->bNumEndpoints; i++) {
    uint8_t addr  = iface->endpoint[i].bEndpointAddress;
    uint8_t attrs = iface->endpoint[i].bmAttributes;

    if (attrs != USB_ENDPOINT_TYPE_INTERRUPT)
      continue;

    if ((addr & USB_ENDPOINT_DIR_MASK) == USB_ENDPOINT_IN)
      data->ep_in = addr;
    else
      data->ep_out = addr;
  }
}

static bool
nsr106_is_supported(uint16_t vid, uint16_t pid)
{
  for (size_t i = 0; i < sizeof(nsr106_supported_devices) / sizeof(nsr106_supported_devices[0]); i++) {
    if (nsr106_supported_devices[i].vendor_id  == vid &&
        nsr106_supported_devices[i].product_id == pid)
      return true;
  }
  return false;
}

static const char *
nsr106_device_name(uint16_t vid, uint16_t pid)
{
  for (size_t i = 0; i < sizeof(nsr106_supported_devices) / sizeof(nsr106_supported_devices[0]); i++) {
    if (nsr106_supported_devices[i].vendor_id  == vid &&
        nsr106_supported_devices[i].product_id == pid)
      return nsr106_supported_devices[i].name;
  }
  return "NSR106 NFC Reader";
}

/* ---- raw HID I/O ---- */

static int
nsr106_hid_write(struct nsr106_data *data, const uint8_t *buf, size_t len, int timeout_ms)
{
  int res = usb_interrupt_write(data->dev, data->ep_out, (char *)buf, (int)len, timeout_ms);
  if (res < 0) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "usb_interrupt_write failed: %s", _usb_strerror(res));
  }
  return res;
}

static int
nsr106_hid_read(struct nsr106_data *data, uint8_t *buf, size_t len, int timeout_ms)
{
  int res = usb_interrupt_read(data->dev, data->ep_in, (char *)buf, (int)len, timeout_ms);
  if (res < 0 && res != -USB_TIMEDOUT) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "usb_interrupt_read failed: %s", _usb_strerror(res));
  }
  return res;
}

/* ---- pn53x_io transport callbacks ---- */

/*
 * Build and send a HID report that wraps a PN532 TamaCommand.
 * pbtData: PN532 command bytes (cmd_code + params), szData bytes total.
 */
static int
nsr106_send(nfc_device *pnd, const uint8_t *pbtData, const size_t szData, const int timeout)
{
  if (szData > NSR106_MAX_PN532_PAYLOAD) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "PN532 payload too large for HID report: %zu > %d", szData, NSR106_MAX_PN532_PAYLOAD);
    pnd->last_error = NFC_EINVARG;
    return pnd->last_error;
  }

  uint8_t report[NSR106_HID_REPORT_SIZE];
  memset(report, 0x00, sizeof(report));

  uint16_t seq  = DRIVER_DATA(pnd)->seq;
  size_t   N    = szData;   /* number of PN532 cmd+params bytes */

  /* Frame header */
  report[0]  = 0x00;              /* HID Report ID                        */
  report[1]  = 0x01;              /* frame type: host→device command      */
  report[2]  = (uint8_t)(N + 12);/* length field                         */
  report[3]  = (uint8_t)(seq & 0xFF);
  report[4]  = (uint8_t)(seq >> 8);

  /* PN532 preamble and length */
  report[5]  = 0xFF;
  report[6]  = 0x00;
  report[7]  = 0x00;
  report[8]  = 0x00;
  report[9]  = (uint8_t)(N + 1); /* D4 + N bytes */
  report[10] = 0xD4;              /* PN532 TFI: host → device             */

  /* PN532 command + params */
  memcpy(&report[11], pbtData, N);

  /* Checksum: XOR(0xFF, sum of bytes [1]..[10+N]) */
  uint8_t cksum = 0;
  for (size_t i = 1; i <= 10 + N; i++)
    cksum += report[i];
  cksum ^= 0xFF;

  report[11 + N] = cksum;
  report[12 + N] = 0xFE; /* end marker */

  /* Advance sequence counter by 2 for next command */
  DRIVER_DATA(pnd)->seq += 2;

  int res = nsr106_hid_write(DRIVER_DATA(pnd), report, sizeof(report), timeout);
  if (res < 0) {
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }
  return NFC_SUCCESS;
}

/*
 * Read a HID report and extract the PN532 response payload.
 * Returns number of bytes written to pbtData (response payload after D5+cmd+1),
 * or a negative NFC_E* error code.
 */
static int
nsr106_receive(nfc_device *pnd, uint8_t *pbtData, const size_t szDataLen, const int timeout)
{
  uint8_t report[NSR106_HID_REPORT_SIZE];
  int res;
  int remaining_time = timeout;

read:
  {
    int usb_timeout;
    if (timeout == USB_INFINITE_TIMEOUT) {
      usb_timeout = USB_TIMEOUT_PER_PASS;
    } else {
      remaining_time -= USB_TIMEOUT_PER_PASS;
      if (remaining_time <= 0) {
        pnd->last_error = NFC_ETIMEOUT;
        return pnd->last_error;
      }
      usb_timeout = (remaining_time < USB_TIMEOUT_PER_PASS) ? remaining_time : USB_TIMEOUT_PER_PASS;
    }

    memset(report, 0, sizeof(report));
    res = nsr106_hid_read(DRIVER_DATA(pnd), report, sizeof(report), usb_timeout);
  }

  if (res == -USB_TIMEDOUT) {
    if (DRIVER_DATA(pnd)->abort_flag) {
      DRIVER_DATA(pnd)->abort_flag = false;
      pnd->last_error = NFC_EOPABORTED;
      return pnd->last_error;
    }
    goto read;
  }

  if (res < 0) {
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  /* Validate frame header */
  if (report[0] != 0x00) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Unexpected HID Report ID: 0x%02X", report[0]);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  if (report[1] != 0x02) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Unexpected frame type: 0x%02X (expected 0x02)", report[1]);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  uint8_t len = report[2];

  if (len <= 5 || len >= NSR106_HID_REPORT_SIZE) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Invalid response frame length: %u", len);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  /* Validate checksum: XOR(0xFF, sum(report[1..len-2])) == report[len-1] */
  uint8_t cksum = 0;
  for (size_t i = 1; i <= (size_t)(len - 2); i++)
    cksum += report[i];
  cksum ^= 0xFF;

  if (cksum != report[len - 1]) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Response checksum mismatch: computed 0x%02X, got 0x%02X",
            cksum, report[len - 1]);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  /* Validate end marker */
  if (report[len] != 0xFD) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Response end marker mismatch: 0x%02X (expected 0xFD)", report[len]);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  /* Validate PN532 TFI and response command code */
  if (report[5] != 0xD5) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "TFI mismatch: 0x%02X (expected 0xD5)", report[5]);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  if (report[6] != (uint8_t)(CHIP_DATA(pnd)->last_command + 1)) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Response command mismatch: 0x%02X (expected 0x%02X)",
            report[6], CHIP_DATA(pnd)->last_command + 1);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  /* Copy response payload (bytes [7..len-2]) */
  size_t payload_len = (size_t)(len - 8);  /* len - 7 - 1 */
  if (payload_len > szDataLen) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "Response payload too large: %zu > %zu", payload_len, szDataLen);
    pnd->last_error = NFC_EIO;
    return pnd->last_error;
  }

  memcpy(pbtData, &report[7], payload_len);
  return (int)payload_len;
}

const struct pn53x_io nsr106_io = {
  .send    = nsr106_send,
  .receive = nsr106_receive,
};

/* ---- libnfc driver callbacks ---- */

static size_t
nsr106_usb_scan(const nfc_context *context, nfc_connstring connstrings[], const size_t connstrings_len)
{
  (void)context;

  usb_prepare();

  size_t device_found = 0;
  struct usb_bus *bus;

  for (bus = usb_get_busses(); bus; bus = bus->next) {
    struct usb_device *dev;
    for (dev = bus->devices; dev; dev = dev->next) {
      if (!nsr106_is_supported(dev->descriptor.idVendor, dev->descriptor.idProduct))
        continue;

      if (!dev->config || !dev->config->interface ||
          !dev->config->interface->altsetting ||
          dev->config->interface->altsetting->bNumEndpoints < 2)
        continue;

      usb_dev_handle *udev = usb_open(dev);
      if (!udev)
        continue;
      usb_close(udev);

      if (snprintf(connstrings[device_found], sizeof(nfc_connstring),
                   "%s:%s:%s", NSR106_USB_DRIVER_NAME,
                   bus->dirname, dev->filename) >= (int)sizeof(nfc_connstring)) {
        /* truncation — skip */
        continue;
      }

      log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG,
              "NSR106 device found: Bus %s Device %s", bus->dirname, dev->filename);

      device_found++;
      if (device_found == connstrings_len)
        return device_found;
    }
  }

  return device_found;
}

struct nsr106_descriptor {
  char *dirname;
  char *filename;
};

static nfc_device *
nsr106_usb_open(const nfc_context *context, const nfc_connstring connstring)
{
  nfc_device *pnd = NULL;
  struct nsr106_descriptor desc = { NULL, NULL };

  int decode_level = connstring_decode(connstring, NSR106_USB_DRIVER_NAME, "usb",
                                       &desc.dirname, &desc.filename);
  if (decode_level < 1)
    goto free_mem;

  usb_prepare();

  struct usb_bus *bus;
  for (bus = usb_get_busses(); bus; bus = bus->next) {
    if (decode_level > 1 && strcmp(bus->dirname, desc.dirname) != 0)
      continue;

    struct usb_device *dev;
    for (dev = bus->devices; dev; dev = dev->next) {
      if (decode_level > 2 && strcmp(dev->filename, desc.filename) != 0)
        continue;

      if (!nsr106_is_supported(dev->descriptor.idVendor, dev->descriptor.idProduct))
        continue;

      usb_dev_handle *udev = usb_open(dev);
      if (!udev)
        continue;

      /* Detach any kernel HID driver so we can claim the interface */
#if !defined(_WIN32)
      usb_detach_kernel_driver_np(udev, 0);
#endif

      int res = usb_set_configuration(udev, 1);
      if (res < 0) {
        log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
                "usb_set_configuration failed: %s", _usb_strerror(res));
        usb_close(udev);
        continue;
      }

      res = usb_claim_interface(udev, 0);
      if (res < 0) {
        log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
                "usb_claim_interface failed: %s", _usb_strerror(res));
        usb_close(udev);
        continue;
      }

      /* Allocate device and driver data */
      pnd = nfc_device_new(context, connstring);
      if (!pnd) {
        usb_release_interface(udev, 0);
        usb_close(udev);
        goto free_mem;
      }

      strncpy(pnd->name,
              nsr106_device_name(dev->descriptor.idVendor, dev->descriptor.idProduct),
              sizeof(pnd->name) - 1);
      pnd->name[sizeof(pnd->name) - 1] = '\0';

      pnd->driver_data = malloc(sizeof(struct nsr106_data));
      if (!pnd->driver_data) {
        usb_release_interface(udev, 0);
        usb_close(udev);
        goto error;
      }

      struct nsr106_data *dd = DRIVER_DATA(pnd);
      memset(dd, 0, sizeof(*dd));
      dd->dev        = udev;
      dd->seq        = 0;
      dd->abort_flag = false;

      /* Discover interrupt endpoints from device descriptor */
      nsr106_get_end_points(dev, dd);
      if (dd->ep_in == 0 || dd->ep_out == 0) {
        /* Fall back to conventional HID endpoint addresses */
        log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG,
                "Endpoint discovery failed, using defaults (IN=0x81, OUT=0x01)");
        dd->ep_in  = 0x81;
        dd->ep_out = 0x01;
      }

      if (pn53x_data_new(pnd, &nsr106_io) == NULL) {
        usb_release_interface(udev, 0);
        usb_close(udev);
        goto error;
      }

      pnd->driver = &nsr106_usb_driver;

      if (pn53x_init(pnd) < 0) {
        usb_release_interface(udev, 0);
        usb_close(udev);
        goto error;
      }

      goto free_mem;
    }
  }
  goto free_mem;

error:
  nfc_device_free(pnd);
  pnd = NULL;

free_mem:
  free(desc.dirname);
  free(desc.filename);
  return pnd;
}

static void
nsr106_usb_close(nfc_device *pnd)
{
  pn53x_idle(pnd);

  int res;
  if ((res = usb_release_interface(DRIVER_DATA(pnd)->dev, 0)) < 0) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "usb_release_interface failed: %s", _usb_strerror(res));
  }

  if ((res = usb_close(DRIVER_DATA(pnd)->dev)) < 0) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR,
            "usb_close failed: %s", _usb_strerror(res));
  }

  pn53x_data_free(pnd);
  nfc_device_free(pnd);
}

static int
nsr106_usb_abort_command(nfc_device *pnd)
{
  DRIVER_DATA(pnd)->abort_flag = true;
  return NFC_SUCCESS;
}

const struct nfc_driver nsr106_usb_driver = {
  .name                             = NSR106_USB_DRIVER_NAME,
  .scan_type                        = NOT_INTRUSIVE,
  .scan                             = nsr106_usb_scan,
  .open                             = nsr106_usb_open,
  .close                            = nsr106_usb_close,
  .strerror                         = pn53x_strerror,

  .initiator_init                   = pn53x_initiator_init,
  .initiator_init_secure_element    = NULL,
  .initiator_select_passive_target  = pn53x_initiator_select_passive_target,
  .initiator_poll_target            = pn53x_initiator_poll_target,
  .initiator_select_dep_target      = pn53x_initiator_select_dep_target,
  .initiator_deselect_target        = pn53x_initiator_deselect_target,
  .initiator_transceive_bytes       = pn53x_initiator_transceive_bytes,
  .initiator_transceive_bits        = pn53x_initiator_transceive_bits,
  .initiator_transceive_bytes_timed = pn53x_initiator_transceive_bytes_timed,
  .initiator_transceive_bits_timed  = pn53x_initiator_transceive_bits_timed,
  .initiator_target_is_present      = pn53x_initiator_target_is_present,

  .target_init           = pn53x_target_init,
  .target_send_bytes     = pn53x_target_send_bytes,
  .target_receive_bytes  = pn53x_target_receive_bytes,
  .target_send_bits      = pn53x_target_send_bits,
  .target_receive_bits   = pn53x_target_receive_bits,

  .device_set_property_bool     = pn53x_set_property_bool,
  .device_set_property_int      = pn53x_set_property_int,
  .get_supported_modulation     = pn53x_get_supported_modulation,
  .get_supported_baud_rate      = pn53x_get_supported_baud_rate,
  .device_get_information_about = pn53x_get_information_about,

  .abort_command = nsr106_usb_abort_command,
  .idle          = pn53x_idle,
  .powerdown     = NULL,
};
