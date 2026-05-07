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
 * @file nsr106_usb.h
 * @brief Driver for NSR106 NFC readers (wCopy NSR106 / 2SING family)
 *
 * These devices use a USB HID interface that wraps PN532 TamaCommands in a
 * proprietary framing protocol. Supported VID:PID pairs:
 *   0x0416:0xB008, 0x0416:0xB029, 0x0416:0xB030, 0x0416:0xB058
 *   0x2518:0x6018, 0x2518:0x6022
 */

#ifndef __NFC_DRIVER_NSR106_USB_H__
#define __NFC_DRIVER_NSR106_USB_H__

#include <nfc/nfc-types.h>

extern const struct nfc_driver nsr106_usb_driver;

#endif /* __NFC_DRIVER_NSR106_USB_H__ */
