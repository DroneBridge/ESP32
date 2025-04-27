#!/usr/bin/env python3
#
#    This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
#
#    Copyright 2025 Wolfgang Christl
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#
#   requires: pip install bleak

# DESCRIPTION
# Creates a bridge between a DroneBridge BLE device and a local GCS listening on UDP 127.0.0.1:14551
# --- START THE GCS AND OPEN UDP 14550 BEFORE RUNNING THIS SCRIPT ----

import asyncio
import logging
import socket
import sys
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.exc import BleakError

# --- Configuration ---
# Target BLE device name (Matches ESP-IDF ble_spp_server example)
DEVICE_NAME = "DroneBridge"

# Target UDP destination (where data from BLE should be sent) -> The GND Station listens to this port
UDP_TARGET_IP = "127.0.0.1"
UDP_TARGET_PORT = 14550

# Local UDP listening port (where data to be sent to BLE comes from)
UDP_LISTEN_IP = "127.0.0.1" # Listen on all interfaces
UDP_LISTEN_PORT = 14551 # Choose a free port

# UUIDs for the ESP-IDF SPP Server example (Service 0xABF0)
SPP_SERVICE_UUID = "0000abf0-0000-1000-8000-00805f9b34fb"
# Characteristic for sending data TO the ESP32 (Python Writes, ESP Receives - UUID 0xABF1)
# Corresponds to UUID_CHR_SPP_DATA_RECEIVE in ESP code (has WRITE property)
SPP_RX_CHAR_UUID = "0000abf1-0000-1000-8000-00805f9b34fb"
# Characteristic for receiving data FROM the ESP32 (Python Reads via Notify, ESP Sends - UUID 0xABF2)
# Corresponds to UUID_CHR_SPP_DATA_NOTIFY in ESP code (has NOTIFY property)
SPP_TX_CHAR_UUID = "0000abf2-0000-1000-8000-00805f9b34fb"
# ---------------------

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Queue to send data from UDP listener to BLE writer task
ble_write_queue = asyncio.Queue()

# Global reference to BLE client and write characteristic (for UDP Protocol)
ble_client_global: Optional[BleakClient] = None
write_characteristic_global: Optional[BleakGATTCharacteristic] = None
udp_transport_global: Optional[asyncio.DatagramTransport] = None


class UDPProtocol(asyncio.DatagramProtocol):
    """Handles receiving UDP datagrams and queuing them for BLE transmission."""
    def connection_made(self, transport: asyncio.DatagramTransport):
        """Called when the UDP socket is set up."""
        global udp_transport_global
        self.transport = transport
        udp_transport_global = transport # Store transport globally for sending
        logger.info(f"UDP listener started on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")

    def datagram_received(self, data: bytes, addr):
        """Called when a UDP datagram is received."""
        logger.debug(f"UDP received {len(data)} bytes from {addr}")
        # Put the received data onto the queue to be sent via BLE
        try:
            ble_write_queue.put_nowait(data)
        except asyncio.QueueFull:
            logger.warning("BLE write queue is full, dropping UDP packet.")

    def error_received(self, exc):
        """Called when a UDP send or receive operation raises an OSError."""
        logger.error(f"UDP error: {exc}")

    def connection_lost(self, exc):
        """Called when the UDP socket is closed."""
        logger.info("UDP listener stopped.")


def ble_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """Handles data received from BLE notifications and sends it via UDP."""
    logger.debug(f"BLE received {len(data)} bytes on handle {characteristic.handle} ({characteristic.uuid})")
    if udp_transport_global:
        try:
            udp_target = (UDP_TARGET_IP, UDP_TARGET_PORT)
            udp_transport_global.sendto(data, udp_target)
            logger.debug(f"Sent {len(data)} bytes to UDP {udp_target}")
        except Exception as e:
            logger.error(f"Failed to send data via UDP: {e}")
    else:
        logger.warning("UDP transport not ready, dropping BLE packet.")


async def ble_writer_task():
    """Waits for data on the queue and writes it to the BLE characteristic."""
    global ble_client_global, write_characteristic_global
    while True:
        try:
            data_to_send = await ble_write_queue.get()
            if ble_client_global and ble_client_global.is_connected and write_characteristic_global:
                try:
                    # Write without response is often preferred for SPP throughput.
                    # The ESP characteristic has WRITE property. If write-without-response
                    # fails or causes issues, change response=True.
                    await ble_client_global.write_gatt_char(
                        write_characteristic_global,
                        data_to_send,
                        response=False # Set to True if write without response doesn't work reliably
                    )
                    logger.debug(f"Sent {len(data_to_send)} bytes to BLE {write_characteristic_global.uuid}")
                except BleakError as e:
                    logger.error(f"Failed to write to BLE characteristic {write_characteristic_global.uuid}: {e}")
                    # Optional: Handle write errors (e.g., requeue, disconnect)
                except Exception as e:
                    logger.error(f"Unexpected error writing to BLE characteristic {write_characteristic_global.uuid}: {e}")
            else:
                logger.warning("BLE not connected or characteristic not found, dropping packet.")

            ble_write_queue.task_done() # Mark task as done for queue management

        except asyncio.CancelledError:
            logger.info("BLE writer task cancelled.")
            break
        except Exception as e:
            logger.error(f"Error in BLE writer task: {e}")
            await asyncio.sleep(1) # Avoid busy-looping on unexpected errors


async def main():
    """Main function to scan, connect, set up bridge, and run."""
    global ble_client_global, write_characteristic_global
    ble_client = None
    udp_transport = None
    udp_protocol = None
    writer_task = None
    tx_char = None # Define tx_char here for cleanup scope

    while True: # Outer loop to handle reconnection
        device_address = None
        logger.info(f"Scanning for BLE device named '{DEVICE_NAME}'...")
        try:
            # Scan for the device
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
            if device:
                device_address = device.address
                logger.info(f"Found device '{DEVICE_NAME}' at address {device_address}")
            else:
                logger.warning(f"Device '{DEVICE_NAME}' not found. Retrying in 10 seconds...")
                await asyncio.sleep(10)
                continue # Restart scanning

            # --- Setup UDP Listener ---
            loop = asyncio.get_running_loop()
            logger.info(f"Setting up UDP listener on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")
            try:
                # Ensure previous transport is closed before creating a new one
                if udp_transport and not udp_transport.is_closing():
                    udp_transport.close()
                    udp_transport = None

                udp_transport, udp_protocol = await loop.create_datagram_endpoint(
                    lambda: UDPProtocol(),
                    local_addr=(UDP_LISTEN_IP, UDP_LISTEN_PORT),
                    family=socket.AF_INET # Use AF_INET6 for IPv6
                )
            except OSError as e:
                logger.error(f"Failed to bind UDP listener to {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}: {e}. "
                             f"Port might be in use. Retrying in 3 seconds...")
                await asyncio.sleep(3)
                continue # Restart scanning/setup
            except Exception as e:
                logger.error(f"Unexpected error setting up UDP: {e}. Retrying in 3 seconds...")
                await asyncio.sleep(3)
                continue # Restart scanning/setup

            # --- Connect to BLE Device ---
            disconnected_event = asyncio.Event()
            def handle_disconnect(_: BleakClient):
                global ble_client_global, write_characteristic_global, udp_transport_global
                logger.warning(f"BLE device {DEVICE_NAME} disconnected.")
                ble_client_global = None
                write_characteristic_global = None
                # Don't clear udp_transport_global here, let the main loop handle UDP cleanup if needed
                # Signal the main loop to attempt reconnection
                disconnected_event.set()

            logger.info(f"Connecting to {device_address}...")
            ble_client = BleakClient(device_address, disconnected_callback=handle_disconnect)

            try:
                await ble_client.connect()
                if ble_client.is_connected:
                    logger.info(f"Successfully connected to BLE device {DEVICE_NAME}.")
                    ble_client_global = ble_client # Store globally

                    # --- Find Characteristics ---
                    logger.info(f"Looking for SPP Service: {SPP_SERVICE_UUID}")
                    # Ensure services are discovered (should happen on connect, but check)
                    await ble_client.get_services()
                    service = ble_client.services.get_service(SPP_SERVICE_UUID)
                    if not service:
                        logger.error(f"SPP Service {SPP_SERVICE_UUID} not found on device.")
                        await ble_client.disconnect()
                        if udp_transport: udp_transport.close()
                        await asyncio.sleep(5)
                        continue # Restart process

                    logger.info(f"Looking for SPP Notify Characteristic (TX): {SPP_TX_CHAR_UUID}")
                    tx_char = service.get_characteristic(SPP_TX_CHAR_UUID)
                    if not tx_char or "notify" not in tx_char.properties:
                        logger.error(f"SPP Notify Characteristic {SPP_TX_CHAR_UUID} (with Notify) not found.")
                        await ble_client.disconnect()
                        if udp_transport: udp_transport.close()
                        await asyncio.sleep(5)
                        continue # Restart process

                    logger.info(f"Looking for SPP Write Characteristic (RX): {SPP_RX_CHAR_UUID}")
                    rx_char = service.get_characteristic(SPP_RX_CHAR_UUID)
                    # Check for write or write-without-response
                    if not rx_char or not any(p in rx_char.properties for p in ["write", "write-without-response"]):
                        logger.error(f"SPP Write Characteristic {SPP_RX_CHAR_UUID} (with Write) not found.")
                        await ble_client.disconnect()
                        if udp_transport: udp_transport.close()
                        await asyncio.sleep(5)
                        continue # Restart process

                    write_characteristic_global = rx_char # Store globally
                    logger.info("Found required BLE Service and Characteristics.")

                    # --- Start Services ---
                    # Start BLE writer task
                    writer_task = asyncio.create_task(ble_writer_task())

                    # Start BLE notifications
                    logger.info(f"Starting notifications on {tx_char.uuid}...")
                    await ble_client.start_notify(tx_char, ble_notification_handler)

                    logger.info("--- Bridge is running ---")
                    logger.info(f"BLE ({DEVICE_NAME}) <--> UDP ({UDP_LISTEN_IP}:{UDP_LISTEN_PORT} -> {UDP_TARGET_IP}:{UDP_TARGET_PORT})")

                    # Keep running until disconnected
                    await disconnected_event.wait()
                    logger.info("Disconnect event received, cleaning up for reconnection...")

                else:
                    logger.error(f"Failed to connect to BLE device {DEVICE_NAME}.")

            except BleakError as e:
                logger.error(f"BleakError during BLE connection or setup: {e}")
            except Exception as e:
                logger.error(f"Error during BLE connection or setup: {e}", exc_info=True)

            finally:
                # --- Cleanup on disconnect or error before retrying ---
                logger.info("Cleaning up resources before potential reconnection...")
                if writer_task and not writer_task.done():
                    writer_task.cancel()
                    try:
                        await writer_task # Wait for task to finish cancellation
                    except asyncio.CancelledError:
                        pass # Expected
                # Clear queues and globals related to BLE connection
                while not ble_write_queue.empty():
                    try:
                        ble_write_queue.get_nowait()
                        ble_write_queue.task_done()
                    except asyncio.QueueEmpty:
                        break # Should not happen if not empty, but safety check
                ble_client_global = None
                write_characteristic_global = None
                # Keep UDP transport alive unless explicitly stopped by error or KeyboardInterrupt

                if ble_client and ble_client.is_connected:
                    try:
                        # Attempt to stop notifications gracefully if still connected and tx_char exists
                        if tx_char:
                            await ble_client.stop_notify(tx_char)
                            logger.info(f"Stopped notifications on {tx_char.uuid}")
                    except Exception as e:
                        logger.warning(f"Error stopping notifications: {e}")
                    try:
                        await ble_client.disconnect()
                        logger.info("BLE client disconnected.")
                    except Exception as e:
                        logger.warning(f"Error during BLE disconnect: {e}")

                # Don't close UDP transport here, allow reconnection attempts
                # It will be closed in the main exception handler or if a new one is created

                logger.info("Waiting 5 seconds before attempting reconnection...")
                await asyncio.sleep(5)

        except Exception as e:
            logger.critical(f"Unhandled exception in main loop: {e}", exc_info=True)
            # Clean up UDP on critical errors before retrying
            if udp_transport and not udp_transport.is_closing():
                udp_transport.close()
                udp_transport = None
            logger.info("Waiting 10 seconds before retrying...")
            await asyncio.sleep(10)


if __name__ == "__main__":
    udp_transport_main = None # Keep track of transport for final cleanup
    try:
        # Assign transport created in main loop to this variable if needed
        # This part is tricky as transport is created inside the loop.
        # A better approach might involve passing the transport object around.
        # For simplicity, we rely on the loop's cleanup and a final check.
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Script interrupted by user.")
    except Exception as e:
        logger.critical(f"Fatal error preventing script execution: {e}", exc_info=True)
    finally:
        # Attempt final cleanup of global UDP transport if it exists
        if udp_transport_global and not udp_transport_global.is_closing():
            logger.info("Closing UDP transport on final exit.")
            udp_transport_global.close()
        logger.info("Script finished.")
