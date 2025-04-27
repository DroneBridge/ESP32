#!/usr/bin/env python3
#
# TCP client to UART bridge using asyncio and pyserial

import asyncio
import logging
import serial_asyncio

# --- Configuration ---
UART_PORT = "/dev/ttyACM0"  # Change to your serial port
BAUD_RATE = 115200
TCP_HOST = "127.0.0.1"
TCP_PORT = 5761  # SITL's UART1 port

logging.basicConfig(level=logging.DEBUG)  # Change logging level to DEBUG for detailed output
logger = logging.getLogger(__name__)

async def handle_tcp_connection(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, uart_writer: asyncio.StreamWriter, uart_reader: asyncio.StreamReader):
    peername = writer.get_extra_info("peername")
    logger.info(f"TCP client connected: {peername}")

    async def tcp_to_uart():
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                logger.debug(f"TCP -> UART: {data}")
                uart_writer.write(data)
                await uart_writer.drain()
        except Exception as e:
            logger.error(f"TCP -> UART error: {e}")
        finally:
            logger.info("TCP connection closed (TCP->UART)")

    async def uart_to_tcp():
        try:
            while True:
                data = await uart_reader.read(256)
                if not data:
                    break
                logger.debug(f"UART -> TCP: {data}")
                writer.write(data)
                await writer.drain()
        except Exception as e:
            logger.error(f"UART -> TCP error: {e}")
        finally:
            logger.info("TCP connection closed (UART->TCP)")

    await asyncio.gather(tcp_to_uart(), uart_to_tcp())
    writer.close()
    await writer.wait_closed()
    logger.info("TCP client disconnected.")

async def main():
    logger.info(f"Opening UART on {UART_PORT} at {BAUD_RATE} baud...")
    try:
        uart_reader, uart_writer = await serial_asyncio.open_serial_connection(url=UART_PORT, baudrate=BAUD_RATE)
    except Exception as e:
        logger.error(f"Failed to open serial connection: {e}")
        return

    logger.info("Serial connection opened successfully.")

    # Create TCP connection
    try:
        reader, writer = await asyncio.open_connection(TCP_HOST, TCP_PORT)
        logger.info(f"Connected to SITL server at {TCP_HOST}:{TCP_PORT}")
    except Exception as e:
        logger.error(f"Failed to connect to SITL server: {e}")
        return

    await handle_tcp_connection(reader, writer, uart_writer, uart_reader)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
