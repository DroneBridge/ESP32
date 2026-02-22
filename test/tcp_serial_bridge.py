import sys
import socket
import serial
import threading
import time
import argparse

# Example usage:
# python tcp_serial_bridge.py --com COM3 --baud 921600 --server 127.0.0.1 --port 5760

# Buffer size for transfers
BUFFER_SIZE = 4096

def serial_to_tcp(serial_port, tcp_socket, stop_event):
    """
    Reads data from the Serial port and sends it to the TCP socket.
    """
    try:
        while not stop_event.is_set():
            # Check if data is waiting in the serial buffer
            if serial_port.in_waiting > 0:
                data = serial_port.read(serial_port.in_waiting or 1)
                if data:
                    tcp_socket.sendall(data)
                    # Optional: Print for debugging (comment out for true transparency)
                    # print(f"Serial -> TCP: {len(data)} bytes")
            else:
                # tiny sleep to prevent 100% CPU usage on empty loops
                time.sleep(0.01)

    except (serial.SerialException, socket.error) as e:
        print(f"\n[!] Error in Serial -> TCP connection: {e}")
        stop_event.set()
    except Exception as e:
        print(f"\n[!] Unexpected error in Serial -> TCP: {e}")
        stop_event.set()

def tcp_to_serial(tcp_socket, serial_port, stop_event):
    """
    Reads data from the TCP socket and sends it to the Serial port.
    """
    try:
        while not stop_event.is_set():
            # recv is blocking, but we set a timeout on the socket globally
            try:
                data = tcp_socket.recv(BUFFER_SIZE)
                if not data:
                    # Empty bytes implies the server closed the connection
                    print("\n[!] TCP Server closed connection.")
                    stop_event.set()
                    break
                
                serial_port.write(data)
                serial_port.flush()
                # Optional: Print for debugging
                # print(f"TCP -> Serial: {len(data)} bytes")
                
            except socket.timeout:
                # Continue loop if timeout to check stop_event
                continue
                
    except (socket.error, serial.SerialException) as e:
        print(f"\n[!] Error in TCP -> Serial connection: {e}")
        stop_event.set()
    except Exception as e:
        print(f"\n[!] Unexpected error in TCP -> Serial: {e}")
        stop_event.set()

def main():
    parser = argparse.ArgumentParser(description="Bi-directional TCP to Serial Bridge")
    
    # TCP Arguments
    parser.add_argument("--server", required=True, help="IP address or Hostname of the TCP Server")
    parser.add_argument("--port", type=int, required=True, help="TCP Port number")
    
    # Serial Arguments
    parser.add_argument("--com", required=True, help="Serial Port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    
    args = parser.parse_args()

    stop_event = threading.Event()
    
    # Initialize resources
    ser = None
    sock = None

    try:
        print(f"[*] Connecting to Serial Port: {args.com} @ {args.baud}...")
        ser = serial.Serial(args.com, args.baud, timeout=0.1)
        print("[+] Serial Connected.")

        print(f"[*] Connecting to TCP Server: {args.server}:{args.port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0) # Set timeout to allow thread checking
        sock.connect((args.server, args.port))
        print("[+] TCP Connected.")
        print("[*] Bridge Started. Press Ctrl+C to stop.")

        # Create threads for bi-directional flow
        t1 = threading.Thread(target=serial_to_tcp, args=(ser, sock, stop_event))
        t2 = threading.Thread(target=tcp_to_serial, args=(sock, ser, stop_event))

        # Daemon threads exit when main program exits
        t1.daemon = True
        t2.daemon = True

        t1.start()
        t2.start()

        # Keep main thread alive to catch KeyboardInterrupt
        while not stop_event.is_set():
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[*] Stopping bridge...")
        stop_event.set()
    except Exception as e:
        print(f"\n[!] Initialization Error: {e}")
        stop_event.set()
    finally:
        # Cleanup
        if ser and ser.is_open:
            ser.close()
            print("[*] Serial port closed.")
        if sock:
            sock.close()
            print("[*] TCP socket closed.")
        sys.exit()

if __name__ == "__main__":
    main()