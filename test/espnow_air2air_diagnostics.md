# ESP-NOW AIR-to-AIR diagnostics

Build with ESP-IDF **5.4.4**. In `idf.py menuconfig`, under
**DroneBridge for ESP32 Configuration**, enable
`CONFIG_DB_ESPNOW_DIAGNOSTICS`. It defaults to disabled.

Use an INFO-level UART debug console and disable the secondary console.
Connect a separate debug adapter to that console, not the MAVLink UART.
USB/JTAG data firmware uses the UART debug console too. Diagnostics refuse
to start if the UART data port or its RX/TX pins overlap the console.
Only ESP-NOW mode starts the diagnostic task. No parameters or forwarding
policies change; original buffers and queue timeouts remain in use.

## Capture

On the same AIR, capture at least 30 seconds with AIR2AIR disabled, then
enabled, including a parameter download in each capture. Record baud rate,
flow control, both system IDs and whether downloads run concurrently.
Keep the debug adapter draining output. Filter captured console output for
`DB_DIAG`. Do not enable per-packet debug logging for this comparison.

## Fields

Peer AIR-to-FC forwarding suppresses RADIO_STATUS (109) and legacy RADIO
(166) after complete frame assembly. These describe the peer radio link;
forwarding their txbuf reports can throttle the local FC and stall MAVFTP.
AIR-to-GND and GND-to-FC forwarding remain unchanged.
`peer_radio_filtered` reports interval/cumulative suppressed frames.
Reboot the FC before comparing the fix to clear accumulated stream slowdown.

Reports cover the preceding `window_ms`, nominally 1000 ms. All durations
include scheduler/preemption time; they measure observed latency, not CPU time.

- `write_gnd`, `write_peer`, `write_local`: completed calls, actual accepted
  bytes, total/average/maximum call duration, and partial/failed write count.
  These are serial-driver acceptance counts, not proof of FC delivery.
  Source classification applies to parsed ESP-NOW MAVLink traffic; other
  serial writes use `local`. On GND, `peer` means received AIR traffic.
- `q_rx_gnd`, `q_rx_peer`: submission timings split by radio origin; both
  share the same six-entry UART queue. On GND, AIR arrivals use `rx_peer`.
- `q_tx`, `q_callback`: outgoing packet submissions and shared Wi-Fi
  send/receive callback submissions. `total_us` and `max_us` include
  successful waits. `fail` is interval/cumulative failed submissions.
- `peak_sample`: maximum occupancy sampled immediately before/after a
  submission. It is a lower bound on the true peak because consumers run
  concurrently. Current queue capacity is six.
- `read_gap_us`: maximum gap between serial read attempts; includes a
  stall still in progress. It crosses reporting boundaries intentionally.
- `rx_full`, `fifo_ovf`: interval/cumulative observed UART RX buffer-full
  and FIFO-overflow notifications. Buffer-full does not alone prove loss;
  FIFO overflow indicates receive overrun. Event counts are lower bounds:
  the unchanged ten-entry event queue may itself overflow. `events=unavailable`
  is expected for USB/JTAG data firmware.

The reporter drains UART notifications every 10 ms and never flushes or
reads UART data. Logging runs in a separate low-priority task, with no stats
lock held while logging or waiting for I/O. It still consumes CPU/console
bandwidth, so keep the same diagnostic build for both comparison captures.

Large peer write durations correlated with large read gaps support UART
blocking. Large queue durations support task contention even if `fail=0`.
The counters identify mechanisms; hardware comparison is required before
claiming the cause of MAVFTP failures is confirmed.
