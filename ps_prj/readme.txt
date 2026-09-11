# PLAT-144: PCIe End-to-End Latency Measurement — Options

**Context:** AM6442 (PCIe root complex) → PCIe Gen2 x1 → Xilinx XDMA IP → AXI4-Lite → AXI-to-Wishbone bridge → Wishbone bus (80 MHz) → FPGA fabric targets

---

## Option 1: Software Timestamp (Simplest, Least Precise)

### Overview

Use the Linux high-resolution monotonic clock to timestamp a register access immediately before and after the PCIe BAR transaction. This measures the round-trip latency as seen by the application layer, including all software and driver overhead.

### Method

```c
#include <time.h>
#include <stdint.h>

struct timespec t0, t1;
volatile uint32_t *bar = (volatile uint32_t *)mmap_base;

clock_gettime(CLOCK_MONOTONIC, &t0);
uint32_t val = bar[REG_OFFSET];   // or a write followed by a readback
clock_gettime(CLOCK_MONOTONIC, &t1);

long elapsed_ns = (t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec);
```

Repeat over many iterations (e.g. 10,000 samples) and record min, mean, and max.

### What it measures

- Full round-trip: userspace → kernel MMIO → PCIe TLP → XDMA → AXI-Lite → Wishbone bridge → target → return path → userspace
- Includes: kernel scheduling jitter, cache effects, driver overhead, PCIe completion timeout window

### Advantages

- No hardware modifications required
- Quick to implement on an existing Linux + XDMA driver setup
- Useful for establishing a system-level baseline

### Limitations

- `clock_gettime` itself has ~20–50 ns overhead on AM6442
- Cannot separate individual latency components (PCIe, AXI, Wishbone)
- Results will vary with CPU load, cache state, and kernel scheduling
- Not suitable for real-time or hard-deadline characterisation

### Expected result range

Typically 5–20 µs end-to-end for a PCIe Gen2 MMIO read on an embedded SoC, dominated by PCIe completion latency and driver overhead.

---

## Option 2: GPIO Loopback with Logic Analyser (Hardware Timestamp, Precise)

### Overview

Use a spare GPIO on the AM6442 as a hardware timestamp signal. Assert the GPIO immediately before initiating the BAR transaction and deassert it once the read/write has returned. Capture the pulse width with a logic analyser or oscilloscope. This removes software overhead from the measurement and gives a hardware-accurate round-trip time.

### Method

#### AM6442 side (bare metal or Linux GPIO)

```c
// Set GPIO high before transaction
gpio_set(GPIO_LATENCY_PROBE, 1);

// Perform PCIe BAR access
volatile uint32_t *bar = (volatile uint32_t *)mmap_base;
uint32_t val = bar[REG_OFFSET];

// Set GPIO low after completion
gpio_set(GPIO_LATENCY_PROBE, 0);
```

On Linux, use the `/dev/gpiochip` character device or a kernel driver with direct register access to minimise GPIO toggle latency. A dedicated bare-metal test application will give cleaner results.

#### Measurement

- Connect a logic analyser or digital oscilloscope probe to the GPIO pin
- Trigger on the rising edge, measure to the falling edge
- Sample rate must be ≥ 100 MSa/s for sub-10 µs resolution
- Repeat and capture a histogram of pulse widths

### What it measures

- Hardware-level round-trip: GPIO assert → PCIe TLP issued → completion received → GPIO deassert
- Excludes: clock_gettime overhead, most kernel scheduling jitter
- Still includes: GPIO driver latency (if using Linux), CPU pipeline effects

### Advantages

- Significantly more accurate than software-only timestamps
- Can be used to characterise worst-case and best-case latency independently
- Does not require any FPGA modifications
- Can be combined with a second GPIO or FPGA-side signal for finer breakdown

### Limitations

- Requires a free GPIO on AM6442 and access to the pin header
- GPIO toggle overhead in Linux can be 1–5 µs; bare-metal GPIO is ~10–20 ns
- Does not distinguish latency components internal to the FPGA

### Suggested test

Run 1,000+ transactions and capture a latency histogram. Look for:
- **Min**: best-case PCIe completion (uncongested, cache warm)
- **Max**: worst-case, may reveal timeout or retry events
- **Spread**: indicates jitter — important for ADC/DAC control paths

---

## Option 3: FPGA-Side Hardware Timestamp Register (Most Precise, Engineering Effort Required)

### Overview

Add a pair of free-running cycle counter registers to the Wishbone fabric. The first register captures the Wishbone cycle count at the moment `WB_STB` is asserted (request received by the target). The second captures the count when `WB_ACK` is returned to the bridge. The host reads these registers after each transaction and computes internal FPGA latency. Combined with the software or GPIO timestamps from Options 1/2, this allows full decomposition of the latency budget.

### FPGA modifications required

Add a simple timestamp capture module on the Wishbone bus:

```vhdl
-- Free-running counter (80 MHz clock domain)
process(clk_wb)
begin
    if rising_edge(clk_wb) then
        free_counter <= free_counter + 1;

        -- Capture on STB assertion (start of transaction)
        if WB_STB_I = '1' and WB_CYC_I = '1' and stb_prev = '0' then
            ts_request <= free_counter;
        end if;

        -- Capture on ACK (end of transaction)
        if WB_ACK_O = '1' then
            ts_ack <= free_counter;
        end if;

        stb_prev <= WB_STB_I;
    end if;
end process;

-- Expose ts_request and ts_ack as read-only Wishbone registers
-- e.g. at alias addresses 0x7FFE and 0x7FFF
```

### What it measures

- **ts_ack − ts_request**: Wishbone-internal latency (fabric + target response time), in cycles at 80 MHz (resolution: 12.5 ns)
- Combined with GPIO measurement: isolates AXI-to-Wishbone bridge CDC latency
- Combined with software timestamp: full latency budget breakdown

### Full latency decomposition

```
Total (SW timestamp)
 ├── AM6442 driver / MMIO overhead      → Option 1 minus Option 2
 ├── PCIe TLP + XDMA processing         → Option 2 minus Option 3 (FPGA-internal)
 ├── AXI-to-Wishbone bridge (incl. CDC) → derived
 └── Wishbone target response           → Option 3 (ts_ack − ts_request)
```

### Advantages

- Highest precision: 12.5 ns resolution at 80 MHz
- No external equipment required once registers are in place
- Permanent diagnostic capability — useful beyond PLAT-144 for ongoing integration
- Directly exposes bridge and CDC overhead, relevant to PLAT-143 clock tolerance work

### Limitations

- Requires an FPGA rebuild and re-flashing (via `flash_ax7103.sh`)
- Adds a small number of LUT/FF resources
- Timestamp registers must be protected from concurrent access if multi-threaded host software is used
- Only measures within the FPGA clock domain; PCIe path is not visible

### Suggested implementation plan

1. Add free-running counter and capture registers to `wb_distribution.vhd` or a new `wb_latency_probe.vhd` module
2. Assign two read-only Wishbone alias addresses (e.g. `0x7FFE`, `0x7FFF`)
3. Update `legacy_dsp_remap.vhd` address decode to include the new module
4. Rebuild, flash, verify with `reg_rw_test.py`
5. Run combined measurement: software timestamp + FPGA register readback in same transaction loop

---

## Recommendation

| Option | Effort | Precision | Equipment needed | Recommended use |
|--------|--------|-----------|-----------------|-----------------|
| 1 — Software timestamp | Low | ±1–5 µs | None | Quick baseline |
| 2 — GPIO loopback | Medium | ±50–100 ns | Logic analyser / oscilloscope | Hardware validation |
| 3 — FPGA timestamp register | High | ±12.5 ns | None (after FPGA rebuild) | Full latency breakdown |

**For PLAT-144, the recommended approach is to run Options 1 and 2 first to establish a baseline, then implement Option 3 to provide the breakdown Tim needs to understand where the latency budget is spent.** Option 3 also directly supports the concern raised about ADC/DAC signal processing latency sensitivity.
