
import time
import machine
import struct
from machine import SPI, Pin, PWM

FPGA_CLK = 24
TT_RST = 12

REG_ENABLE = 0x00
REG_PERIOD = 0x04
REG_SAMPLE = 0x08
REG_SAMPLE = 0x0c


# Setup buffer for error reports in interrupts
import micropython
micropython.alloc_emergency_exception_buf(100)

class PeripheralCommunicationSPI():

    """
    Read/write data in TinyQV peripheral over SPI 
    On an FPGA that is flashed with test_harness

    | Bits | Meaning |
    | ---- | ------- |
    | 31    | Read or write command: 1 for a write, 0 for a read |
    | 30-29 | Transaction width 0, 1 or 2 for 8, 16 or 32 bits |
    | 28-6  | Unused |
    | 5-0   | The register address |
    """

    def __init__(self, spi, cs_n):
        self.spi = spi
        self.cs_n = cs_n

    @micropython.native
    def read32_into(self, addr : int, buf, cmd):
        """
        Read into a pre-allocated buffer. Fastest
        """

        self.cs_n.value(0)
        self.spi.write(cmd)
        self.spi.readinto(buf)
        self.cs_n.value(1)

    @micropython.native
    def read32(self, addr : int):
        if addr < 0 or addr > 2**5:
            raise ValueError("Invalid address")

        cmd = bytearray([ 0b01000000, 0, 0, addr ])
        read_data = bytearray([0xFF, 0, 0xFF, 0])
        self.read32_into(addr, read_data, cmd)
        return read_data


    def write32(self, addr, data):
        if addr < 0 or addr > 2**5:
            raise ValueError("Invalid address")

        self.cs_n.value(0)
        cmd = [ 0b11000000, 0, 0, addr ]
        p = bytearray(cmd + list(data))
        #print('write', p)

        #spi.write(b'\xC0\x00\x00' + struct.pack('>B', address) + struct.pack('>L', data))

        self.spi.write(p)
        self.cs_n.value(1)


# globals
spi = None
spi_cs_n = None
read_pcm_cmd = bytearray([ 0b01000000, 0, 0, REG_SAMPLE ])
pcm_sample = bytearray(4)
last_trigger : int = -1
pcm_buffer = micropython.RingIO(4*16*10)

timing_buffer = micropython.RingIO(4*16*10)
timing_sample = bytearray(1)

@micropython.native
def pcm_ready_interrupt(p):
    
    # check how often we are able to go back to this
    #global last_trigger
    #t : int = time.ticks_us()
    #if last_trigger > 0:
    #    diff : int = time.ticks_diff(t, last_trigger)
    #    timing_sample[0] = diff//10
    #    timing_buffer.write(timing_sample)
    #    #print('pcm trigg', diff)
    #last_trigger = t

    # Track how long we spend inside handler
    #start = time.ticks_us()

    #time.sleep_ms(1)
    spi_cs_n.value(0)
    spi.write(read_pcm_cmd)
    spi.readinto(pcm_sample)
    spi_cs_n.value(1)

    # put pcm_sample onto queue
    pcm_buffer.write(pcm_sample)

    #duration = time.ticks_diff(time.ticks_us(), start)
    #print('d', duration)

def read_with_interrupt(duration=10):

    total_samples = 0
    read_buffer = bytearray(10)
    pcm_start = time.ticks_us()
    pcm_last = pcm_start
    while True:

        # check timing queue
        #n_bytes = timing_buffer.readinto(read_buffer)
        #if n_bytes > 0:
        #    for t in read_buffer[0:n_bytes]:
        #        print('t', t)

        # check PCM queue
        n_bytes = pcm_buffer.readinto(read_buffer)
        if n_bytes > 0:
            total_samples += (n_bytes/4)
            pcm_last = time.ticks_us()

        # check if we are stuck
        since_last = time.ticks_diff(time.ticks_us(), pcm_last)
        if since_last > 1000*1000:
            print('TIMEOUT', since_last)
            break

        # check if we are done
        since_start = time.ticks_diff(time.ticks_us(), pcm_start)
        if since_start > int(duration*1000*1000):
            print('FINISHED')
            break

        #time.sleep_ms()
    
    if total_samples == 0:
        print('FAILED, 0 samples')
    delta = time.ticks_diff(pcm_last, pcm_start)+1
    effective_samplerate = total_samples / (delta/1e6)
    print('read', delta/1e3, total_samples, effective_samplerate)

def run_test():

    # Set clock frequency of RP2040
    machine.freq(200_000_000)

    # Set everything to inputs, to be safe
    for i in range(30):
        Pin(i, Pin.IN, pull=None)

    clk = start_peripheral()
    print(clk)

    # The SPI communication expects pico-ice
    interrupt = Pin(1, Pin.IN)
    
    for i in range(10):
        print(interrupt.value())
        time.sleep(0.10)

    interrupt.irq(trigger=Pin.IRQ_RISING, handler=pcm_ready_interrupt)

    # SPI communication
    # Chip select, active low
    global spi_cs_n
    spi_cs_n = Pin(13, Pin.OUT) # Pin 9 is used for ICE40 flash SS, should be avoided

    # NOTE: Above 6 Mhz started to get read/write errors
    global spi
    spi = SPI(0, 4_000_000,
            sck=Pin(2),
            mosi=Pin(3),
            miso=Pin(0),
            #sck=Pin(10),
            #mosi=Pin(11),
            #miso=Pin(8),
            bits=8,
            phase=0,
            polarity=0,
            firstbit=SPI.MSB)
    peri = PeripheralCommunicationSPI(spi, spi_cs_n)


    ctrl = peri.read32(REG_ENABLE)
    print('control', ctrl)

    # Set and check scale
    clkp = peri.read32(REG_PERIOD)
    scale_config = bytearray([0x00, 0x00, 0x00, 64])
    peri.write32(REG_PERIOD, scale_config)

    clkp = peri.read32(REG_PERIOD)
    assert clkp == scale_config, (clkp, scale_config)

    #for i in range(10):
    #    peri.write32(REG_ENABLE, bytearray([0x00, 0x00, 0x00, 0x01]))
    #    time.sleep(2.0)
    #    peri.write32(REG_ENABLE, bytearray([0x00, 0x00, 0x00, 0x00]))
    #    time.sleep(2.0)

    # Enable the PDM clock
    print('enable clock')
    clock_config = bytearray([0x00, 0x00, 0x00, 0x01])
    peri.write32(REG_ENABLE, clock_config)
    ctrl = peri.read32(REG_ENABLE)
    assert ctrl == clock_config, (ctrl, clock_config)

    # Try to read PCM data as fast as we can with interrupt
    # XXX: disabled, was unable to go faster than 11-12 khz
    for i in range(0):
        read_with_interrupt()

        # HACK: try to recover missing interrupt by doing a read
        peri.read32(REG_SAMPLE)


    samplerate = 16000
    duration = 4.0
    print("Recording PCM", duration, 'seconds')
    pcm_start = time.ticks_us()
    n_samples = int(duration*samplerate)
    samples = bytearray(2*n_samples)
    read_pcm_samples_loop(peri, samples, interrupt)
    pcm_read_duration = time.ticks_diff(time.ticks_us(), pcm_start)
    per_sample = pcm_read_duration / n_samples
    deadline = 63 # at 16khz, only have 62 us between each sample
    print('PCM read duration', per_sample)
    assert per_sample < deadline, (per_sample, deadline)

    file_path = 'pcm.raw'
    with open(file_path, 'wb') as f:
        f.write(samples)
    print('Wrote', file_path)
    

@micropython.native
def read_pcm_samples_loop(peri, samples, interrupt):

    n_samples = len(samples) # 1 byte per each, for now
    print('s', n_samples)

    # pre-allocate buffers
    mem = memoryview(samples) 
    tx = bytearray([ 0b01000000, 0, 0, REG_SAMPLE, 0, 0, 0, 0 ])
    rx = bytearray(8)

    # cache member references, avoids lookup
    # and we do not have the function call overhead
    cs_n = peri.cs_n
    spi = peri.spi

    i = 0
    while i < n_samples:
        ready = interrupt()
        if ready:
            cs_n(0)
            spi.write_readinto(tx, rx)
            mem[i+1] = rx[6]
            mem[i] = rx[7]
            i += 2
            cs_n(1)

def start_peripheral():

    # Start the ICE40 FPGA
    ice_creset_b = machine.Pin(27, machine.Pin.OUT)
    ice_creset_b.value(0)

    ice_done = machine.Pin(26, machine.Pin.IN)
    time.sleep_us(10)
    ice_creset_b.value(1)

    while ice_done.value() == 0:
        print(".", end = "")
        time.sleep(0.001)
    print("ICE40 done")


    # Start the TinyTapeout TinyQV peripheral
    rst_n = Pin(TT_RST, Pin.OUT)
    clk = Pin(FPGA_CLK, Pin.OUT)

    clk.off()
    rst_n.on()
    time.sleep(0.001)
    rst_n.off()

    clk.on()
    time.sleep(0.001)
    clk.off()
    time.sleep(0.001)

    for i in range(10):
        clk.off()
        time.sleep(0.001)
        clk.on()
        time.sleep(0.001)

    rst_n.on()
    time.sleep(0.001)
    clk.off()

    time.sleep(0.001)
    clk = PWM(FPGA_CLK, freq=64_000_000, duty_u16=32768)
    print("TT clock running")

    return clk


if __name__ == '__main__':
    run_test()



