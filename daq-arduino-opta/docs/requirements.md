# STM32H747XI Arduino Opta - 8-Channel High-Speed ADC Acquisition System Requirements (UPDATED)

## System Overview
Develop a high-performance, real-time analog data acquisition system using STM32H747XI dual-core microcontroller on Arduino Opta platform for industrial-grade burst sampling of 8 analog channels with RAM buffering and USB transfer capability.

## Core Acquisition Flow
The system operates in three distinct phases:

1. **High-Speed RAM Acquisition Phase**
   - All 8 channels acquire samples simultaneously at exactly 10 kHz
   - Samples are stored directly in Opta's RAM memory using DMA
   - RAM writing must NOT distort sampling timing (hardware-timed acquisition)
   - Acquisition continues for user-defined duration (typically 1-10 seconds)

2. **Acquisition Stop Phase**
   - Sampling stops cleanly after predetermined sample count is reached
   - ADC and DMA operations cease
   - System prepares for data transfer phase

3. **USB Data Transfer Phase**
   - Acquired samples are transferred from RAM to laptop via USB serial
   - Transfer occurs at maximum sustainable USB serial speed
   - Data format: timestamp, raw ADC values, calculated voltages
   - No real-time constraints during this phase

## Hardware Requirements

### ADC Configuration
- **Number of Channels**: 8 analog input channels (A0 through A7 - all available Arduino Opta analog pins)
- **ADC Resolution**: 12-bit native resolution (expandable to 16-bit using oversampling)
- **Input Voltage Range**: 0-10V (Arduino Opta native range with internal voltage divider factor of 0.3034)
- **Voltage Range Adjustment**: Software-configurable maximum range (0-5V, 0-10V via scaling factors)
- **Input Impedance**: High impedance (>1MΩ) to minimize loading effects
- **Channel Configuration**: Single-ended inputs
- **Hardware**: Utilize ADC1, ADC2, and ADC3 in simultaneous mode for maximum parallelism
- **Internal Processing**: Account for Arduino Opta's internal voltage divider (3.3V max to MCU, 10V max input)

### Timing Requirements
- **Sampling Frequency**: Exactly 10.000 kHz per channel
- **Sampling Mode**: Burst acquisition - continuous sampling for defined duration, then stop
- **Timing Source**: Hardware timer (TIM1) triggering ADC conversions via TRGO
- **Jitter Specification**: Maximum ±50ns timing deviation between samples
- **Phase Alignment**: All 8 channels sampled simultaneously within ±1 ADC clock cycle
- **Sample Loss**: Zero tolerance during acquisition phase

### Memory Management Requirements
- **RAM Buffer Size**: Configurable acquisition duration:
  - 1 second: 8 channels × 2 bytes × 10,000 samples = 160 kB
  - 5 seconds: 8 channels × 2 bytes × 50,000 samples = 800 kB
  - 10 seconds: 8 channels × 2 bytes × 100,000 samples = 1.6 MB
- **Buffer Structure**: Ping-pong double buffering using DMA circular mode
- **Memory Protection**: Buffer overflow detection and prevention
- **Data Integrity**: Sample counter verification and data validation

### Real-time Performance
- **Dual-Core Utilization**: 
  - **Cortex-M7**: USB communication, data formatting, system control, user interface
  - **Cortex-M4**: Real-time ADC control, DMA management, precise timing operations
- **DMA Configuration**: Multi-stream DMA with circular buffer mode for continuous data flow
- **Inter-Core Communication**: Shared memory regions for status and control data exchange

## Software Requirements

### Data Acquisition System (STM32H747XI/Arduino)

#### Core Functionality
1. **ADC Control System**:
   - Configure ADC1+ADC2+ADC3 in triple simultaneous mode
   - Hardware timer trigger (TIM1) for precise 10 kHz sampling
   - DMA-based data transfer with zero CPU intervention during sampling
   - Automatic channel sequencing for all 8 channels per trigger event

2. **RAM Buffer Management**:
   - Pre-allocated static buffers in RAM (avoid dynamic allocation)
   - Double buffering: one buffer filling while other is being processed
   - Sample counting and acquisition stop logic
   - Buffer overflow protection with immediate error indication

3. **Acquisition Control**:
   - User button (Arduino Opta built-in) to start/stop acquisition cycles
   - LED status indicators:
     - **Red LED**: System ready/idle
     - **Green LED**: Active sampling in progress
     - **Blue LED**: Data transfer in progress
     - **Blinking patterns**: Error states

#### Serial Monitor Interface (Diagnostics Only)
**No Serial Plotter visualization required.** Serial Monitor provides:

1. **Acquisition Status Messages**:
   ```
   === Arduino Opta ADC Acquisition System ===
   System Ready - Press USER button to start acquisition
   
   [START] Acquisition started - 10 seconds @ 10 kHz
   Channels: 8, Buffer size: 1.6 MB
   
   [PROGRESS] Samples acquired: 25,000 (25% complete)
   [PROGRESS] Samples acquired: 50,000 (50% complete)
   [PROGRESS] Samples acquired: 75,000 (75% complete)
   
   [STOP] Acquisition complete!
   Total samples per channel: 100,000
   Total acquisition time: 10.000 seconds
   Actual sampling rate: 10000.0 Hz
   Data integrity: OK (0 errors)
   
   [TRANSFER] Starting USB data transfer...
   Transfer rate: 825 kB/s
   [TRANSFER] Complete! Ready for next acquisition.
   ```

2. **System Diagnostics** (available on command):
   ```
   === System Status ===
   Core temperatures: M7: 42°C, M4: 40°C
   RAM usage: 1.6MB / 1.8MB available (89%)
   DMA errors: 0
   Buffer overruns: 0
   System uptime: 00:15:32
   Last acquisition: 100,000 samples, no errors
   ```

3. **Configuration Commands**:
   ```
   Commands available:
   - 'status' - Show system status
   - 'config' - Show acquisition configuration
   - 'test' - Run system self-test
   - 'help' - Show available commands
   ```

### PC Data Acquisition Software (Python)

#### Communication Interface
- **Protocol**: USB Serial (921600 baud minimum for fast data transfer)
- **Data Format**: Binary transfer for efficiency, with CSV conversion on PC side
- **Handshaking**: Command/response protocol with ACK/NACK confirmation
- **Flow Control**: Hardware handshaking (RTS/CTS) to prevent data loss during transfer

#### Python Application Features
1. **Acquisition Control**:
   - Automatic Arduino Opta device detection
   - Configure acquisition parameters (duration, channels, voltage ranges)
   - Remote start/stop acquisition commands
   - Real-time transfer progress monitoring

2. **Data Reception and Storage**:
   - **Binary Reception**: Raw data received as binary stream for speed
   - **CSV Conversion**: Automatic conversion to CSV format with headers:
     ```
     Timestamp_ms, A0_raw, A0_volts, A1_raw, A1_volts, ..., A7_raw, A7_volts
     ```
   - **Data Precision**: 
     - Timestamp: 0.1ms resolution (float)
     - ADC raw values: uint16_t (0-4095 for 12-bit)
     - Voltages: 4 decimal places (0.0001V resolution)
   - **File Management**: Automatic timestamped files (e.g., `ADC_Data_20250925_143022.csv`)

3. **Transfer Performance**:
   - **Data Rate Calculation**: Real-time transfer rate monitoring
   - **Progress Indication**: Visual progress bar during large transfers
   - **Error Detection**: Data integrity verification during transfer
   - **Retry Logic**: Automatic retry for failed transfers

#### Python Requirements (requirements.txt):
```
numpy>=1.21.0
matplotlib>=3.5.0  # For optional post-acquisition plotting
pandas>=1.3.0
pyserial>=3.5
tkinter  # GUI interface
scipy>=1.7.0  # For data analysis functions
tqdm>=4.60.0  # Progress bars
```

## Performance Specifications

### Timing Performance
- **Sampling Accuracy**: ±0.01% frequency stability over acquisition period
- **Channel-to-Channel Skew**: <100ns (simultaneous sampling via triple ADC mode)
- **Acquisition Jitter**: <±50ns sample-to-sample timing variation
- **Stop Precision**: Acquisition stops within ±1 sample of target count

### Data Throughput
- **Acquisition Rate**: 160 kB/s during sampling (8 channels × 2 bytes × 10 kHz)
- **RAM Storage**: Direct DMA to RAM with zero CPU overhead
- **USB Transfer Rate**: 
  - Minimum sustained: 800 kB/s (5x acquisition rate for fast transfer)
  - Target: 1.5 MB/s for optimal user experience
- **Maximum Buffer Capacity**: 10 seconds acquisition (1.6 MB total)

### Memory Requirements
- **Static RAM Allocation**: Pre-allocated buffers to avoid fragmentation
- **Buffer Overhead**: 10% additional space for safety margins
- **Stack Usage**: Minimal stack usage during high-speed acquisition
- **Available RAM**: STM32H747XI has 1MB RAM - system uses ~1.8MB for 10-second acquisition

## System Operation Modes

### Mode 1: Manual Trigger
- User presses button to start acquisition
- Fixed duration acquisition (configurable: 1, 5, or 10 seconds)
- Automatic stop after sample count reached
- LED indication throughout process

### Mode 2: PC-Controlled
- Python application sends start command via USB
- Configurable acquisition duration from PC
- Real-time status updates to PC during acquisition
- Automatic data transfer initiation after acquisition complete

### Mode 3: Continuous Cycle
- Repeated acquisition cycles with user-defined intervals
- Automatic file naming with sequential numbering
- Long-term data logging capability
- System health monitoring between cycles

## Safety and Error Handling

### Error Detection
- **Buffer Overflow**: Immediate detection with LED error indication
- **DMA Errors**: Hardware error detection with automatic recovery
- **Timing Violations**: Real-time monitoring of sample timing accuracy
- **Communication Errors**: USB transfer error detection and retry logic

### Error Recovery
- **Graceful Shutdown**: Clean stop on any critical error
- **System Reset**: Automatic restart capability after recoverable errors
- **Data Preservation**: Partial data saved even if acquisition interrupted
- **Error Logging**: Persistent error history in non-volatile memory

### Industrial Compliance
- **Operating Temperature**: -40°C to +85°C (Arduino Opta specification)
- **Input Protection**: Built-in Arduino Opta overvoltage protection
- **EMI Immunity**: Arduino Opta platform compliance with industrial standards
- **Power Supply Tolerance**: Wide input voltage range handling

## Development Deliverables

### Code Deliverables
1. **Arduino Sketch**: Complete STM32H747XI dual-core firmware with:
   - M7 core: Main control, USB communication, user interface
   - M4 core: Real-time ADC control and DMA management
   - HAL-based implementation with Arduino IDE compatibility

2. **Python Application**: Complete PC-side software with:
   - GUI interface for acquisition control
   - Automatic device detection and configuration
   - Data reception and CSV conversion
   - Real-time progress monitoring

3. **Configuration Tools**: 
   - Acquisition parameter setup utility
   - System calibration and test procedures
   - Performance validation scripts

### Documentation
1. **User Manual**: Complete setup and operation guide
2. **Technical Reference**: API documentation and code structure
3. **Calibration Procedures**: Accuracy verification and adjustment
4. **Troubleshooting Guide**: Common issues and solutions

### Validation and Testing
1. **Timing Accuracy Testing**: Oscilloscope verification of 10 kHz sampling
2. **Data Integrity Testing**: Long-duration acquisition validation
3. **Performance Benchmarking**: Transfer rate and system resource utilization
4. **Industrial Environment Testing**: Temperature and EMI compliance verification

## Future Enhancement Possibilities
- **Multi-Device Synchronization**: Multiple Arduino Opta units with shared timebase
- **Advanced Triggering**: External trigger sources for event-based acquisition
- **Real-time Processing**: On-board filtering and analysis during acquisition
- **Web Interface**: Browser-based monitoring and control
- **Cloud Integration**: Direct data upload to analysis platforms