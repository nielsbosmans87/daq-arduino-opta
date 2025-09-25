# DAQ Arduino Opta - 8-Channel High-Speed ADC Acquisition System Requirements

## System Overview
Develop a high-performance, real-time analog data acquisition system using STM32H747XI dual-core microcontroller on Arduino Opta platform for industrial-grade continuous sampling of 8 analog channels.

## Hardware Requirements

### ADC Configuration
- **Number of Channels**: 8 analog input channels (A0 through A7 - all available Arduino Opta analog pins)
- **ADC Resolution**: 16-bit (using oversampling) with configurable range in software
- **Input Voltage Range**: 0-10V (Arduino Opta native range with internal voltage divider factor of 0.3034)
- **Voltage Range Adjustment**: Software-configurable maximum range (e.g., 0-5V, 0-10V via scaling factors)
- **Input Impedance**: High impedance (>1MΩ) to minimize loading effects  
- **Channel Configuration**: Single-ended inputs
- **Hardware**: Utilize ADC1, ADC2, and ADC3 in triple simultaneous mode for maximum parallelism
- **Internal Processing**: Account for Arduino Opta's internal voltage divider (3.3V max to MCU, 10V max input)

### Timing Requirements
- **Sampling Frequency**: Exactly 10.000 kHz per channel
- **Sampling Mode**: Continuous, simultaneous sampling of all 8 channels
- **Timing Source**: Hardware-based timing using TIM1 or similar high-resolution timer
- **Jitter Specification**: Maximum ±50ns timing deviation
- **Phase Alignment**: All 8 channels must be sampled within ±1 ADC clock cycle
- **Sample Loss**: Zero tolerance - no missed samples allowed

### Real-time Performance
- **Dual-Core Utilization**: 
  - Cortex-M7: Main processing, data handling, PC communication, real-time calculations
  - Cortex-M4: Real-time ADC control, timing-critical operations, DMA management
- **DMA Configuration**: Triple-DMA streams with circular buffer mode and double buffering to prevent data loss
- **Buffer Management**: Minimum 2-second data buffer (20,000 samples × 8 channels × 2 bytes = 320kB)
- **Inter-Core Communication**: Shared memory for high-speed data transfer between M7 and M4

## Software Requirements

### Data Acquisition System (STM32H747XI/Arduino)

#### Core Functionality
1. **ADC Control**:
   - Configure ADC1+ADC2 in simultaneous regular conversion mode
   - Hardware trigger from timer for precise 10kHz sampling
   - DMA-based data transfer to minimize CPU overhead
   - Automatic channel sequencing for all 8 channels

2. **Data Management**:
   - Circular buffer implementation with ping-pong buffering
   - Real-time data integrity checking
   - Timestamp generation for each sample batch
   - Buffer overflow protection and reporting

3. **User Interface**:
   - User button (Arduino Opta built-in) for start/stop data logging
   - LED indicators for system status (sampling active, data logging, error states)
   - Serial interface for diagnostics and reduced-rate visualization

#### Visualization & Diagnostics
1. **Arduino IDE Serial Plotter Integration**:
   - Transmit decimated data at 100Hz (every 100th sample)
   - Format: CSV-style output for up to 4 channels simultaneously
   - Channel selection capability via serial commands
   - Real-time plotting of selected channels

2. **Diagnostic Information** (Terminal Window):
   ```
   === ADC Acquisition Diagnostics ===
   Sampling Rate: 10000.0 Hz (Target: 10000.0 Hz) [±0.01%]
   Maximum Jitter: ±23 ns (Spec: ±50 ns)
   Missed Samples: 0 (Total: 1,250,000) [0.000%]
   Sampling Status: ACTIVE | STOPPED | ERROR
   Data Logging: ENABLED | DISABLED
   Buffer Usage: 67.3% (2/3 buffers available)
   ADC Channels: A0:4095 A1:2048 A2:1024 A3:512 A4:256 A5:128 A6:64 A7:32
   Input Voltages: 10.00V 5.00V 2.50V 1.25V 0.63V 0.31V 0.16V 0.08V
   Core Usage: M7:45% M4:23%
   Temperatures: M7:45°C M4:42°C Ambient:25°C
   Uptime: 00:02:05.234
   Data Integrity: OK (CRC errors: 0)
   ```

3. **Error Monitoring**:
   - ADC overrun detection and reporting
   - DMA error detection
   - Buffer overflow warnings
   - Timing deviation alerts
   - System health monitoring

### PC Data Acquisition Software (Python)

#### Communication Interface
- **Protocol**: USB Serial (115200 baud minimum, prefer 921600 baud for high data rates)
- **Data Format**: CSV format with timestamp, raw ADC values, and calculated voltages
- **Handshaking**: Command/response protocol for reliable data transfer
- **Flow Control**: Software-based XON/XOFF to prevent data loss

#### Python Application Features
1. **Data Capture**:
   - Automatic detection of Arduino Opta device
   - Configurable acquisition duration or continuous mode
   - Real-time data rate monitoring
   - Data integrity verification

2. **Data Storage**:
   - **Primary Format**: CSV files with headers: `Timestamp_ms, A0_raw, A0_volts, A1_raw, A1_volts, ..., A7_raw, A7_volts`
   - **Data Precision**: Timestamp in milliseconds (float), ADC raw values (uint16_t), Voltages with 4 decimal places
   - **File Management**: Automatic file naming with timestamp (e.g., `ADC_Data_20250916_143022.csv`)
   - **Session Duration**: Optimized for 10-second acquisitions (100,000 samples per channel)

3. **Real-time Monitoring**:
   - Live plot of selected channels during acquisition
   - Data rate and quality indicators
   - System status display
   - Buffer level monitoring

#### Python Requirements File (requirements.txt):
```
numpy>=1.21.0
matplotlib>=3.5.0
pandas>=1.3.0
pyserial>=3.5
tkinter  # Usually included with Python
scipy>=1.7.0
datetime  # Built-in Python module
```

## Performance Specifications

### Timing Performance
- **Sampling Accuracy**: ±0.01% frequency stability
- **Channel-to-Channel Skew**: <1 ADC clock cycle (typically <100ns)
- **Long-term Stability**: <10 ppm drift over 24 hours
- **Settling Time**: Full-scale step response within 1 sample period

### Data Throughput
- **Raw Data Rate**: 160 kB/s (8 channels × 2 bytes × 10 kHz)
- **CSV Data Rate**: ~400 kB/s including timestamps and voltage calculations  
- **USB Serial Transfer Rate**: Minimum 460 kB/s sustained (4x overhead margin)
- **Maximum Recording Time**: 10 seconds per session (typical), unlimited sessions
- **Buffer Depth**: 2 seconds at full rate (640 kB total)

## Safety and Reliability Requirements

### Industrial Environment Compliance
- **Operating Temperature**: -40°C to +85°C
- **Input Protection**: Overvoltage protection on all analog inputs
- **ESD Protection**: IEC 61000-4-2 compliance
- **EMI Immunity**: IEC 61000-4-3 compliance

### Error Handling
- **Graceful Degradation**: Continue operation with reduced channels if individual ADC fails
- **Automatic Recovery**: System restart capability after recoverable error conditions
- **Data Validation**: Real-time CRC or checksum verification of critical data paths
- **Buffer Overflow Protection**: Immediate error indication and controlled shutdown
- **Watchdog Timer**: Independent monitoring of both CPU cores with 1-second timeout

### Diagnostics and Maintenance
- **Built-in Self Test**: Power-on diagnostics of all subsystems
- **Calibration Verification**: Periodic accuracy checks
- **Performance Logging**: Historical performance data storage
- **Remote Monitoring**: Status reporting via communication interface

## Development and Testing Requirements

### Code Quality Standards
- **Documentation**: Comprehensive inline comments and API documentation
- **Coding Standard**: Arduino/STM32 HAL best practices
- **Version Control**: Git with tagged releases
- **Testing**: Unit tests for critical functions

### Validation Testing
- **Accuracy Testing**: Verify against precision signal generator
- **Stress Testing**: 24-hour continuous operation validation
- **Performance Testing**: Confirm timing specifications under load
- **Environmental Testing**: Operation across temperature range

### Deliverables
1. **Arduino Sketch**: Complete STM32H747XI firmware
2. **Python Application**: Complete PC-side acquisition software
3. **Documentation**: User manual, API reference, setup guide
4. **Test Results**: Performance validation report
5. **Schematic**: Connection diagram for Arduino Opta setup

## Optional Enhancements (Future Versions)
- **Web Interface**: Browser-based monitoring and control
- **Data Processing**: Real-time FFT analysis and filtering
- **Multiple Device Support**: Synchronization of multiple Arduino Opta units
- **Cloud Integration**: Data upload to cloud storage/analysis platforms
- **Advanced Triggering**: External trigger support for event-based acquisition