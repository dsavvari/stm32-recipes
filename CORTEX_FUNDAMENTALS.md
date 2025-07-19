# 🧠 Cortex-M Fundamentals Exploration Projects

This directory contains educational projects designed to explore fundamental ARM Cortex-M features in a hands-on way. Each project focuses on specific low-level concepts that are essential for embedded systems development.

## 📚 Project Overview

### 1. **cortexSysTick** - System Timer and Timing Fundamentals
**Focus:** SysTick timer, system timing, delay functions, tick-based scheduling

🎯 **What You'll Learn:**
- SysTick timer configuration and operation
- System tick generation and timing accuracy
- Millisecond delay implementation
- Tick-based task scheduling concepts
- Timer interrupt handling basics
- System clock and timing relationships

🔧 **Hardware Demo:**
- LED1 (PA5): 1Hz blinking using SysTick delays
- LED2 (PB14): 2Hz blinking with different timing
- External LED (PA0): 4Hz blinking showing precise timing
- Button (PC13): Timing measurement and responsiveness test
- Demonstrates precise timing control and interrupt-driven timing

**Key Features:**
- SysTick timer configuration for 1ms ticks
- Non-blocking delay functions
- Tick counter and overflow handling
- Multiple concurrent timing operations
- Foundation for RTOS-style scheduling

### 2. **cortexExceptions** - Exception and Interrupt Handling
**Focus:** NVIC, interrupt priorities, exception handlers

🎯 **What You'll Learn:**
- Exception vs Interrupt concepts
- NVIC (Nested Vector Interrupt Controller) configuration
- Interrupt priorities and preemption
- Critical sections and interrupt masking
- Exception handlers and vector table

🔧 **Hardware Demo:**
- LED1 (PA5): SysTick controlled (1Hz, highest priority)
- LED2 (PB14): Timer2 controlled (2Hz, medium priority)
- External LED (PA0): Button controlled (lowest priority)
- Button (PC13): Triggers external interrupt
- Watch how higher priority interrupts preempt lower ones

**Key Features:**
- Multiple interrupt sources with different priorities
- Demonstrates priority-based preemption
- Safe critical section implementation
- Fault handlers with debugging information

### 3. **cortexPowerModes** - Power Management and Sleep Modes
**Focus:** ARM Cortex-M power modes, WFI/WFE, STM32L4 low-power features

🎯 **What You'll Learn:**
- ARM Cortex-M power modes (Normal, Sleep, Deep Sleep)
- WFI (Wait For Interrupt) vs WFE (Wait For Event)
- STM32L4 specific power modes (Stop 0/1, Standby)
- Wake-up sources and latency considerations
- Power consumption optimization techniques

🔧 **Hardware Demo:**
- LED1 (PA5): Power mode indicator (blink patterns)
- LED2 (PB14): Activity indicator (pulses on wake-up)
- External LED (PA0): Current measurement probe point
- Button (PC13): Mode switching and wake-up source
- Cycles through 6 different power modes

**Key Features:**
- Real-time power mode switching
- Wake-up latency measurements
- Current consumption measurement points
- Auto-sleep timeout functionality

### 4. **cortexMemoryMap** - Memory Architecture and Management
**Focus:** Memory regions, stack management, pointer arithmetic, memory protection

🎯 **What You'll Learn:**
- ARM Cortex-M memory map regions and their purposes
- Stack pointer (MSP vs PSP) management
- Memory protection concepts
- Linker script understanding
- Stack overflow detection
- Memory debugging techniques

🔧 **Hardware Demo:**
- LED1 (PA5): Memory access indicator
- LED2 (PB14): Stack operation indicator
- External LED (PA0): Memory protection violation indicator
- Button (PC13): Trigger memory operations
- Interactive memory exploration

**Key Features:**
- Complete memory map visualization
- Stack usage monitoring and overflow detection
- Controlled memory access tests
- Pointer arithmetic demonstrations
- Memory region validation

### 5. **cortexSystemControl** - MSP/PSP and CONTROL Register
**Focus:** Stack pointer management, privilege levels, system control

🎯 **What You'll Learn:**
- MSP (Main Stack Pointer) vs PSP (Process Stack Pointer)
- CONTROL register and privilege levels
- Thread mode vs Handler mode
- Privileged vs Unprivileged execution
- System Control Block (SCB) registers
- Stack switching and dual-stack operation
- CPU ID and feature detection

🔧 **Hardware Demo:**
- LED1 (PA5): Stack pointer indicator (MSP=solid, PSP=blink)
- LED2 (PB14): Privilege level indicator (Privileged=solid, Unprivileged=blink)
- External LED (PA0): CONTROL register bit pattern
- Button (PC13): Toggle between MSP/PSP and privilege levels
- Cycles through 4 different system states

**Key Features:**
- MSP/PSP switching with assembly helpers
- Privilege level changes (Privileged ↔ Unprivileged)
- Stack pointer monitoring and validation
- CONTROL register manipulation and analysis
- System information extraction (CPUID, features)

### 6. **cortexDebugTrace** - Debug and Trace Features
**Focus:** ITM, SWO, DWT, performance monitoring, debug capabilities

🎯 **What You'll Learn:**
- ITM (Instrumentation Trace Macrocell) for debug output
- SWO (Serial Wire Output) trace data transmission
- DWT (Data Watchpoint and Trace) for performance monitoring
- Cycle counter for precise timing measurements
- Function execution profiling
- Memory access pattern analysis
- Real-time trace data collection

🔧 **Hardware Demo:**
- LED1 (PA5): ITM activity indicator
- LED2 (PB14): Performance measurement indicator
- External LED (PA0): Watchpoint trigger indicator
- Button (PC13): Trigger measurements and trace events
- SWO Pin (PB3): Trace data output for debugger

**Key Features:**
- ITM channel-based debug output (no UART needed)
- DWT cycle counter for performance measurement
- Function execution timing and profiling
- Memory access monitoring and analysis
- Real-time trace streaming via SWO

## 🛠️ Building the Projects

### VS Code Integration:
**Tasks Available**: All Cortex-M projects are available in VS Code tasks
- **Build Project**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Build Project" → Select from dropdown
- **Flash STM32**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Flash STM32" → Select project to flash
- **Debug Project**: `F5` → Select project from dropdown for debugging

**Available Projects in VS Code**:
- `cortexSysTick` - System Timer and Timing Fundamentals
- `cortexExceptions` - Exception and Interrupt Handling
- `cortexPowerModes` - Power Management and Sleep Modes
- `cortexMemoryMap` - Memory Architecture and Management
- `cortexSystemControl` - MSP/PSP and CONTROL Register
- `cortexDebugTrace` - Debug and Trace Features
- Plus basic projects: `bareMetalBlink`, `bareMetalPWM`

### Build All Projects:
```bash
# Inside Docker container
cmake --build build

# Or build specific project
cmake --build build --target stm32l4-cortexSysTick
cmake --build build --target stm32l4-cortexExceptions
cmake --build build --target stm32l4-cortexPowerModes
cmake --build build --target stm32l4-cortexMemoryMap
cmake --build build --target stm32l4-cortexSystemControl
cmake --build build --target stm32l4-cortexDebugTrace
```

### Memory Usage:
```
Project              Text    Data    BSS     Total
cortexSysTick         1596    20      1992    3608
cortexExceptions     34192   1760    2388    38340
cortexPowerModes     35296   1760    2388    39444
cortexMemoryMap      39384   2784    2396    44564
cortexSystemControl  35300   1760    2388    38348
cortexDebugTrace     36200   1760    2388    39848
```

## 📋 Learning Path Recommendation

1. **Start with `cortexSysTick`** - Master system timing and SysTick fundamentals
2. **Progress to `cortexExceptions`** - Learn interrupt priorities and NVIC
3. **Explore `cortexPowerModes`** - Understand power management
4. **Deep dive with `cortexMemoryMap`** - Master memory architecture
5. **Advanced topics with `cortexSystemControl`** - Grasp stack pointers and privilege levels
6. **Expert level with `cortexDebugTrace`** - Master debugging and performance analysis

## 🔍 Key Cortex-M Concepts Covered

### SysTick Timer
- **System Timer**: 24-bit down-counter with auto-reload
- **Tick Generation**: Configurable tick rate (typically 1ms)
- **Interrupt Handler**: SysTick_Handler for periodic tasks
- **Timing Functions**: Delay implementation and time measurement

### Exception System
- **Exception Numbers 1-15**: System exceptions (Reset, NMI, HardFault, etc.)
- **Exception Numbers 16+**: External interrupts (EXTI, TIM, UART, etc.)
- **Priority Levels**: -3 (Reset) to configurable (0-15 on STM32L4)
- **Exception Handlers**: Proper interrupt service routine implementation

### Power Management
- **Run Mode**: Normal operation, all clocks active
- **Sleep Mode**: CPU stopped, peripherals running, wake on any interrupt
- **Deep Sleep**: Selected clocks stopped, wake on specific interrupts
- **Standby**: Minimal power, RAM lost, reset on wake-up

### Memory Architecture
- **Code Region** (0x00000000): Flash, ROM, Vector Table
- **SRAM Region** (0x20000000): Data, Stack, Heap
- **Peripheral Region** (0x40000000): APB/AHB Peripherals
- **System Region** (0xE0000000): Cortex-M System Control

### System Control and Debugging
- **MSP and PSP**: Main and Process Stack Pointer usage
- **CONTROL Register**: Privilege level and mode control
- **SCB Registers**: System Control Block register usage
- **ITM, SWO, DWT**: Debug and trace feature utilization

## 🧪 Experimentation Ideas

### SysTick Timer:
- Implement different tick rates (1ms, 10ms, 100ms)
- Create multiple concurrent timers using tick counters
- Measure timing accuracy and jitter
- Build a simple task scheduler using SysTick

### Exception Handling:
- Modify interrupt priorities and observe preemption behavior
- Implement custom fault handlers with detailed debugging
- Create nested interrupt scenarios
- Add more interrupt sources (UART, SPI, etc.)

### Power Management:
- Measure actual current consumption with multimeter
- Implement wake-up from different sources
- Create battery-powered scenarios
- Optimize for specific power budgets

### Memory Management:
- Implement simple memory allocator
- Create memory protection violations (safely)
- Explore different linker script configurations
- Implement stack canaries for overflow detection

### System Control and Debugging:
- Experiment with MSP/PSP switching and observe effects
- Toggle privilege levels and monitor behavior
- Analyze CONTROL register effects on system behavior
- Utilize ITM and SWO for advanced debugging scenarios

## 📖 Additional Resources

- **ARM Cortex-M Architecture Reference Manual**
- **STM32L4 Reference Manual** - Chapter on Power Control (PWR)
- **Joseph Yiu's "Definitive Guide to ARM Cortex-M"**
- **CMSIS Documentation** for standardized APIs

## 🚀 Next Steps

After mastering these fundamentals, you'll be ready for:
- RTOS integration (FreeRTOS, Zephyr)
- Advanced peripheral usage (DMA, advanced timers)
- Communication protocols (I2C, SPI, UART, CAN)
- Bootloader and firmware update mechanisms
- Real-time audio/signal processing

## 🎯 Hardware Testing

All projects are designed for the **STM32L4S5VI** on the **B-L4S5I-IOT01A** development board but can be easily ported to other STM32L4 devices by adjusting:
- GPIO pin assignments
- Memory sizes in linker scripts
- Peripheral base addresses (if needed)

The code uses CMSIS standard definitions for maximum portability across ARM Cortex-M devices.

---

*These projects provide a solid foundation for understanding ARM Cortex-M architecture before moving on to more complex embedded systems topics.*
