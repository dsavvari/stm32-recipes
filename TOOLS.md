
# Flashing B-L4S5I-IOT01A with OpenOCD on macOS

## Requirements

- OpenOCD installed (`brew install open-ocd`)

- USB connection (onboard ST-Link V3)

- Firmware file (e.g., `.elf`, `.bin`, or `.hex`)

---

## Connecting the Board

The **B-L4S5I-IOT01A** has an onboard ST-Link V3. Simply plug it into your Mac using a ****data-capable**** USB cable.

---

## Basic OpenOCD Command

```bash

openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "init" \
  -c "reset halt" \
  -c "program path/to/your.elf verify reset exit"

```

For `.bin` files:

```bash

openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "program path/to/your.bin 0x08000000 verify reset exit"

```

## Example Full Command

```bash

openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "init" \
  -c "reset halt" \
  -c "program bareMetalBlink/stm32l4-bareMetalBlink.elf verify reset exit"

```

The stlink.cfg file is one of OpenOCD’s built-in interface configuration files. You don’t need to write it yourself.f — it comes with the OpenOCD installation.

---

## Example OpenOCD Session listening on port 3333

```bash
 openocd -f interface/stlink.cfg -f target/stm32l4x.cfg

```

This will start a server that listens on port 3333. You can then connect to it using GDB or other tools.

---

## Troubleshooting

### 1. Check if ST-Link is Detected

Run:

```bash

system_profiler SPUSBDataType | grep -i ST

```

Expected output:

```

STMicroelectronics ST-LINK

```

If not found, check cable or USB port.

---

### 2. Specify Transport and Speed

```bash

openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "transport select hla_swd" \
  -c "adapter speed 1000" \
  -c "init" \
  -c "reset halt" \
  -c "program path/to/your.elf verify reset exit"

```

---

### 3. Enable Verbose Debugging

```bash

openocd -d3 -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "init" \
  -c "reset halt"

```

Watch for errors like:

```

Error: target not halted

Error: timed out while waiting for target halted

```

---

### 4. Try Connect Under Reset

```bash

openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -c "init" \
  -c "reset halt"

```

---

### 5. Try ST Tools (st-util)

Install and test with **STM32CubeProgrammer** to confirm that the board and ST-Link work independently of OpenOCD.

st-util is a command-line tool that can be used to communicate with ST-Link. It can be installed via Homebrew:

```bash 
brew install stlink

```

Then run:

```bash 
st-util

```

This will start a server that listens on port 4242. You can then connect to it using GDB or other tools.

---

## Use GDB to connect to OpenOCD or st-util



```bash

gdb-multiarch path/to/your.elf

target extended-remote :4242

set substitute-path /path /to_path  'do this if you have a different path'

monitor reset halt

load

```


## VSCode Configuration 

### Launch Configuration 
Install the cortex-debug extension in VSCode. Create a `.vscode/launch.json` file with the following content:

{
    "version": "2.0.0",
    "configurations": [
        {
            "name": "GDB Debug with st-util or openocd",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "external",
            "executable": "/Users/dimitris/PROJECTS/stm32-recipes/t/bareMetalBlink/stm32l4-bareMetalBlink.elf", // Ensure this path is correct
            "gdbTarget": "localhost:3333",
            "device": "STM32L4S5VI", 
            "overrideLaunchCommands": [
                "target remote localhost:3333",
                "set substitute-path /repo /Users/dimitris/PROJECTS/stm32-recipes",
                "monitor reset halt",
                "load",
                "b main"
            ],
        },
    ],
  
}

You will also need to tell cortex-debug the location of arm-none-eabi-gdb. You can do this by adding the following to your settings.json file:

```json 
{
    "cortex-debug.armToolchainPath": "/path/to/arm-none-eabi-gdb"

}


Then add the following openocd or st-util configuration to your task.json file:

The first one will start an OpenOCD server and the second one will flash the STM32 device.

You will need to adjust the paths to the OpenOCD configuration files and the path to your firmware file.

You will also need to run the first task before starting a debug session, using the launch.json file above.

{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Start OpenOCD Server for STM32",
            "type": "shell",
            "command": "openocd",
            "args": [
                "-f", "interface/stlink.cfg",
                "-f", "target/stm32l4x.cfg"
            ],
            "problemMatcher": [],
            "isBackground": true,
            "group": {
                "kind": "build",
                "isDefault": false
            },
            "detail": "Start OpenOCD server for STM32 debugging"
        },
        {
            "label": "Flash STM32",
            "type": "shell",
            "command": "openocd",
            "args": [
                //∏"-d3",
                "-f", "interface/stlink.cfg",
                "-f", "target/stm32l4x.cfg",
                "-c", "init",
                "-c", "reset halt",
                "-c", "program /Users/dimitris/PROJECTS/stm32-recipes/t/bareMetalBlink/stm32l4-bareMetalBlink.elf verify reset exit"
            ],
            "problemMatcher": [],
            "isBackground": false,
            "group": {
                "kind": "build",
                "isDefault": false
            },
            "detail": "Flash a binary to STM32 using OpenOCD"
        }
    ]

}

---

## Helpful Paths

### OpenOCD Script Location (macOS)

```bash

/opt/homebrew/share/openocd/scripts/interface/stlink.cfg

/opt/homebrew/share/openocd/scripts/target/stm32l4x.cfg

```

---
