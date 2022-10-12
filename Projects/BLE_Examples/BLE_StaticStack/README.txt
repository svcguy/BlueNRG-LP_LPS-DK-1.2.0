----------------------------
How to build a bluenrg static stack library
----------------------------
The BlueNRG static stack is provided inside the BlueNRG-LP DK in binary format (i.e. at a fixed
address in Flash) and usually there is no need to build it again.
If you want to customize and build it, the following steps need to be followed.

------
Prerequisites
------
Install GNU ARM Embedded Toolchain and add the binary folder in the system PATH. The utility
create_sym_lib.exe in BlueNRG-1 SDK Utility folder needs the following utilities:
- arm-none-eabi-readelf 
- arm-none-eabi-gcc
- arm-none-eabi-ar
Open the Windows Command Prompt and try to invoke those commands to check if they are correctly
installed.

------
Build steps
------
- Build "BLE Static Stack" project. If necessary, change MEMORY_FLASH_APP_SIZE (for linker) to increase
  (or possibly reduce) the flash reserved for the BlueNRG Stack.
- If MEMORY_FLASH_APP_SIZE has been changed, change also RESET_MANAGER_SIZE (for C preprocessor).
- Use create_sym_lib.exe utility to generate the required library with symbols to be referenced by the
  application. See post-build step used in BLE Static Stack project.
- Take a note of the first available address in RAM (excluding CSTACK) from map file. (e.g. 0x200007AC).
  It has to be used when defining MEMORY_RAM_APP_OFFSET inside application project.
   
