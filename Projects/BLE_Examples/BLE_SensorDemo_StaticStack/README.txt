----------------------------------------------------------------------
Instructions to build an application that uses the BLE Static Stack project
----------------------------------------------------------------------

----------------------------
How to compile application
----------------------------

- Remove libbluenrg_lp_stack.a and stack_user_cfg.c from project files.
- Remove all the external modules and drivers, which have been included in static stack (their
  symbols are exported and can be used by application):
    - AES
    - BLECNTR
    - Cryptolib
    - NVM
    - PKA
    - RNG
    - GAP and GATT profiles
    - HAL_VTimer
- Include bluenrg_lp_stack_init_if.c and libbluenrg_lp_static_stack.a in your project
  (from Middlewares\ST\Bluetooth_LE\Library\static_stack).
- Define MEMORY_FLASH_APP_OFFSET for linker, depending on the Flash size occupied by the BlueNRG
  static stack image. E.g. if static stack image uses the Flash till address 0x1005a438, offset
  must be equal to 0x1A800, which is the offset of the first available Flash location aligned to
  the start of a page.
- Define MEMORY_RAM_APP_OFFSET for linker, in order to allocate variables in free RAM space,
  where no other variables have been allocated by the static stack library.
  See MEMORY_RAM_APP_OFFSET used inside BLE_SensorDemo_Static_Stack project to know which RAM offset
  must be used by your application when using the Static Stack binary included insided the BlueNRG-LP DK.
- Use the same modular configuration used for static stack (e.g. BLE_STACK_FULL_CONF if static stack has been
  built with BLE_STACK_FULL_CONF, use BLE_STACK_BASIC_CONF if static stack has been built with BLE_STACK_BASIC_CONF).

If OTA bootloader is used, RESET_MANAGER_SIZE must be defined instead of MEMORY_FLASH_APP_OFFSET for linker.
In addition, RESET_MANAGER_SIZE macro must be defined for C preprocessor. Both macros must have the same value.

