The project BLE_StackDevConfigUpdater_NWK configurations: 
 - Stack_Updater_UART
 - Stack_Updater_SPI
 use a fixed image of the DTM (file update_fw_image.c) to update the target device.
 
The file update_fw_image.c contains the image from DTM project, configurations:
  - UART_FOR_UPDATER
  - SPI_FOR_UPDATER
  
It is possible to generate a new file update_fw_image.c by using the convert_bin2c.exe script.
The usage is:
 - UART version: convert_bin2c.exe -i DTM_UART_FOR_UPDATER.bin -o update_fw_image.c
 or
 - SPI version: convert_bin2c.exe -i DTM_SPI_FOR_UPDATER.bin -o update_fw_image.c
 where the files DTM_UART_FOR_UPDATER.bin and DTM_SPI_FOR_UPDATER.bin are input binary image that the script translate in a c array.
