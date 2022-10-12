BlueNRG-LP secure bootloader utilities:

1) key_generation.exe: this utility generates the public and private keys for the RSA-2048 algorithm, using the OPENSSL commands.
   NOTEs: the OPENSSL SW must be installed and the path must be added on Windows environment variables.
2) computeR2.exe: it is used by the key_generation.exe utility: to be not used in standalone mode. 
2) create_signed_bin.exe: this utility creates a signed FW starting from a binary file.
3) store_key_OTP.exe: this utility stores all the information inside the OTP area. 

Refer to "The BlueNRG-LP UART bootloader protocol (AN5471)" for detailed information about how to use the secure bootloader utilities.
