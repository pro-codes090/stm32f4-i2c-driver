
# stm32f4-I2C-driver
I2C Bare metal drivers for stm32 f4 family of microcontrollers written in C from scratch .The driver contain apis for the user to send and receive data i 12c communication . The driver has two flavor of apis polling based (blocking ) and Interrupt based (non blocking ) check  examples in the Src folder for more reference .   

## Project file structure 

**Src** :   The Src folder contains the main application file ( main.c )  .  The user might change the contents of the file if needed . The folder contains example code each example with its own source file . 

**drivers** : folder contains further sub folder  **Src** and **Inc**   .
- **Src** : Src folder contains driver source (.c) file which has all the driver apis the application file uses 
- **Inc** : Inc folder contains the  driver header (.h) file which has all the function prototypes and configuration macros .  


# driver apis Usage  
All the examples and code have been tested on [Stm32f407vgt6 disc1 board](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) if you own a different board from st of F4 family no changes should be made to the code 
##
 Check out this video for a video guide on how to use the apis and many other things [Stm32 I2C Communication bare metal from scratch](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) 
