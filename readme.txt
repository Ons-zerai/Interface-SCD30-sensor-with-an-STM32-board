
                                             /***** A basic example of Reading CO2 measurement from SCD30 sensor *******/ 
                                             ***************************************************************************

    /// This project was build using STM32CubeIDE with HAL librairies.///

The STM32WL board communicates with the SCD30 sensor via I2C : all we need is to configure the I2C interface and the UART to 
display data in the terminal.


/****Procedure *****/
 
First of all, we need to tell sensor to start measurement by filling the buffer "" cnt measurement " with the appropriate commands,
then, we check if there's a measurement ready to be read or not, if yes we tell the sensor that we want to read the current data 
by filling the buffer "" cnt measurement "" with the appropriate commands. Finally, we display the data using the virtual com port VCP.


                  /*************** About the sensor *******************/

                   Sensor address ( 7 bits )            :: 0x61 
                   Sensor address with read argument:1  :: 0xC3
                   Sensor address with write argument:0 :: 0xC2
                   Command that starts measurement      :: 0x0010 
                   Command that gets  ready status      :: 0x0202
                   Command that reads measurement       :: 0x0300





                  /*************** I2C connection *********************/
 

                           SCD30 SENSOR    ||    STM32WL
                       ---------------------------------------
                              VIN                 3.3V
                              GND                 GND 
                              SDA                 PB7
                              SCL                 PB8
                      -----------------------------------------