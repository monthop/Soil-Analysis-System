# Soil-Analysis-System

An arduino program to output readings of 4 different soil sensors using the Arduino Uno.

Combined1.ino is the code for all four sensors put together.

The four sensors in the code by description are:
//EC meter
//pH sensor
//DHT22 temperature sensor
//Moisture Sensor


Currently I am having trouble outputting all of my data to one baud rate to view in the Serial Moniter window using the Arduino 1.7.1 IDE. I can only view all of the sensor data by switching the data baud rate values (which can be selected in the bottom right corner of the Serial Moniter window) to their respective sensors baud rate values. The sensors with baud rates are listed here and are in the void setup section of the Combined1.ino file. I am eventually going to output all of the 4 sensor values to an LCD display. If I don't need to synchronize all of the baud rates to output it onto the LCD screen then that's OK too. It would just be nice to verify my numbers somehow. 

[Sensors with different baud rates in void setup]  
//EC meter. Reads ms/cm
Serial.begin(115200);

//pH meter. Reads pH value
Serial.begin(9600);

//DHT22 temperature sensor. Reads Temperature
Serial.begin(9600);

//Moisture sensor. Reads "Moisture Sensor Value:" <- Not showing up in Serial Moniter
Serial.begin(57600);
