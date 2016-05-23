# Soil-Analysis-System

An arduino program to output readings of 4 different soil sensors using the Arduino Uno.

Combined1.ino is the code for all four sensors put together.

The four sensors in the code by description are:
//EC meter
//pH sensor
//DHT22 temperature sensor
//Moisture Sensor

Currently I am having trouble outputting all of my data onto one baud rate in the Serial Monitor window using the Arduino 1.7.1 IDE. I can only view all of the sensor data by switching the data baud rate values (selected in the bottom right corner of the Serial Monitor window) to their respective baud rate sensor values. The sensors with baud rates are listed in this email and are in the Combined1.ino file, in the void setup section. I am eventually going to output all of the 4 sensor values to an LCD display... If I don't need to synchronize all of the baud rates to output it onto the LCD screen then that would be great too. The Serial Monitor window is my only current testing procedure to view the values right now.

[Sensors with different baud rates in void setup]  
//EC meter. Reads ms/cm
Serial.begin(115200);

//pH meter. Reads pH value
Serial.begin(9600);

//DHT22 temperature sensor. Reads Temperature
Serial.begin(9600);

//Moisture sensor. Reads "Moisture Sensor Value:" <- Not showing up in Serial Moniter
Serial.begin(57600);
