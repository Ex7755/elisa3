
#ifndef IR_COMMUNICATION_H
#define IR_COMMUNICATION_H


/**
 * \file irCommunication
 * \brief IR local communication module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.12.14
 * \copyright GNU GPL v3

 The proximity sensors can be used to implement a local communication using the infrared light.
 A simple communication protocol is used: "0" and "1" are encoded with different signal frequencies, moreover 
 2 start bits (used to detect the data and then to sync with it) and 2 CRC bits are used for each byte, that is 
 for each byte 12 bits are exchanged.
 When the IR local communication is initialized the robot starts listening for incoming data; when the user decides 
 to send a byte the robot stops listening and starts transmitting the required data and then continues listening.
 The proximity and ground sensors can still be used when the IR local communication is enabled, this means that the 
 robot is in principle able to avoid obstacles, but beware that the sensors update frequency is reduced.
 The throughput of the IR local communication is about 1 bytes/sec when the robots are continuously receiving and 
 transmitting data.
 The IR local communication is implemented using a state machine, in order for the state machine to work the function "irCommTasks"
 need to be called as fast as possible (put it in you main loop).
 Limitations:
 - no RX or TX queue (one byte at a time): use the function "irCommDataSent" to know when the next byte can be sent and 
 the function "irCommDataAvailable" when the next data can be read
 - reduced sensors frequency update: in the worst case (cotinuously receiving and transmitting data) is about 3 Hz; 
 this means that the grounds cannot be used for cliff avoidance
 - the data are sent using all the sensors, cannot select a single sensors from which to send the data. Moreover the data isn't sent
 contemporaneously from all the sensors, but the sensors used are divided in two groups of 4 alternating sensors; this is to reduce 
 the istantaneous power consumption required by all the sensors.
 - maximum communication distance is about 5 cm

*/


#include "variables.h"
#include "utility.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize the IR communication; need to be called only once.
 * \return none
 */
void irCommInit();

/**
 * \brief Initialize the IR communication state machine in reception mode (used internally).
 * \return none
 */
void irCommInitReceiver();

/**
 * \brief Initialize the IR communication state machine in transmission mode (used internally).
 * \return none
 */
void irCommInitTransmitter();

/**
 * \brief Stop the IR communication, only sensors sampling for obstacle and cliff avoidance is enabled.
 * \return none
 */
void irCommDeinit();

/**
 * \brief Handle the internal state machine of the IR communication. Need to be called repeatedly from the main loop.
 * \return none
 */
void irCommTasks();

///**
// * \brief Set the data to be sent through IR; the obstacle avoidance and cliff avoidance will be disabled during the transmission.
// * \param value: byte to be sent; sensor: sensor id mask (bit 0 corresponds to sensor 0 in front of robot, id increases clockwise).
// * \return none
// */
//void irCommSendData(unsigned char value, unsigned char sensorMask);

/**
 * \brief Set the data to be sent through IR; the obstacle avoidance and cliff avoidance will be disabled during the transmission.
 * \param value: byte to be sent; sensor.
 * \return none
 */
void irCommSendData(unsigned char value);

/**
 * \brief Tell whether the last transmission is terminated or not.
 * \return 1 if the last byte is sent, 0 otherwise
 */
unsigned char irCommDataSent();

/**
 * \brief Tell whether incoming data are received or not.
 * \return 1 if one byte is received, 0 otherwise
 */
unsigned char irCommDataAvailable();

/**
 * \brief Get the last byte received through IR.
 * \return the value of the byte received
 */
unsigned char irCommReadData();

/**
 * \brief Get the last sensor id that receives the message.
 * \return the sensor id (0..7); 0 is the front sensor, sensors id increases clockwise
 */
signed char irCommReceivingSensor();

/**
 * \brief Get the direction of the sensor that receives the last message.
 * \return the angle of the sensor (-180..180)
 */
signed int getBearing(unsigned char sensor);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
