#ifndef SOLOMOTO_RDRIVER_H
#define SOLOMOTO_RDRIVER_H

#include "SOLOMotorControllersUart.h"
#include <string>
#define HWSERIAL Serial1 // Macro, you can replace this with your microcontroller's UART

class SoloBrushlessMotor
{
public:
    /**************************************ENUM******************************************/
    // Enum to represent baud rate options
    enum class BaudRate
    {
        RATE_115200 = 0,
        RATE_937500 = 1
    };
    /************************************************************************************/
    /*##################################################################################*/
    /**********************************METHODS/FUNCTIONS*********************************/
    /**
     * @brief   Constructor: Setting related to SOLO
     * @param   <_deviceAddress> Device address for identification, {0~255}
     * @param   <_serial> Serial Port to be used, has to be type HardwareSerial
     * @param   <_baudRate> 115200 or 937500, most microcontroller allow 115200
     * @param   <_millisecondsTimeout> milliseconds until timeout
     * @param   <_packetFailureTrialAttempts> number of attempts if the packet failed.
     * @return  None
     */
    SoloBrushlessMotor(    
    unsigned char _deviceAddress = 0, HardwareSerial &_serial = HWSERIAL, 
    BaudRate _baudRate = BaudRate::RATE_115200, long _millisecondsTimeout = 50, int _packetFailureTrialAttempts = 5);


    /**
     * @brief   Initialization: Motor related. Initialize SOLO with pwm, currentlimit, number of poles, by default, using hall
     * @param   <myCurrentLimit> Currentlimit of motor
     * @param   <myNumberOfPoles> Pole number of motor, double if using pair as unit
     * @return  None
     * @note    PWM frequency for communication is set by default to 20kHz this will work for most setups
     */
    void init(float myCurrentLimit = 6, long myNumberOfPoles = 4);

    /**
     * @brief   Set reference current Iq with specified direction
     * @param   <current> desired reference current (A)
     * @return  None
     */
    void setCurrent(double current);

    /**
     * @brief   Auto Calibrate SOLO current loop Kp and Ki, only need to be run with one setup. 
     */
    void calibration();

    /**
     * @brief   Print all Information Of Board at once
     * @return  Current Readings in A
     */
    void getInformation();

    /**
     * @brief   Get quadrature current (Iq).
     * @return  Quadrature current (Iq).
     */
    float getQuadratureCurrentIq();

    /**
     * @brief   Get torque reference current (Iq).
     * @return  Torque reference current (Iq).
     */
    float getTorqueReferenceIq();

    /**
     * @brief   Get current limit.
     * @return  Current limit.
     */
    float getCurrentLimit();

    /**
     * @brief   Get phase A current.
     * @return  Phase A current.
     */
    float getPhaseACurrent();

    /**
     * @brief   Get phase B current.
     * @return  Phase B current.
     */
    float getPhaseBCurrent();

    /**
     * @brief   Get phase A voltage.
     * @return  Phase A voltage.
     */
    float getPhaseAVoltage();

    /**
     * @brief   Get phase B voltage.
     * @return  Phase B voltage.
     */
    float getPhaseBVoltage();

private:
    SOLOMotorControllers *myMotor;

    // the device address of SOLO:
    unsigned char SOLO_address1 = 0;

    // Motor's Number of Poles
    long numberOfPoles = 4;

    // Current Limit of the Motor
    float currentLimit = 2.7;

    // Battery of Bus Voltage
    float busVoltage = 0;

    // Desired Speed Reference
    long desiredMotorSpeed = 0;

    // Motor speed feedback
    long actualMotorSpeed = 0;

    // Define Desired Torque referrrence
    float desiredTorque_Iq = 1.5;

    // Motor Torque feedback
    float actualMotorTorque = 0;

    // Max Measureable Current
    float MaxMeasurableCurrent_SOLO_UNO = 2.7;

};

#endif

