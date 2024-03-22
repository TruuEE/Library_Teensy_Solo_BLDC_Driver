#include "SoloBrushlessMotor.h"
#include <iostream>

// Change the Pin Based on Your Design
#define TORQUE_PIN  2
#define CURRENT_LIMIT_PIN  3
#define DIRECTION_PIN 4

SoloBrushlessMotor::SoloBrushlessMotor(
    unsigned char _deviceAddress, 
    HardwareSerial &_serial, 
    BaudRate _baudRate, 
    long _millisecondsTimeout, 
    int _packetFailureTrialAttempts)
{
    // Define baud rate for using library
    SOLOMotorControllers::UartBaudrate _baudrateSOLO;
    if (_baudRate == BaudRate::RATE_937500)
        _baudrateSOLO = SOLOMotorControllers::UartBaudrate::RATE_937500;
    else if (_baudRate == BaudRate::RATE_115200)
        _baudrateSOLO = SOLOMotorControllers::UartBaudrate::RATE_115200;
    else
    {
        Serial.println("Invalid baud rate, set to default");
        _baudrateSOLO = SOLOMotorControllers::UartBaudrate::RATE_115200;
    }

    myMotor = new SOLOMotorControllersUart(_deviceAddress, _serial, _baudrateSOLO, _millisecondsTimeout, _packetFailureTrialAttempts);
    delay(1000);
}

void SoloBrushlessMotor::init(float myCurrentLimit, long myNumberOfPoles)
{
    currentLimit = myCurrentLimit;

    Serial.println("\n Trying to Connect To SOLO");
    delay(1000);
    // wait here till communication is established
    while (myMotor->CommunicationIsWorking() == false)
    {
    delay(500);
    }
    Serial.println("\n Communication Established succuessfully!");


    // Initial Configuration of the device and the Motor
    myMotor->SetOutputPwmFrequencyKhz(20);
    myMotor->SetCurrentLimit(myCurrentLimit);
    myMotor->SetMotorPolesCounts(myNumberOfPoles);
    myMotor->SetCommandMode(SOLOMotorControllers::CommandMode::DIGITAL);
    myMotor->SetMotorType(SOLOMotorControllers::MotorType::BLDC_PMSM);
    myMotor->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::HALL_SENSORS);
    // Mode for torque is being set using hardware pins

    // Set to Analogue Control
    myMotor->SetCommandMode(SOLOMotorControllers::CommandMode::ANALOGUE);

    /***Set up PINS**************************************************************/
    pinMode(DIRECTION_PIN, OUTPUT); // Direction Control Pin

    // declare PWM enabled output with 31kHz of fixed Frequency
    analogWriteFrequency(TORQUE_PIN, 31000);
    analogWriteFrequency(CURRENT_LIMIT_PIN, 31000);

    // declare resolution
    analogWriteResolution(8);
    /****************************************************************************/
}

void SoloBrushlessMotor::setCurrent(double current)
{
    // Define the desired torque in Amps
    double desiredTorque_Iq = abs(current);
    Serial.print("Desired Torque (Iq): ");
    Serial.println(desiredTorque_Iq);

    // Defnine the direction, TRUE means CW, FALSE means CCW
    bool clockWise;
    if (current >= 0)
        clockWise = true;
    else
        clockWise = false;

    // Define the Direction of Rotation
    if (clockWise)
        digitalWrite(DIRECTION_PIN, LOW); // CW
    else
        digitalWrite(DIRECTION_PIN, HIGH); // CCW

    // Converted value to PWM duty cycle for desired Iq on 8 bit PWM of Arduino
    int desiredDutyCycle_Iq = (int)(255 * (desiredTorque_Iq / currentLimit));

    // Set the right Duty cycle on Pin ~3 for Torque
    analogWrite(TORQUE_PIN, desiredDutyCycle_Iq);

    // set a new reference for torque [RPM]
    myMotor->SetTorqueReferenceIq(abs(current));

}

void SoloBrushlessMotor::calibration()
{
    // run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
    // run ID. always after selecting the Motor Type!
    // ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
    // the ID. values will be remembered by SOLO after power recycling
    myMotor->MotorParametersIdentification(SOLOMotorControllers::Action::START);
    Serial.println("\n Identifying the Motor");
    // wait at least for 2sec till ID. is done
    delay(2000);
}

void SoloBrushlessMotor::getInformation()
{
    float quadratureCurrentIq = myMotor->GetQuadratureCurrentIqFeedback();
    float torqueReferenceIq = myMotor->GetTorqueReferenceIq();
    float currentLimit = myMotor->GetCurrentLimit();
    float phaseACurrent = myMotor->GetPhaseACurrent();
    float phaseBCurrent = myMotor->GetPhaseBCurrent();
    float phaseAVoltage = myMotor->GetPhaseAVoltage();
    float phaseBVoltage = myMotor->GetPhaseBVoltage();
    
    Serial.print("Quadrature Current Iq: ");
    Serial.println(quadratureCurrentIq);

    Serial.print("Torque Reference Iq: ");
    Serial.println(torqueReferenceIq);

    Serial.print("Current Limit: ");
    Serial.println(currentLimit);

    Serial.print("Phase A Current: ");
    Serial.println(phaseACurrent);

    Serial.print("Phase B Current: ");
    Serial.println(phaseBCurrent);

    Serial.print("Phase A Voltage: ");
    Serial.println(phaseAVoltage);

    Serial.print("Phase B Voltage: ");
    Serial.println(phaseBVoltage);

    Serial.println();
}

float SoloBrushlessMotor::getQuadratureCurrentIq()
{
    return myMotor->GetQuadratureCurrentIqFeedback();
}

float SoloBrushlessMotor::getTorqueReferenceIq()
{
    return myMotor->GetTorqueReferenceIq();
}

float SoloBrushlessMotor::getCurrentLimit()
{
    return myMotor->GetCurrentLimit();
}

float SoloBrushlessMotor::getPhaseACurrent()
{
    return myMotor->GetPhaseACurrent();
}

float SoloBrushlessMotor::getPhaseBCurrent()
{
    return myMotor->GetPhaseBCurrent();
}

float SoloBrushlessMotor::getPhaseAVoltage()
{
    return myMotor->GetPhaseAVoltage();
}

float SoloBrushlessMotor::getPhaseBVoltage()
{
    return myMotor->GetPhaseBVoltage();
}