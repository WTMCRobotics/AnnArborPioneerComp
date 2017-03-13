/*
 * CANTalonDriveTrain.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 */

#include <CANTalonDriveTrain.h>

CANTalonDriveTrain::CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro*  pGyro)
{
	m_pController = pController;
	m_pGyro		  = pGyro;
	m_pGyro->Calibrate();

	m_leftSpeed 	= 0.0;
	m_rightSpeed 	= 0.0;
	m_speedFactor	= 1.0;

#if defined(MODE_Voltage)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPercentVbus);
	m_rightMasterDrive.Set(0);

#elif defined(MODE_Speed)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_rightMasterDrive.Set(0.0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(DRIVE_ENCDR_STEPS *  4);
	m_rightMasterDrive.ConfigNominalOutputVoltage(+0.0f, -0.0f);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMasterDrive.SetVoltageRampRate(DRIVE_RAMP_VoltsPerSec);
	m_rightMasterDrive.SetSensorDirection(true);
	m_rightMasterDrive.SetPID(DRIVE_PID_P_GAIN, DRIVE_PID_I_GAIN, DRIVE_PID_D_GAIN);

	m_leftMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_leftMasterDrive.Set(0.0);
	m_leftMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_leftMasterDrive.ConfigEncoderCodesPerRev(DRIVE_ENCDR_STEPS * 4);
	m_leftMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_leftMasterDrive.SetVoltageRampRate(DRIVE_RAMP_VoltsPerSec);
	m_leftMasterDrive.SetSensorDirection(true);
	m_leftMasterDrive.SetPID(DRIVE_PID_P_GAIN, DRIVE_PID_I_GAIN, DRIVE_PID_D_GAIN);

#elif defined(MODE_Position)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPosition);
	m_rightMasterDrive.Set(0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
#else
#error No mode selected for TallonSRX
#endif

	m_rightSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_rightSlaveDrive.Set(CAN_ID_RIGHTMASTER);

	m_leftSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_leftSlaveDrive.Set(CAN_ID_LEFTMASTER);

}

CANTalonDriveTrain::~CANTalonDriveTrain()
{

}

void CANTalonDriveTrain::UpdateStats(void)
{
	m_leftSpeed  = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	m_leftPosition  = m_leftMasterDrive.GetPosition();
	m_rightPosition = m_rightMasterDrive.GetPosition();

	m_leftEncoderPos  = m_leftMasterDrive.GetEncPosition();
	m_rightEncoderPos = m_rightMasterDrive.GetEncPosition();

	m_leftEncoderVel  = m_leftMasterDrive.GetEncVel();
	m_rightEncoderVel = m_rightMasterDrive.GetEncVel();
}

void CANTalonDriveTrain::Stop(void)
{
	m_leftSpeed = 0.0;
	m_leftMasterDrive.Set(-m_leftSpeed);
	m_rightSpeed = 0.0;
	m_rightMasterDrive.Set(m_rightSpeed);
}

void CANTalonDriveTrain::Update(double leftCommand, double rightCommand, bool slowSpeed)
{
	m_leftTarget  = Deadband(leftCommand)  * DRIVE_MAX_SPEED * m_speedFactor;
	m_rightTarget = Deadband(rightCommand) * DRIVE_MAX_SPEED * m_speedFactor;


	if (slowSpeed)
	{
		m_leftTarget  *= kSlowSpeedFactor;
		m_rightTarget *= kSlowSpeedFactor;
	}

	m_leftMasterDrive.Set(-m_leftTarget);
	m_rightMasterDrive.Set(m_rightTarget);
	UpdateStats();
}

void CANTalonDriveTrain::ArcadeDrive(double commandYAxis, double commandXAxis, bool slowSpeed)
{
	m_leftTarget  = Deadband(commandYAxis)  * DRIVE_MAX_SPEED * m_speedFactor;
	commandXAxis = Deadband(commandXAxis);

	if (slowSpeed)
	{
		m_leftTarget  *= kSlowSpeedFactor;
	}

	m_leftMasterDrive.Set(-(m_leftTarget + commandXAxis));
	m_rightMasterDrive.Set(m_leftTarget);
}

void CANTalonDriveTrain::AutoDriveStraight(double leftCommand, double rightCommand)
{
//<<<<<<< HEAD
	// For testing with just the left joystick
//=======
	// For testing with just the left joystick, change leftCommand and rightCommand
	//				to m_leftTarget. Uncomment next line.
//>>>>>>> 94395bccd5f26fad7846fcee760d875d9f189364
	// m_leftTarget  = Deadband(leftCommand)  * DRIVE_MAX_SPEED * m_speedFactor;
	// m_leftTarget = for the both motors since using one joystick


	if(leftCommand == 0)
		encVelDiff = 0;
	else
		encVelDiff = abs(m_rightEncoderVel) - abs(m_leftEncoderVel);

	if (encVelDiff > 100)
	{
		m_leftMasterDrive.Set(-((leftCommand * DRIVE_MAX_SPEED * m_speedFactor)));
		m_rightMasterDrive.Set((rightCommand * DRIVE_MAX_SPEED * m_speedFactor)); // - adjustBy);
	}
	else if(encVelDiff < -100)
	{
		m_leftMasterDrive.Set(-((leftCommand * DRIVE_MAX_SPEED * m_speedFactor))); //- adjustBy));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * m_speedFactor);
	}
	// else runs if the difference is -100 to 100
	else
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * m_speedFactor));
		m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * m_speedFactor);
	}

	UpdateStats();
}

void CANTalonDriveTrain::AutoDriveStraight2(double leftCommand, double rightCommand)
{
	currentAngle = m_pGyro->GetAngle();
	if(trunc(currentAngle) > 3)
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * m_speedFactor));
		m_rightMasterDrive.Set((rightCommand + adjustBy) * DRIVE_MAX_SPEED * m_speedFactor);
	}
	else if(trunc(currentAngle) < -3)
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * m_speedFactor));
		m_rightMasterDrive.Set((rightCommand - adjustBy) * DRIVE_MAX_SPEED * m_speedFactor);
	}
	else
	{
		m_leftMasterDrive.Set(-(leftCommand * DRIVE_MAX_SPEED * m_speedFactor));
			m_rightMasterDrive.Set(rightCommand * DRIVE_MAX_SPEED * m_speedFactor);
	}
}


void CANTalonDriveTrain::AutoCalculateTurn(double desiredAngle, double turnSpeed)
{
	calculatedSpeed = turnSpeed * ((desiredAngle < 0) ? -100 : 100);
}

bool CANTalonDriveTrain::AutoTurn(double desiredAngle)
{
	// Gyro is reset before this function is called
	frc::SmartDashboard::PutNumber("Desired Angle  : ", desiredAngle);
	currentAngle = m_pGyro->GetAngle();
	DriveTrainUpdateDashboard();
	while (trunc(currentAngle) != desiredAngle)
	{
		m_leftMasterDrive.Set(calculatedSpeed);
		m_rightMasterDrive.Set(calculatedSpeed);
		currentAngle = m_pGyro->GetAngle();
		DriveTrainUpdateDashboard();
	}

	Stop();
	return true;
}

bool CANTalonDriveTrain::AutoMove(double desiredRevolutions, double leftSpeed, double rightSpeed)
{
	frc::SmartDashboard::PutNumber("Desired Revolutions  : ", desiredRevolutions);
	DriveTrainUpdateDashboard();
	// +2000 attempts to make the wheel stop a little before it gets to the desired spot
	revolutionsDone = (static_cast<double>(m_leftMasterDrive.GetEncPosition()) + 2000) / static_cast<double>(DRIVE_ENCDR_STEPS * 4);
	UpdateStats();
	if((abs(revolutionsDone)) < desiredRevolutions)
	{
		AutoDriveStraight(-leftSpeed, -rightSpeed);
		UpdateStats();
		DriveTrainUpdateDashboard();
		return false;
	}

	Stop();
	return true;
}

void CANTalonDriveTrain::DriveTrainUpdateDashboard(void)
{
	frc::SmartDashboard::PutNumber("Revolutions Done  : ", abs(revolutionsDone));
	frc::SmartDashboard::PutNumber("Left Enc. Pos. from Drivetrain : ", m_leftEncoderPos);
	frc::SmartDashboard::PutNumber("Right Enc. Pos. from Drivetrain : ", m_rightEncoderPos);
	frc::SmartDashboard::PutNumber("Current Angle from Drivetrain : ", currentAngle);
}


// Not used
void CANTalonDriveTrain::AutoTurnStart(double currentAngle, double deltaAngle, double turnSpeed)
{
	m_deltaAngle = deltaAngle;
	if(deltaAngle == 0)
		return;

	m_startAngle = currentAngle;
	m_endAngle = fmod((m_startPosition + deltaAngle), 360.0);

	double speed = turnSpeed * ((deltaAngle < 0) ? -100 : 100);
	m_leftMasterDrive.Set(speed);
	m_rightMasterDrive.Set(speed);

	UpdateStats();
}

// Not used
bool CANTalonDriveTrain::AutoTurnUpdate(double currentAngle)
{
	if(m_deltaAngle == 0)
		return true;

	m_leftPosition = currentAngle;
	m_deltaPosition = m_endPosition - m_leftPosition;

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	if (m_deltaPosition > 0)
	{
		Stop();
		UpdateStats();
		return true;
	}

	UpdateStats();
	return false;
}

// Not used
void CANTalonDriveTrain::AutoMoveStart(double legLength, double leftSpeed, double rightSpeed)
{
	m_startPosition = m_leftMasterDrive.GetPosition();
	m_endPosition = m_startPosition - legLength;

	m_leftMasterDrive.Set(leftSpeed * DRIVE_MAX_SPEED);
	m_rightMasterDrive.Set(-rightSpeed * DRIVE_MAX_SPEED);

	UpdateStats();
}

// Not used
bool CANTalonDriveTrain::AutoMoveUpdate(void)
{
	m_leftPosition = m_leftMasterDrive.GetPosition();
	m_rightPosition = m_rightMasterDrive.GetPosition();
	m_deltaPosition = m_endPosition - m_leftPosition;

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	if (m_deltaPosition > 0)
	{
		Stop();
		UpdateStats();
		return true;
	}

	UpdateStats();
	return false;
}



double CANTalonDriveTrain::Deadband(double commandValue)
{
	//return commandValue;
	return (fabs(commandValue) >= DRIVE_COMMAND_DEADBAND) ? commandValue : 0.0;
}

