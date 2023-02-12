// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;

public class Robot extends TimedRobot {
  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(6);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(2);

  private final XboxController m_stick = new XboxController(0);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  // private final Compressor theCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  // Compressor compressor;

  private CANSparkMax mPivotMotor;
  // private RelativeEncoder mPivotEncoder;
  private SparkMaxPIDController mPivotPIDController;

  private static final double kPivotPowerOut = 1.0;
  private static final double kPivotPowerIn = -0.7;
  // private static final double kExtensionPowerOut = 0.6;
  // private static final double kExtensionPowerIn = -0.6;
  // private static final double kPivotBoostAmount = -3;
  // private static final double kPivotBoost2Amount = -15;

  private static final double kPivotCLRampRate = 0.5;
  // private static final double kExtensionCLRampRate = 0.5;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);

    diffDrive.setDeadband(0.04);

    mPivotMotor = new CANSparkMax(9, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotPIDController = mPivotMotor.getPIDController();
    //mPivotEncoder = mPivotMotor.getEncoder();

    mPivotPIDController.setP(0.1);
    mPivotPIDController.setI(1e-8);
    mPivotPIDController.setD(1);
    mPivotPIDController.setIZone(0);
    mPivotPIDController.setFF(0);
    mPivotPIDController.setOutputRange(kPivotPowerIn, kPivotPowerOut);
    mPivotMotor.setClosedLoopRampRate(kPivotCLRampRate);

    mPivotPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    if(Math.abs(m_stick.getLeftY()) > 0.2 || Math.abs(m_stick.getLeftX()) > 0.2)
    {
      diffDrive.arcadeDrive(-m_stick.getLeftY() * 0.75, -m_stick.getLeftX() * 0.75);
    }
    else
    {
      diffDrive.arcadeDrive(-m_stick.getRightY(), (-m_stick.getRightX()) * 0.75);
    }

    if(m_stick.getBButtonPressed())
    {
      mPivotPIDController.setReference(60, CANSparkMax.ControlType.kPosition);
    }
    if(m_stick.getAButtonPressed())
    {
      mPivotPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }
}
