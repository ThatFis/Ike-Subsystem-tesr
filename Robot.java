// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//test drive on imports//


import com.revrobotics.spark.SparkMax;     
import com.revrobotics.spark.SparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

 
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

private final RobotContainer m_robotContainer;
private XboxController Driver;
private XboxController Operater;

//Callouts//

//Motors that outtake the Coral//
private SparkMax CoralMotorL;
private SparkMax CoralMotorR;

//The Motor that pivets the ball//
private SparkMax BallMotorP;

//The Motor that intakes the ball//
private SparkMax BallMotorI;

//Lift the Motor//
private SparkMax LiftMotor;



//private final DoubleSolenoid Endsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);
//private final DoubleSolenoid clawsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6,9);
//private final DoubleSolenoid Ballsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 10);
//private final DoubleSolenoid Armsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 4);

// private boolean BallOut = false;
// private boolean Processor = false;
// private boolean ground = false;
// private boolean End = false;

public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //Controller type//  
    //Driver = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    //Operater = new XboxController(Constants.OperatorConstants.kOperatorControllerPort);


//
    BallMotorI = new SparkMax(Constants.OperatorConstants.BallMotorI, MotorsType .kBrushless);
    BallMotorP = new SparkMax(Constants.OperatorConstants.BallMotorP, MotorType.kBrushless);

//
    CoralMotorL = new SparkMax(Constants.OperatorConstants.CoralMotorL, MotorType.kBrushless);
    CoralMotorR = new SparkMax(Constants.OperatorConstants.CoralMotorR, MotorType.kBrushless);
//

}
  


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic(){

          //Motors intake pivets the ball subsystem//

          // Set the motor speed based on trigger values
          if (Operater.getRightTriggerAxis() > 0.1) {
            // Move motor forward
            BallMotorP.set(-.4); // Scale speed down to 50%
        } else if (Operater.getLeftTriggerAxis() > 0.05) {
            // Move motor backward
            BallMotorP.set(.4); // Scale speed down to 50%
        } else {
            // Stop motor
            BallMotorP.set(0);

         //inatke coral//   
        }
        if (Operater.getRightBumperButtonPressed()) {
          // Move motor forward
          BallMotorI.set(-.4); // Scale speed down to 50%
      } else if (Operater.getLeftBumperButtonPressed()) {
          // Move motor backward
          BallMotorI.set(.4); // Scale speed down to 50%
      } else {
          // Stop motor
          BallMotorI.set(0);

      //elevator subsystem

      // Set the motor speed based on trigger values
      if (Driver.getAButtonPressed()) {
          // Move motor forward
          LiftMotor.set(-.2); // Scale speed down to 50%
      } else if (Driver.getAButtonReleased()) {
          // Move motor backward
          LiftMotor.set(.97); // Scale speed down to 50%
      } else {
          // Stop motor
          LiftMotor.set(0);
      }


      // Motors that push the coral to the reef

       // Control the motors based on bumper inputs
       if (Driver.getAButtonPressed()) {
           // Spin motors forward
           CoralMotorL.set(.8); // 80% speed forward
           CoralMotorR.set(.8); // 80% speed forward

       } else if (Driver.getBButtonPressed()) {
           // Spin motors backward
           CoralMotorL.set(-.8); // 80% speed forward
           CoralMotorR.set(-.8); // 80% speed forward
       } else{
           // Stop motors
           CoralMotorL.set(0);
           CoralMotorR.set(0);

       }
      }

    //on and off for hang cylinder



    //if(Operater.getRightBumperButtonPressed()){

      //End = !End;
  //}  
  //if (End){
    
    //Endsolenoid.set(Value.kForward);
   
//} else {
    //Endsolenoid.set(Value.kReverse);
//}
    
  
    
    //if(Operater.getYButtonPressed()){

      //BallOut = !BallOut;
  //}  
  //if (BallOut){
    
    //clawsolenoid.set(Value.kForward);
   
//} else {
    //clawsolenoid.set(Value.kReverse);
//}

    

  //if(Operater.getXButtonPressed()){

      //Processor = !Processor;
  //}  
  //if (Processor){
    
    //Ballsolenoid.set(Value.kForward);
   
//} else {
    //Ballsolenoid.set(Value.kReverse);
//}

//if(Operater.getBButtonPressed()){

  //ground = !ground;
//}  
//if (ground){

//Armsolenoid.set(Value.kForward);

//} else {
//Armsolenoid.set(Value.kReverse);
//}




   }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
