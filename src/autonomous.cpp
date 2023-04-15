
// #include "main.h"
#include "autonomous.hpp"
// #include "main.h"
pros::Controller driver(pros::E_CONTROLLER_MASTER);
void controlFunction(void *controlArg)
{

	Drivetrain *drive = ((control_arg *)controlArg)->drive;
	IntakeRoller *intake = ((control_arg *)controlArg)->intake;
	Shooter *shooter = ((control_arg *)controlArg)->shooter;
	int curTime = pros::millis();
	while (pros::millis() - curTime < 14990 + 1000000)
	{
		drive->updateDrivetrain(driver);
		intake->updateIntake(driver);
		shooter->updateShooter(driver);

		pros::delay(50);
	}
}
void odomAuton()
{
	std::uint32_t autoStartTime = pros::millis();

	control_arg *control_task_arg = new control_arg;

	Drivetrain *drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller *intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter *shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	drivetrainObj->resetGyro();
	shooterObj->setIndexerState(true);
	pros::delay(2500);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// TESTED
	drivetrainObj->turnAngle(90.0);
	// driver.print(2, 2, "%f  ", drivetrainObj->angleSepoint);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned())
	{
	}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	// driver.print(2, 2, "SETTLED  ");
	drivetrainObj->turnAngle(0.0);
	driver.print(2, 2, "%f  ", drivetrainObj->angleSepoint);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned())
	{
	}
	driver.print(2, 2, "SETTLED  ");
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	//!!!!!!!!!!!!! NEED TO TEST !!!!!!!!!!!!!!!!!!
	drivetrainObj->moveDistance(48.0, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	while (!drivetrainObj->isSettledMove())
	{
	}
	driver.print(2, 2, "SETTLED  ");
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	drivetrainObj->turnAngle(90.0);
	// driver.print(2, 2, "%f  ", drivetrainObj->angleSepoint);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned())
	{
	}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	
	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();
}
void PIDRollerLow(){
		std::uint32_t autoStartTime = pros::millis();

	control_arg *control_task_arg = new control_arg;

	Drivetrain *drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller *intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter *shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	while(drivetrainObj->gyro->is_calibrating()){}
	shooterObj->setIndexerState(true);
	

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);

	//ROLLERS
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	intakeObj->setVel(-300);
	drivetrainObj->moveSeconds(750, 50);
	intakeObj->setVel(0);
	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
	intakeObj->spinSec(500,-300);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	drivetrainObj->moveSeconds(500, -100);
	
	// This can be changed back to 45 if needed
	// drivetrainObj->turnAngle(90.0);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	// while (!drivetrainObj->isSettledTurned())
	// {
	// }
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


	// drivetrainObj->moveDistance(-50.0, 0.0);
	// drivetrainObj->resetEnc();
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	// while (!drivetrainObj->isSettledMove())
	// {
	// }
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// shooterObj->setTargetRPM(200.0);
	// shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP_AUTO);
	// pros::delay(1500);
	// shooterObj->indexAll2();


	while(pros::millis() - autoStartTime < 14500)
	{}
	//drivetrainObj->~Drivetrain();
	controlTask.remove();
	delete(intakeObj);
	delete(drivetrainObj);

}
void odomAutonAWP()
{
	std::uint32_t autoStartTime = pros::millis();

	control_arg *control_task_arg = new control_arg;

	Drivetrain *drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller *intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter *shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	while(drivetrainObj->gyro->is_calibrating()){}
	shooterObj->setIndexerState(true);
	// pros::delay(250);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);

	shooterObj->setTargetRPM(420.0);
	shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP_AUTO);

	//ROLLERS
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	intakeObj->setVel(-300);
	drivetrainObj->moveSeconds(750, 50);
	intakeObj->setVel(0);
	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
	intakeObj->spinSec(500,-300);
	
	
	//TURN DRIVE STRAIGHT
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	drivetrainObj->moveDistance(-5.5, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	while (!drivetrainObj->isSettledMove())
	{
	}

	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


	// This can be changed back to 45 if needed
	drivetrainObj->turnAngle(-130.0);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned())
	{
	}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	
	drivetrainObj->moveDistance(96.0, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	intakeObj->spinSec(700, -300);
	intakeObj->spinSec(300, 300);
	while (!drivetrainObj->isSettledMove())
	{
	}
	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// INTAKE AND SHOT
	
	
	drivetrainObj->turnAngle(-45.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	
	while (!drivetrainObj->isSettledTurned())
	{
	}
	intakeObj->setVel(600);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	
	drivetrainObj->moveDistance(-8.0, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	while (!drivetrainObj->isSettledMove() && pros::millis() - autoStartTime < 12000)
	{
	}
	
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


	shooterObj->indexAll2();
	
	while(pros::millis() - autoStartTime < 14500)
	{}
	//drivetrainObj->~Drivetrain();
	controlTask.remove();
	delete(intakeObj);
	delete(drivetrainObj);
}
void PIDAutonFarRollDisk(){
	std::uint32_t autoStartTime = pros::millis();

	control_arg *control_task_arg = new control_arg;

	Drivetrain *drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller *intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter *shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	drivetrainObj->resetGyro();
	shooterObj->setIndexerState(true);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);





	shooterObj->setTargetRPM(500.0);
	shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP_AUTO);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	
	drivetrainObj->moveDistance(-45.0, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	while (!drivetrainObj->isSettledMove())
	{
	}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	drivetrainObj->turnAngle(23.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned())
	{
	}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	pros::delay(6000);
	shooterObj->indexAll2();


	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();
}
void PIDAutonFarRoller(){
	std::uint32_t autoStartTime = pros::millis();

	control_arg *control_task_arg = new control_arg;

	Drivetrain *drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller *intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter *shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	drivetrainObj->resetGyro();
	shooterObj->setIndexerState(true);
	

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	//DRIVE IN FRONT OF ROLLER
	drivetrainObj->moveDistance(15.0, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	while (!drivetrainObj->isSettledMove()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	//TURN TO ROLLER
	drivetrainObj->turnAngle(-90.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	drivetrainObj->moveSeconds(750, 50);
	intakeObj->spinSec(500,-400);
	//SPIN ROLLERS
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();
}
void PIDAutonTwoRoller(){
	std::uint32_t autoStartTime = pros::millis();

	control_arg *control_task_arg = new control_arg;

	Drivetrain *drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller *intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter *shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	drivetrainObj->resetGyro();
	shooterObj->setIndexerState(true);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	drivetrainObj->moveSeconds(750, 50);
	intakeObj->spinSec(500,-400);

	//TURN AND DRIVE ACROSS

	drivetrainObj->turnAngle(-135.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::TURN_ANGLE);
	while (!drivetrainObj->isSettledTurned())
	{}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


	drivetrainObj->moveDistance(-200.0, 0.0);
	drivetrainObj->resetEnc();
	drivetrainObj->setState(Drivetrain::DrivetrainStates::MOVE_DISTANCE);
	while (!drivetrainObj->isSettledMove())
	{}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

}
void HCRoller()
{
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj = IntakeRoller();
	Shooter shooterObj = Shooter();
	Expansion expansionObj = Expansion();
	// expansionObj.expansionPistonR->set_value(false);
	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	pros::delay(3000);

	//===== START =====

	// ROLLERS
	drivetrainObj.moveSeconds(700, 40);
	drivetrainObj.setVel(40);
	intakeObj.rollToColourSEC(800);
	drivetrainObj.moveEncoder(-5, 200);
}
void HCRollerDisc()
{
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj = IntakeRoller();
	Shooter shooterObj = Shooter();
	Expansion expansionObj = Expansion();
	// expansionObj.expansionPistonR->set_value(false);
	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	shooterObj.setMotorSpeed(450);
	pros::delay(3000);

	//===== START =====

	// ROLLERS

	shooterObj.indexAll();

	shooterObj.setMotorSpeed(0);

	drivetrainObj.moveSeconds(800, 40);
	drivetrainObj.setVel(40);
	intakeObj.rollToColourSEC(800);
	drivetrainObj.moveEncoder(-5, 200);
	// pros::delay(500);

	// Turn & Drive to middle
	//  drivetrainObj.turnGyro(-64.0, 100);

	// drivetrainObj.moveEncoder(120, 100);
	// pros::delay(500);

	// //Shoot into goal
	// shooterObj.indexAll();
	// shooterObj.setMotorSpeed(0);

	// //turn & drive to 2nd roller
	// drivetrainObj.turnGyro(-22.0, 100);
	// // drivetrainObj.moveEncoder(65, 100);
}
void HCRollerTwo()
{
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj = IntakeRoller();
	Shooter shooterObj = Shooter();

	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	pros::delay(3000);

	//===== START =====

	// ROLLERS

	drivetrainObj.moveEncoder(20, 100);
	pros::delay(500);
	drivetrainObj.turnGyro(22.0, 100);
	drivetrainObj.moveSeconds(700, 40);
	drivetrainObj.setVel(40);
	intakeObj.rollToColourSEC(800);
}
void HCRollerTwoDisc()
{
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj = IntakeRoller();
	Shooter shooterObj = Shooter();

	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	shooterObj.setMotorSpeed(275);
	pros::delay(4000);

	//===== START =====

	// ROLLERS
	shooterObj.indexAll2();

	shooterObj.setMotorSpeed(0);

	drivetrainObj.moveEncoder(70, 200);
	pros::delay(500);
	drivetrainObj.turnGyro(70.0, 100);
	drivetrainObj.moveSeconds(850, 50);
	drivetrainObj.setVel(45);
	intakeObj.rollToColourSEC(250);
	drivetrainObj.setVel(0);
}
