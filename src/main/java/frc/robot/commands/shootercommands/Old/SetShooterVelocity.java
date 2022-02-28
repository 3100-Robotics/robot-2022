package frc.robot.commands.shootercommands.Old;
// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.shootercommands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Robot;
// import frc.robot.subsystems.Shooter;

// public class SetShooterVelocity extends InstantCommand {
//   private final double percentage;
//   private final Shooter m_shooter;

//   public SetShooterVelocity(Shooter m_subsystem, double p) {

//     m_shooter = m_subsystem;
//     percentage = p;
//     addRequirements(m_shooter);

    
//   }

//   @Override
//   public void initialize() {
//     System.out.println("Ahh");
//     m_shooter.setPercentVelocity(percentage);
//   }
//   public void execute(){

//     System.out.println("Wahh");
//     m_shooter.setPercentVelocity(percentage);

//   }
// }