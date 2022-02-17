// public void AutoPowerCellMoverShooter() {
//     if (Robot.oi.getShooterButton()) {
//       shooterkp = Constants.SHOOTER_KP;
//       shooterki = Constants.SHOOTER_KI;
//       shooterkd = Constants.SHOOTER_KD;
//       // Get actual speed
//       actual_speed = Robot.powerCellMover.getShooterSpeed();
//       SmartDashboard.putNumber("ProcessVariable", actual_speed);
//       System.out.println(actual_speed);
//       /*if (actual_speed < TARGET_RUN_SPEED_SHOOTER - RUN_TOLERANCE_SHOOTER) {
//         current_speed = 1.0; // speed up as quickly as possible
//       } else if (actual_speed < TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER) {
//         current_speed = MAINTAIN_RUN_SPEED_SHOOTER;
//       } else {
//         // implies actual_speed >= TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER
//         current_speed = SLOW_RUN_SPEED_SHOOTER;
//       }*/
//       if (OI.auto_shooting) {
//         TARGET_RUN_SPEED_SHOOTER = OI.auto_shooting_speed;
//         MAINTAIN_RUN_SPEED_SHOOTER = TARGET_RUN_SPEED_SHOOTER * Constants.RPM_TO_SHOOTER_POWER_CONVERSION;
//       }

//       speedError = TARGET_RUN_SPEED_SHOOTER - actual_speed;
//       speedErrorPercent = speedError / TARGET_RUN_SPEED_SHOOTER;
      
//       speedProportional = speedErrorPercent;
//       if (Math.abs(TARGET_RUN_SPEED_SHOOTER - actual_speed) <=200) { // Anti-Windup
//         speedIntegral = speedIntegral + speedErrorPercent;
//       }
//       speedDerivative = speedErrorPercent - lastSpeedErrorPercent;

//       correction = (speedProportional * shooterkp) + (speedIntegral * shooterki) + (speedDerivative * shooterkd) + MAINTAIN_RUN_SPEED_SHOOTER; //PIDF controller output
//       cappedCorrection = Math.max(Math.min(correction, 1.0), SLOW_RUN_SPEED_SHOOTER);

//       Robot.powerCellMover.runShooterOpen(cappedCorrection);

//       lastSpeedErrorPercent = speedErrorPercent;

//       if (((actual_speed > TARGET_RUN_SPEED_SHOOTER - RUN_TOLERANCE_SHOOTER) && (actual_speed < TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER)) || shooterSpunUp) {
//         Robot.powerCellMover.runIndexer(INDEXER_SHOOTING_RUN_SPEED);
//         shooterSpunUp = true;
//         if (idx2singulatorDelayCount >= 5) {
//            Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
//         } else {
//           idx2singulatorDelayCount++;
//         }
//       } else {
//         Robot.powerCellMover.runIndexer(0);
//         Robot.powerCellMover.setSingulatorSpeed(0);
//         spinupDelayCount++;
//       }
//     } else {
//       idx2singulatorDelayCount = 0;
//       spinupDelayCount = 0;
//       shooterSpunUp = false;
//       // Slowing down motor and don't want to do it too fast
//       if (current_speed < MIN_RUN_SPEED) {
//         current_speed = 0;
//       } else {
//         current_speed = current_speed * MANUAL_RUN_SPEED_SHOOTER;
//       }
//       Robot.powerCellMover.runShooterOpen(current_speed);
//     }
//   }