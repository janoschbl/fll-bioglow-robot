def load(program, robot):
    @program(1, "DerErsteRun?!")
    def route_program():
        robot.set_gyro_use(True)
        robot.straight(410)
        robot.straight(-410)
        robot.straight(30)
        robot.turn(46)
        robot.motor_angle(robot.right_motor, speed=300, angle=-95)
        robot.straight(530)
        robot.motor_angle(robot.right_motor, speed=300, angle=95)
        robot.set_drivebase_settings(straight_speed=800)
        robot.straight(-500)
           
    @program(2, "Mariia")
    def mariia():
        robot.set_gyro_use(True)
        robot.straight(600)
        robot.turn(60)
        robot.straight(700)
        robot.turn(45)
        robot.straight(440)
        robot.turn(-60)
        robot.set_drivebase_settings(straight_speed=60)
        robot.straight(100)
        robot.set_drivebase_settings(straight_speed=450)
        robot.straight(-50)
        robot.turn(90)
        robot.straight(600)
        robot.turn(30)
        robot.straight(200)
        
    @program(3, "Drohnenfiech")
    def drohnenfiech():
        robot.set_gyro_use(True)
        robot.straight(680)
        robot.turn(-70)
        robot.turn(70)
        robot.straight(-200)
        
        
        

        
    