def load(program, robot):
    @program(4, "DerErsteRun?!")
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
    
    @program(1, "gamble")
    def gamble():   
        robot.reset_drivebase_settings()
        robot.set_drivebase_settings(600)
        robot.set_gyro_use(True)
        robot.straight(-50)
        robot.turn(40)
        robot.straight(750)
        robot.turn(105)
        robot.motor_angle(robot.left_motor, 300, -110)
        robot.straight(-330)
        robot.motor_angle(robot.left_motor, 300, 110)
        robot.straight(65)
        robot.turn(-120)
        robot.straight(-50)
        robot.straight(30)
        robot.motor_angle(robot.right_motor, 300, -140)
        robot.straight(130)
        robot.motor_angle(robot.right_motor, 1400, -390)
        robot.set_drivebase_settings(straight_speed=1000)
        robot.straight(-500)
        robot.turn(30)
        robot.straight(-300)
        
        
        
        
        
        

        
    