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


    @program(5, "Mariia")
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

    @program(1, "setup")
    def setupmotor():
        robot.motor_target(robot.right_motor, 300, 45)
        robot.motor_target(robot.left_motor, 300, 0)
        robot.wait(2000)
        
    @program(2, "betterGamble")
    def bettergamble():
        robot.reset_drivebase_settings()
        robot.set_drivebase_settings(450)
        robot.set_gyro_use(True)
        robot.straight(-50)
        robot.reset_heading(0)
        robot.straight(100)
        robot.turn(20)
        robot.straight(600)
        robot.turn(110)
        robot.motor_angle(robot.left_motor, 300, -115)
        robot.straight(-320)
        robot.motor_angle(robot.left_motor, 300, 115)
        robot.straight(70)
        robot.turn(-115)
        robot.motor_angle(robot.right_motor, 300, -285)
        robot.set_drivebase_settings(straight_speed=200)
        robot.straight(75)
        robot.motor_angle(robot.right_motor, 1000, -380)

    @program(6, "altePalme")
    def altePalme():
        robot.set_gyro_use(True)
        robot.straight(300)
        robot.motor_angle(robot.right_motor, 300, 40)
        robot.straight(80)
        robot.motor_angle(robot.right_motor, 200, -40)
        robot.straight(-120)
        robot.motor_angle(robot.right_motor, 300, 40)
        robot.straight(-200)
                        
        
