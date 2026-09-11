def load(program, robot):
    @program(1, "MissionKeineAhnungDigga?!")
    def route_program():
        robot.set_gyro_use(True)
        robot.set_drivebase_settings(turn_rate=100)
        robot.straight(300)
        robot.turn(-13)
        robot.straight(120)
        robot.straight(-100)

    @program(4, "Blink")
    def blink_program():
        while True:
            robot.hub.display.pixel(2, 2, 100)
            robot.wait(250)
            robot.hub.display.pixel(2, 2, 0)
            robot.wait(250)
