def robot_checks(robot_list,robots):
    for robot in robots:
        if robot in robot_list:
            print("found it!")
        else: print("robot:[{}] not found in robot_list!".format(robot.name))