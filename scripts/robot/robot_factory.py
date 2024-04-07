class RobotFactory:
    def __init__(self, identification_type):
        self.identification_type = identification_type

    def get_identification(self):
        if self.identification_type == 'nationality':
            return "nationality"
        elif self.identification_type == 'race':
            # return race identification
            pass
        elif self.identification_type == 'gender':
            # return gender identification
            pass
        else:
            raise ValueError(f"Unsupported identification type: {self.identification_type}")

    def create_robot(self, robot_number):
        if robot_number == 1:
            return 1
        elif robot_number == 2:
            return 2
        elif robot_number == 3:
            return 3
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")