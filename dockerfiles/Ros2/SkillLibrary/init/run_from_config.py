import json
import os
from skill_library.behaviours import skills
import rclpy
import asyncio

async def execute_tasks():
    TASKS = json.loads(os.environ['TASKS'])

    ROBOT_NAME = os.environ['ROBOT_NAME']
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
    ROBOT_POSE = json.loads(os.environ['ROBOT_POSE'])
    NURSE_POSE = [float(x) for x in json.loads(os.environ['NURSE_POSE'])]
    ARM_POSE = [float(x) for x in json.loads(os.environ['ARM_POSE'])]

    IC_CORRIDOR_POSE = [-37.00, 16.00, 0.00]
    PC_CORRIDOR_POSE = [-27.25, 16.00, 0.00]
    ARM_CORRIDOR_POSE = [-37.00, 34.00, 0.00]

    skill_library = skills.Skills()

    IC_CORRIDOR_POSE = skill_library.poselist_to_posestamped(IC_CORRIDOR_POSE)
    PC_CORRIDOR_POSE = skill_library.poselist_to_posestamped(PC_CORRIDOR_POSE)
    ARM_CORRIDOR_POSE = skill_library.poselist_to_posestamped(ARM_CORRIDOR_POSE)
    NURSE_POSE = skill_library.poselist_to_posestamped(NURSE_POSE)
    ARM_POSE = skill_library.poselist_to_posestamped(ARM_POSE)


    for index, task in enumerate(TASKS):
        print(f"Executing task: {task}")
        skill_library.publish_log(message="started skill", skills=task)
        match task:
            case "navto":
                if TASKS[index+1] == "approach-nurse":
                    nav_target = [IC_CORRIDOR_POSE, PC_CORRIDOR_POSE, NURSE_POSE]
                elif TASKS[index+1] == "approach-arm":
                    nav_target = [PC_CORRIDOR_POSE, ARM_CORRIDOR_POSE, ARM_POSE]
                else:
                    nav_target = []
                result = await skill_library.navigate(nav_target)
            case "approach-nurse":
                result = await skill_library.approach(target="nurse")
            case "approach-robot":
                result = await skill_library.approach(target="robot")
            case "approach-arm":
                result = await skill_library.approach(target="robot")
            case "authenticate-nurse":
                result = await skill_library.authenticate_person()
            case "open-drawer":
                result = await skill_library.operate_drawer()
            case "close-drawer":
                result = await skill_library.operate_drawer()
            case "deposit":
                result = await skill_library.operate_drawer()
            case "pick-up-sample":
                result = await skill_library.send_message(topic="/arm_pickup", message="pickup-sample")
            case _:
                result = False
        if not result:
            skill_library.publish_log(severity="WARN", message="SKILL-FAILURE", skills=task)
            break
        skill_library.publish_log(message="skill-success", skills=task)
    else:
        skill_library.publish_log(severity="WARN", message="SUCCESS")
    rclpy.spin(skill_library)


if __name__ == "__main__":
    rclpy.init()
    asyncio.run(execute_tasks())