from pickplace.world_setup import build_world
from pickplace.task import PickAndPlaceTask

MAX_STEPS = 60000

def main():
    world, robot, box = build_world()
    task = PickAndPlaceTask(world, robot, box)
    last_state = None
    for i in range(MAX_STEPS):
        task.step()
        if task.state_name != last_state:
            print(f"[step {i:6d}] -> {task.state_name}")
            last_state = task.state_name
        if task.done:
            break
    print("Final box position:", box.getPositions()[3:6])
    print("Task finished:", task.done)

if __name__ == "__main__":
    main()
