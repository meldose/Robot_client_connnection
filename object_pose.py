import random

def get_random_pose():
    
    pos_1 = [random.uniform(-1,1)for _ in range(3)]
    orientation=[random.uniform(-1,1) for _ in range(4)]
    norm=sum(q** 2 for q in orientation)**0.5
    orientation = [q/norm for q in orientation]
    return {

        "position": pos_1,
        "orientation": orientation
    }

def get_object_poses(num_objects=5):
    """Retrieve a list of object poses."""
    objects = []
    for i in range(num_objects):
        pose = get_random_pose()
        objects.append({
            "id": i + 1,
            "name": f"Object_{i+1}",
            "position": pose["position"],
            "orientation": pose["orientation"]
        })
    return objects

if __name__ == "__main__":
    object_poses = get_object_poses()
    for obj in object_poses:
        print(f"ID: {obj['id']}, Name: {obj['name']}, Position: {obj['position']}, Orientation: {obj['orientation']}")
