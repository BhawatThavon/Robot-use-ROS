import numpy as np
import cv2
import yaml

def load_map_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        map_data = yaml.safe_load(file)

    map_image_path = map_data['image']
    resolution = map_data['resolution']
    origin = map_data['origin']
    map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)

    return map_image, resolution, origin

def myRRT(Map, startPoint, targetPoint, option=None):
    # Check for option parameter
    if option is None:
        option = {'MaxIter': 1000, 'RandomSampleThresh': 0.5, 'StepLength': 20,
                  'OccupyThresh': None, 'InflateRadius': None, 'Display': False, 'PauseTime': 0.1}

    # Preprocess the map
    map_img = Map
    if map_img.ndim == 3:
        map_img = cv2.cvtColor(map_img, cv2.COLOR_RGB2GRAY)
    if option['OccupyThresh'] is None:
        option['OccupyThresh'], _ = cv2.threshold(map_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, map_img = cv2.threshold(map_img, option['OccupyThresh'], 255, cv2.THRESH_BINARY)
    if option['InflateRadius'] is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (option['InflateRadius'], option['InflateRadius']))
        map_img = cv2.erode(map_img, kernel)

    # Initialize RRTree
    RRTree = {'point': np.array([startPoint]), 'parent': [0]}

    # Initialize finish flag
    finish = False

    # RRT loop
    height, width = map_img.shape
    for _ in range(option['MaxIter']):
        # Sample a point
        if np.random.rand() <= option['RandomSampleThresh']:
            sample = np.random.rand(2) * [width, height]
        else:
            sample = targetPoint

        # Find closest node in RRTree
        parentIndex = np.argmin(distanceCost(RRTree['point'], sample))
        closestNode = RRTree['point'][parentIndex]

        # Check if target reached
        if distanceCost(closestNode, targetPoint) < option['StepLength'] and checkPath(closestNode, targetPoint, map_img):
            finish = True
            RRTree['point'] = np.append(RRTree['point'], [targetPoint], axis=0)
            RRTree['parent'].append(parentIndex)
            break

        # Extend tree towards sample
        orin = np.arctan2(sample[1] - closestNode[1], sample[0] - closestNode[0])
        newPoint = np.round(closestNode + option['StepLength'] * np.array([np.cos(orin), np.sin(orin)]))
        if not checkPath(closestNode, newPoint, map_img):
            continue

        # Add new point to RRTree
        _, index = min((distanceCost(node, newPoint), i) for i, node in enumerate(RRTree['point']))
        if index != parentIndex:
            continue
        else:
            RRTree['point'] = np.append(RRTree['point'], [newPoint.astype(int)], axis=0)
            RRTree['parent'].append(parentIndex)

    # Reconstruct path
    PathList = []
    if finish:
        PathList.append(targetPoint)
        index = RRTree['parent'][-1]
        while index != 0:
            point = RRTree['point'][index]
            PathList.append(point)
            index = RRTree['parent'][index]
    else:
        print('Can not find a path from startPoint to targetPoint!')

    # Return the PathList
    return PathList

# Define function to display map and path
def display_map_and_path(map_image, start_point, target_point, path_list):
    # Convert path list to numpy array
    path_array = np.array(path_list)

    # Draw the path on the map
    map_with_path = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
    for i in range(1, len(path_array)):
        cv2.line(map_with_path, tuple(path_array[i - 1]), tuple(path_array[i]), (0, 255, 0), 2)

    # Draw start and target points
    cv2.circle(map_with_path, tuple(start_point.astype(int)), 5, (0, 0, 255), -1)
    cv2.circle(map_with_path, tuple(target_point.astype(int)), 5, (255, 0, 0), -1)

    # Display the map with path
    cv2.imshow('Map with Path', map_with_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def distanceCost(a, b):
    if a.ndim == 1:
        a = np.expand_dims(a, axis=0)
    if b.ndim == 1:
        b = np.expand_dims(b, axis=0)
    return np.sqrt(np.sum((a - b) ** 2, axis=1))

def feasiblePoint(point, map_img):
    height, width = map_img.shape
    x, y = point
    return 0 < x <= width and 0 < y <= height and map_img[int(y), int(x)] == 255

def checkPath(point, newPoint, map_img):
    orin = np.arctan2(newPoint[1] - point[1], newPoint[0] - point[0])
    delta = np.array([np.cos(orin), np.sin(orin)])
    dist = distanceCost(point, newPoint)
    for r in range(int(dist)):
        tempPoint = np.round(point + r * delta).astype(int)
        if not feasiblePoint(tempPoint, map_img):
            return False
    return True

# Load map from YAML file
map_image, resolution, origin = load_map_from_yaml('7floor2.yaml')

# Example usage
start_point = np.array([-0.416339, 0.150605])  # Example start point
target_point = np.array([1.23451, 2.29474])  # Example target point
path_list = myRRT(map_image, start_point, target_point)

# Display or use the resulting path
print("Path List:", path_list)

