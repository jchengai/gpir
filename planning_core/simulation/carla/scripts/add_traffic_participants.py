import carla
import numpy as np
import math

def spawn_vehicles_around_ego_vehicles(ego_vehicle, radius, spawn_points, numbers_of_vehicles):
    # parameters:
    # ego_vehicle :: your target vehicle
    # radius :: the distance limitation between ego-vehicle and other free-vehicles
    # spawn_points  :: the available spawn points in current map
    # numbers_of_vehicles :: the number of free-vehicles around ego-vehicle that you need
    np.random.shuffle(spawn_points)  # shuffle  all the spawn points
    ego_location = ego_vehicle.get_location()
    accessible_points = []
    for spawn_point in spawn_points:
        dis = math.sqrt((ego_location.x-spawn_point.location.x)**2 + (ego_location.y-spawn_point.location.y)**2)
        # it also can include z-coordinate,but it is unnecessary
        if dis < radius:
            print(dis)
            accessible_points.append(spawn_point)

    vehicle_bps = world.get_blueprint_library().filter('vehicle.*.*')   # don't specify the type of vehicle
    vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]  # only choose car with 4 wheels

    vehicle_list = []  # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
    if len(accessible_points) < numbers_of_vehicles:
        # if your radius is relatively small,the satisfied points may be insufficient
        numbers_of_vehicles = len(accessible_points)

    for i in range(numbers_of_vehicles):  # generate the free vehicle
        point = accessible_points[i]
        vehicle_bp = np.random.choice(vehicle_bps)
        try:
            vehicle = world.spawn_actor(vehicle_bp, point)
            vehicle_list.append(vehicle)
        except:
            print('failed')  # if failed, print the hints.
            pass
# you also can add those free vehicle into trafficemanager,and set them to autopilot.
# Only need to get rid of comments for below code. Otherwise, the those vehicle will be static
    tm = client.get_trafficmanager()  # create a TM object
    tm.global_percentage_speed_difference(1.0)  # set the global speed limitation
    tm_port = tm.get_port()  # get the port of tm. we need add vehicle to tm by this port
    for v in vehicle_list:  # set every vehicle's mode
        v.set_autopilot(True, tm_port)  # you can get those functions detail in carla document
        tm.ignore_lights_percentage(v, 0)
        tm.distance_to_leading_vehicle(v, 0.5)
        # tm.vehicle_percentage_speed_difference(v, -20)
# create client object
client = carla.Client("localhost", 2000)
client.set_timeout(10)

# create world object
world = client.load_world('Town05')
spawn_points = world.get_map().get_spawn_points()
model3_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
model3_bp.set_attribute('color', '255,0,0')   # set vehicle's color to red
model3_spawn_point = np.random.choice(spawn_points)
model3_actor = world.spawn_actor(model3_bp, model3_spawn_point)
spawn_vehicles_around_ego_vehicles(ego_vehicle=model3_actor, radius=10,
                                   spawn_points=spawn_points, numbers_of_vehicles=10)
 # model3_actor.set_autopilot(True)  # set the ego-vehicle to autopilot