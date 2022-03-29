#!/usr/bin/env python3

import carla
import time
import rospy
import argparse
import pygame as pg

from numpy import random
from pygame.locals import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


class AgentGenerator:
    def __init__(self, args):
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.tm = self.client.get_trafficmanager(args.tm_port)
        self.tm.set_global_distance_to_leading_vehicle(1.0)
        self.tm.set_hybrid_physics_mode(args.hybrid)
        # self.tm.global_percentage_speed_difference(-50.0)
        self.tm.set_random_device_seed(1)

        blueprints = self.world.get_blueprint_library().filter(args.filterv)
        blueprints = [
            x for x in blueprints if int(x.get_attribute("number_of_wheels")) == 4
        ]
        blueprints = [x for x in blueprints if not x.id.endswith("isetta")]
        blueprints = [x for x in blueprints if not x.id.endswith("carlacola")]
        blueprints = [x for x in blueprints if not x.id.endswith("cybertruck")]
        blueprints = [x for x in blueprints if not x.id.endswith("t2")]
        self.blueprints = blueprints
        self.agent_list = []
        self.minimum_distance = 15
        self.spawn_offset = {
            "front": 15,
            "front_left": 10,
            "back_left": -10,
            "front_right": 10,
            "back_right": -10,
            "back": -15,
            "double_left": 0,
            "double_right": 0,
        }
        self.autopilot_enabled = False

        while True:
            actors = self.world.get_actors().filter("vehicle.*")
            if actors != None:
                self.ego = actors[0]
                print("find ego vehicle, id: {}".format(self.ego.id))
                break
            print("wait for ego vehicle")
            time.sleep(1)

    def random_blueprint(self):
        random.seed(int(time.time()))
        blueprint = random.choice(self.blueprints)
        if blueprint.has_attribute("color"):
            color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)
        if blueprint.has_attribute("driver_id"):
            driver_id = random.choice(
                blueprint.get_attribute("driver_id").recommended_values
            )
            blueprint.set_attribute("driver_id", driver_id)
        blueprint.set_attribute("role_name", "autopilot")
        return blueprint

    def get_transform(self, waypoint):
        tf = waypoint.transform
        tf.location.z += 1
        return tf

    def spawn_agent(self, transform):
        batch = [carla.command.SpawnActor(self.random_blueprint(), transform)]
        for response in self.client.apply_batch_sync(batch, False):
            if response.error:
                print("{}".format(response.error))
                return None
            else:
                self.agent_list.append(response.actor_id)
        return self.agent_list[-1]

    def get_nearby_waypoint(self, waypoint, distance):
        nearby_wp = (
            waypoint.next(distance)
            if distance >= 0
            else waypoint.previous(abs(distance))
        )
        return nearby_wp[-1] if nearby_wp else None

    def spawn_agent_at(self, location="front", given_offset=None):
        ego_waypoint = self.map.get_waypoint(self.ego.get_transform().location)
        offset = given_offset if given_offset else self.spawn_offset[location]
        spawn_point = None

        if location == "front" or location == "back":
            spawn_point = self.get_nearby_waypoint(ego_waypoint, offset)
        elif location == "front_left" or location == "back_left":
            spawn_point = ego_waypoint.get_left_lane()
            if spawn_point:
                spawn_point = self.get_nearby_waypoint(spawn_point, offset)
        elif location == "front_right" or location == "back_right":
            spawn_point = ego_waypoint.get_right_lane()
            if spawn_point:
                spawn_point = self.get_nearby_waypoint(spawn_point, offset)
        elif location == "double_left":
            spawn_point = ego_waypoint.get_left_lane().get_left_lane()
            if spawn_point:
                spawn_point = self.get_nearby_waypoint(spawn_point, given_offset)
        elif location == "double_right":
            spawn_point = ego_waypoint.get_right_lane().get_lane()
            if spawn_point:
                spawn_point = self.get_nearby_waypoint(spawn_point, given_offset)

        if not spawn_point:
            print("can't find spawn location at ego's {}".format(location))
            return False

        actor = self.spawn_agent(self.get_transform(spawn_point))
        if actor != None:
            print(
                "succeed to spawn agent at {}".format(self.get_transform(spawn_point))
            )
            if self.spawn_offset[location] >= 0:
                self.spawn_offset[location] += self.minimum_distance
            else:
                self.spawn_offset[location] -= self.minimum_distance
        return actor

    def set_auto_lane_change(self, enable):
        for actor in self.agent_list:
            self.tm.auto_lane_change(self.world.get_actor(actor), enable)

    def set_ignore_light_percentage(self, percentage):
        for actor in self.agent_list:
            self.tm.ignore_lights_percentage(self.world.get_actor(actor), percentage)

    def set_speed_percentage_difference(self, actor, percentage):
        actor = self.world.get_actor(actor)
        self.tm.vehicle_percentage_speed_difference(actor, percentage)

    def trigger_autopilot(self):
        port = self.tm.get_port()
        print("{} autopilot".format("Disable" if self.autopilot_enabled else "Enable"))
        for agent in self.agent_list:
            self.world.get_actor(agent).set_autopilot(~self.autopilot_enabled, port)
        self.autopilot_enabled = not self.autopilot_enabled

    def destory(self):
        print("\ndestroying %d agents" % len(self.agent_list))
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in self.agent_list]
        )
        time.sleep(0.5)


class KeyboardHandler:
    def __init__(self, args):
        self.cmd_count = 0
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.ego_cmd_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        if not args.joy_only:
            self.agent_generator = AgentGenerator(args)

    def init_joy(self):
        joy = Joy()
        joy.header.frame_id = "map"
        joy.header.stamp = rospy.Time.now()
        for i in range(8):
            joy.axes.append(0)
        for i in range(11):
            joy.buttons.append(0)
        return joy

    def publish_joy(self, joy):
        self.cmd_count += 1
        self.joy_pub.publish(joy)

    def publish_ego_start_cmd(self):
        cmd = PoseStamped()
        cmd.header.frame_id = "map"
        cmd.header.stamp = rospy.Time.now()
        self.ego_cmd_pub.publish(cmd)

    def update(self):
        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                joy = self.init_joy()
                if event.key == pg.K_w:
                    print("{}: Increse reference speed.".format(self.cmd_count))
                    joy.buttons[3] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_s:
                    print("{}: Decrease reference speed.".format(self.cmd_count))
                    joy.buttons[0] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_a:
                    print("{}: Suggest left lane change.".format(self.cmd_count))
                    joy.buttons[2] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_d:
                    print("{}: Suggest right lane change.".format(self.cmd_count))
                    joy.buttons[1] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_1:
                    print("add 1 obstacle")
                    joy.buttons[5] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_2:
                    print("add 2 obstacle")
                    joy.buttons[5] = 2
                    self.publish_joy(joy)
                elif event.key == pg.K_3:
                    print("add 3 obstacle")
                    joy.buttons[5] = 3
                    self.publish_joy(joy)
                elif event.key == pg.K_i:
                    self.agent_generator.spawn_agent_at("front")
                elif event.key == pg.K_k:
                    self.agent_generator.spawn_agent_at("back")
                elif event.key == pg.K_u:
                    self.agent_generator.spawn_agent_at("front_left")
                elif event.key == pg.K_j:
                    self.agent_generator.spawn_agent_at("back_left")
                elif event.key == pg.K_o:
                    self.agent_generator.spawn_agent_at("front_right")
                elif event.key == pg.K_l:
                    self.agent_generator.spawn_agent_at("back_right")
                elif event.key == pg.K_e:
                    _ = self.agent_generator.spawn_agent_at("front", 10)
                    _ = self.agent_generator.spawn_agent_at("front", 35)
                    _ = self.agent_generator.spawn_agent_at("front_left", 50)
                    _ = self.agent_generator.spawn_agent_at("back_left", -15)
                    _ = self.agent_generator.spawn_agent_at("front_left", 100)
                    _ = self.agent_generator.spawn_agent_at("double_left", 30)
                    _ = self.agent_generator.spawn_agent_at("double_left", 60)
                    _ = self.agent_generator.spawn_agent_at("front_right", 5)
                    _ = self.agent_generator.spawn_agent_at("front_right", 70)
                    self.agent_generator.set_auto_lane_change(False)
                    self.agent_generator.set_ignore_light_percentage(100.0)
                elif event.key == pg.K_b:
                    print("Trigger agents' autopilot")
                    self.publish_ego_start_cmd()
                    self.agent_generator.trigger_autopilot()

    def destroy(self):
        self.agent_generator.destory()
        pass


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    argparser.add_argument(
        "--tm-port",
        metavar="P",
        default=8000,
        type=int,
        help="port to communicate with TM (default: 8000)",
    )
    argparser.add_argument(
        "--filterv",
        metavar="PATTERN",
        default="vehicle.*",
        help='vehicles filter (default: "vehicle.*")',
    )
    argparser.add_argument(
        "--hybrid",
        default=False,
        action="store_true",
        help="Enanble hybrid physics model",
    )
    argparser.add_argument(
        "--joy-only",
        default=False,
        help="joy only model",
    )
    argparser.add_argument("--img", type=str, required=True)
    args, unknown = argparser.parse_known_args()

    rospy.init_node("key2joy")
    rate = rospy.Rate(10)

    pg.init()
    screen = pg.display.set_mode((886, 542))
    img = pg.image.load(args.img)
    keyboard_handler = KeyboardHandler(args)

    try:
        while not rospy.is_shutdown():
            keyboard_handler.update()
            screen.blit(img, (1, 1))
            pg.display.flip()
            rate.sleep()
    finally:
        keyboard_handler.destroy()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pg.quit()
