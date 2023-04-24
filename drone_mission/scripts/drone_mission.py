import rospy
import numpy as np
import math
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from geometry_msgs.msg import PoseStamped


def get_dist(curr, des):
    return math.sqrt(pow(curr.x - des[0], 2) + pow(curr.y - des[1], 2) + pow(curr.z - des[2], 2))


class Mission:
    def __init__(self):
        self.drone_status = State()
        self.present_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.curr = self.present_pose.pose.position

        self.rate = rospy.Rate(20)
        self.t0 = rospy.get_time()

        self.rock_vec = np.array([60.2, -12.5, 21])
        self.probe_vec = np.array([40.5, 3.8, 15])
        self.rover_vec = np.array([12.6, -65.0, -3.5])

        self.distThr = 0.5

        self.local_pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.pose_cb)
        state_sub = rospy.Subscriber("/mavros/state", State, callback=self.status_cb)

    def setmode_offb(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pose_pub.publish(self.goal_pose)
            self.rate.sleep()

        # if set_mode_client.call(offb_set_mode).mode_sent == True:
        #     self.offboardCheck = True
        #     rospy.loginfo("OFFBOARD enabled")

        last_req = rospy.Time.now()
        while not rospy.is_shutdown():  # and not self.offboardCheck:
            if self.drone_status.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if set_mode_client.call(offb_set_mode).mode_sent == True:
                    # self.offboardCheck = True
                    rospy.loginfo("OFFBOARD enabled")
                    # break

                last_req = rospy.Time.now()
            else:  # if A*B else !(A*B) = !A + !B
                if not self.drone_status.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Vehicle armed")
            last_req = rospy.Time.now()

    def navigate(self, x, y, z):
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        rospy.loginfo("Desired position: x={}, y={}, z={}".format(x, y, z))
        des = [x, y, z]
        # print(self.curr)
        d = get_dist(self.curr, des)
        while d > self.distThr and not rospy.is_shutdown():
            # print("Drone pose received", self.curr)
            # self.curr = self.currPose.pose.position
            # print(d, '--' , self.curr, '--' , self.des)
            azimuth = math.atan2(self.rock_vec[1] - self.present_pose.pose.position.y,
                                 self.rock_vec[0] - self.present_pose.pose.position.x)
            # print(azimuth)
            if azimuth > math.pi:
                azimuth -= 2.0 * math.pi
            else:
                azimuth += 2.0 * math.pi
            q = quaternion_from_euler(0, 0, azimuth)
            # print(q)
            self.goal_pose.pose.orientation.x = q[0]
            self.goal_pose.pose.orientation.y = q[1]
            self.goal_pose.pose.orientation.z = q[2]
            self.goal_pose.pose.orientation.w = q[3]
            d = get_dist(self.curr, des)
            self.local_pose_pub.publish(self.goal_pose)
            self.rate.sleep()
            if d <= self.distThr:
                print(azimuth)
                print(q)
                break

    def goto_sample_probe(self):
        print("=== Going to probe location ========")
        locations = np.matrix([[40.8, 3.5, 20], ])
        for waypt in locations:
            x, y, z = waypt.tolist()[0]
            self.navigate(x, y, z)

    def pick_probe(self):
        print("=== Pick probe using visual servoing(currently position control) ========")
        locations = np.matrix([[40.8, 3.5, 12.2], [40.8, 3.5, 20]])
        for waypt in locations:
            x, y, z = waypt.tolist()[0]
            self.navigate(x, y, z)

    def goto_rock(self):
        print("=== GO to rock and Face rock ========")
        locations = np.matrix([[60, -15, 18.89], ])
        for waypt in locations:
            x, y, z = waypt.tolist()[0]
            self.navigate(x, y, z)

    def circle_rock(self, x, y, z, r, n, zh):
        print("=== Circle rock ========")
        # r = 8;
        # n = 24;
        init_ang = math.atan2(y - self.curr.y, x - self.curr.x)
        for i in range(n):
            self.goal_pose.pose.position.x = x + r * math.cos(i * 2 * math.pi / n - init_ang)
            self.goal_pose.pose.position.y = y + r * math.sin(i * 2 * math.pi / n - init_ang)
            self.goal_pose.pose.position.z = z + zh
            des = [self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, self.goal_pose.pose.position.z]
            # rospy.loginfo("Desired position: x={}, y={}, z={}".format(self.des[0], self.des[1], self.des[2]))
            azimuth = math.atan2(y - self.goal_pose.pose.position.y, x - self.goal_pose.pose.position.x)
            # print(azimuth)
            if azimuth > math.pi:
                azimuth -= 2.0 * math.pi
            else:
                azimuth += 2.0 * math.pi
            q = quaternion_from_euler(0, 0, azimuth)
            # print(q)
            self.goal_pose.pose.orientation.x = q[0]
            self.goal_pose.pose.orientation.y = q[1]
            self.goal_pose.pose.orientation.z = q[2]
            self.goal_pose.pose.orientation.w = q[3]
            rospy.loginfo(
                "Desired position: x={}, y={}, z={}, qx={}, qy = {}, qz = {}, qw = {}".format(des[0], des[1],
                                                                                              des[2], q[0], q[1],
                                                                                              q[2], q[3]))
            d = get_dist(self.curr, des)
            while d > self.distThr and not rospy.is_shutdown():
                self.local_pose_pub.publish(self.goal_pose)
                d = get_dist(self.curr, des)
                self.rate.sleep()

    def pose_cb(self, points):
        self.present_pose = points
        self.curr = self.present_pose.pose.position

    def status_cb(self, data):
        self.drone_status = data

    def launch(self):
        while not rospy.is_shutdown():
            print("Check 1")
            self.rate.sleep()
            self.setmode_offb()
            self.goto_sample_probe()
            self.pick_probe()
            self.goto_rock()
            self.circle_rock(self.rock_vec[0], self.rock_vec[1], self.rock_vec[2], 3, 16, 1)


def main():
    rospy.init_node('ProbeMission')
    rospy.loginfo("[ProbeMission] initialised")
    mission_instance = Mission()
    mission_instance.launch()
    rospy.spin()


if __name__ == '__main__':
    main()
