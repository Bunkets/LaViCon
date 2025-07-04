#!/usr/bin/env python3
import rospy, time, cv2, numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray


class YoloTracker:
    # ─────────────────────────── SET-UP ────────────────────────────
    def __init__(self):
        rospy.init_node("yolo_tracker_node")

        # publish / subscribe
        self.task_pub  = rospy.Publisher("/task_status", Bool,  queue_size=5)
        self.cmd_pub   = rospy.Publisher("/voice_cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/camera/depth/image_raw",  Image,           self.depth_callback)
        rospy.Subscriber("/camera/color/image_raw",  Image,           self.image_callback)
        rospy.Subscriber("/reference_object",        String,          self.target_callback)
        rospy.Subscriber("/intent",                  String,          self.intent_callback)
        rospy.Subscriber("/yolo/detections",         Detection2DArray,self.dets_callback,
                         queue_size=1)

        # state variables
        self.latest_dets   = None
        self.latest_depth  = None
        self.latest_frame  = None
        self.bridge        = CvBridge()

        self.intent        = None           # go_to | face | follow | obstacle | between
        self.target_label  = None           # single-object tasks
        self.left_label    = None           # “between” tasks
        self.right_label   = None

        self.task_completed   = False
        self.center_tolerance = 0.05        # ±5 % of frame width

        # ── obstacle-avoidance FSM
        self.avoid_state       = "idle"     # idle|rotate_away|forward_clear|realign
        self.state_start_time  = None
        self.clear_distance    = 1.7        # m
        self.rotate_min_time   = 1.5        # s
        self.realign_min_time  = 1.2        # s

        # ── between-objects FSM
        self.between_state      = "align"   # align|shoot|done
        self.between_start_time = None

        # COCO → name (subset)
        self.indoor_classes = {
            0:'person',39:'bottle',56:'chair',57:'couch',58:'plant',59:'bed',
            60:'table',61:'toilet',63:'laptop',64:'mouse',65:'remote',
            66:'keyboard',67:'cell phone',69:'oven',70:'toaster',71:'sink',
            72:'refrigerator',73:'book',74:'clock',75:'vase',76:'scissors',
            77:'teddy bear',78:'hair drier',79:'toothbrush'
        }

        rospy.loginfo("YOLO Tracker node initialised.")
        self.track_loop()

    # ─────────────────────────── CALLBACKS ─────────────────────────
    def dets_callback(self, msg):   self.latest_dets  = msg
    def image_callback(self, msg):  self.latest_frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
    def depth_callback(self, msg):  self.latest_depth = self.bridge.imgmsg_to_cv2(msg,"32FC1")

    def target_callback(self, msg):
        txt = msg.data.lower().strip()
        # reset common flags
        self.avoid_state     = "idle"
        self.between_state   = "align"
        self.task_completed  = False

        if "," in txt:                                    # between X,Y
            self.left_label, self.right_label = [s.strip() for s in txt.split(",",1)]
            self.target_label = None                      # disable single-object logic
            rospy.loginfo(f"Between target: {self.left_label} | {self.right_label}")
        else:                                             # single object
            self.target_label = txt
            rospy.loginfo(f"Now tracking object: {self.target_label}")

    def intent_callback(self, msg):
        new_intent = msg.data.lower().strip()
        if new_intent != self.intent:
            self.intent          = new_intent
            self.task_completed  = False
            self.avoid_state     = "idle"
            self.between_state   = "align"
            rospy.loginfo(f"Intent set to: {self.intent}")

    # ─────────────────────────── HELPERS ───────────────────────────
    def stop_robot(self):  self.cmd_pub.publish(Twist())

    def bbox_is_close(self,bbox,w,h,a_thr=0.28,w_thr=0.45):
        x1,y1,x2,y2 = bbox; area=(x2-x1)*(y2-y1)
        return (area/(w*h) >= a_thr) or ((x2-x1)/w >= w_thr)

    def choose_largest_bbox(self,dets,label):
        if dets is None: return None
        best,area = None,0
        for d in dets.detections:
            if not d.results: continue
            if self.indoor_classes.get(d.results[0].id)!=label: continue
            cx,cy,w,h = d.bbox.center.x,d.bbox.center.y,d.bbox.size_x,d.bbox.size_y
            x1,y1,x2,y2 = cx-w/2,cy-h/2,cx+w/2,cy+h/2
            a=w*h
            if a>area: best,area=(x1,y1,x2,y2),a
        return best

    def compute_velocity_from_bbox(self,bbox,img_w):
        x1,_,x2,_ = bbox
        center = (x1+x2)/2
        offset = (center-img_w/2)/(img_w/2)
        ang = 0.0 if abs(offset)<self.center_tolerance else -0.5*offset
        lin = max(0.1,0.3*(1-abs(offset)))
        return lin,ang

    def get_depth_at_bbox_center(self,bbox):
        if self.latest_depth is None: return None
        x1,y1,x2,y2 = bbox
        cx,cy = int((x1+x2)/2), int((y1+y2)/2)
        h,w   = self.latest_depth.shape
        cx,cy = np.clip(cx,0,w-1), np.clip(cy,0,h-1)
        d = self.latest_depth[cy,cx]
        if np.isnan(d) or d<=0: return None
        return d/1000 if d>100 else d

    def _report_done(self):
        if not self.task_completed:
            self.task_pub.publish(True)
            self.stop_robot()
            self.task_completed = True

    # ─────────────────── INTENT HANDLERS (short) ───────────────────
    def handle_go_to(self,bbox,w):
        twist=Twist(); lin,ang=self.compute_velocity_from_bbox(bbox,w)
        depth=self.get_depth_at_bbox_center(bbox)
        if (depth and depth<1.0) or self.bbox_is_close(bbox,w,self.latest_frame.shape[0]):
            lin=0; self._report_done()
        twist.linear.x, twist.angular.z = lin,ang
        self.cmd_pub.publish(twist)

    def handle_face_target(self,bbox,w):
        twist=Twist(); _,ang=self.compute_velocity_from_bbox(bbox,w)
        twist.angular.z=ang
        self.cmd_pub.publish(twist)
        if abs(ang)<0.15: self._report_done()

    def handle_follow_target(self,bbox,w):
        twist=Twist(); lin,ang=self.compute_velocity_from_bbox(bbox,w)
        depth=self.get_depth_at_bbox_center(bbox)
        if depth:
            lin = -0.2 if depth<1.5 else 0.3
        twist.linear.x,twist.angular.z=lin,ang
        self.cmd_pub.publish(twist)

    # ─────────── obstacle-avoid (right-hand bypass) ────────────
    def handle_avoid_target(self,bbox,w):
        now=time.time(); twist=Twist()

        if self.avoid_state=="idle":
            if bbox and (self.get_depth_at_bbox_center(bbox) or 9e9)<self.clear_distance:
                self.avoid_state="rotate_away"; self.state_start_time=now
            else:
                lin,ang = self.compute_velocity_from_bbox(bbox,w) if bbox else (0.25,0)
                twist.linear.x,twist.angular.z=lin,ang; self.cmd_pub.publish(twist); return

        if self.avoid_state=="rotate_away":
            twist.angular.z=-0.43; self.cmd_pub.publish(twist)
            if now-self.state_start_time>=3.0:
                self.avoid_state="forward_clear"; self.state_start_time=now; return; return

        if self.avoid_state=="forward_clear":
            twist.linear.x=0.4; twist.angular.z=-0.2   # right-hand arc
            self.cmd_pub.publish(twist)
            if now-self.state_start_time>=10.0:
                self.avoid_state="realign"; self.state_start_time=now; return; return

        if self.avoid_state=="realign":
            twist.angular.z=+0.15; self.cmd_pub.publish(twist)
            if now-self.state_start_time>=self.realign_min_time:
                self._report_done(); self.avoid_state="idle"

    # ───────────── between-objects (inner-edge) ──────────────
    def handle_go_between(self,box_l,box_r,w):
        centre_tol, v_align, v_shoot, shoot_time = 0.04,0.22,0.30,13.0
        now=time.time(); twist=Twist()

        if self.between_state=="align":
            if not box_l or not box_r:        # keep last motion
                twist.linear.x=v_align; self.cmd_pub.publish(twist); return
            x_left  = box_l[2];  x_right = box_r[0]
            mid_x   = 0.5*(x_left+x_right)
            offset  = (mid_x - w/2)/(w/2)
            twist.linear.x=v_align; twist.angular.z=-0.6*offset
            self.cmd_pub.publish(twist)
            if abs(offset)<centre_tol:
                self.between_state="shoot"; self.between_start_time=now; return

        if self.between_state=="shoot":
            twist.linear.x=v_shoot; self.cmd_pub.publish(twist)
            if now-self.between_start_time>=shoot_time:
                self._report_done(); self.between_state="done"

    # ────────────────────────── MAIN LOOP ─────────────────────────
    def track_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_frame is None:
                rate.sleep(); continue
            w = self.latest_frame.shape[1]

            # — “between” intent is handled first (needs no target_label)
            if self.intent=="between":
                box_l = self.choose_largest_bbox(self.latest_dets,self.left_label)
                box_r = self.choose_largest_bbox(self.latest_dets,self.right_label)
                if box_l and box_r and (box_l[0]+box_l[2])>(box_r[0]+box_r[2]):
                    box_l,box_r = box_r,box_l
                self.handle_go_between(box_l,box_r,w)
                rate.sleep(); continue

            # — all other intents need a single target label
            if self.target_label is None:
                rate.sleep(); continue
            bbox = self.choose_largest_bbox(self.latest_dets,self.target_label)

            if self.intent=="obstacle":
                self.handle_avoid_target(bbox,w); rate.sleep(); continue

            if bbox is None:
                twist=Twist(); twist.angular.z=-0.3; self.cmd_pub.publish(twist)
                rospy.logwarn("Target not visible — searching")
                rate.sleep(); continue

            if   self.intent=="go_to":   self.handle_go_to(bbox,w)
            elif self.intent=="face":    self.handle_face_target(bbox,w)
            elif self.intent=="follow":  self.handle_follow_target(bbox,w)
            else: rospy.logwarn(f"Unknown intent: {self.intent}")

            rate.sleep()


if __name__ == "__main__":
    try:  YoloTracker()
    except rospy.ROSInterruptException:  pass
