#!/usr/bin/env python

from fv_lib.Fv_utils import Fv
import cv2
import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from std_msgs.msg import Header



from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import argparse
import sys
import signal

def signal_handler(signal, frame):
    print("\nprogram exiting...")
    sys.exit(0)


def image_cvt2msg(img_dx, img_dy):
    img = np.concatenate((img_dx, img_dy), axis=1)
    img = (img + 30) * 256. / 60

    img_resized = cv2.resize(img, (0, 0), fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    img_resized = np.array(img_resized, dtype=np.uint8)

    img_colorized = cv2.applyColorMap(img_resized, cv2.COLORMAP_JET)

    return img_colorized



if __name__ == '__main__':
    try:
        sys.argv = rospy.myargv()

        parser = argparse.ArgumentParser(description='Fingervision args parser')
        parser.add_argument('cam_id',type=int,  # default type is string
                            help='id of camera input')  # required args
        parser.add_argument('fitting_params_file', # default type is string
                            help='file path of linear fitting parameters')  # required args
        parser.add_argument('--interp_nx', type=int,
                            help='size of interpolation on X axis')  # optional args
        parser.add_argument('--interp_ny', type=int,
                            help='size of interpolation on Y axis')  # optional args

        args = parser.parse_args()

        if args.cam_id == 1:
            rospy.init_node('FV_L', anonymous=False)
        elif args.cam_id == 2:
            rospy.init_node('FV_R', anonymous=False)

        if args.interp_nx == None or args.interp_ny == None:
            fv= Fv(args.cam_id)
        else:
            print 'Nx Ny set.'
            fv = Fv(args.cam_id, args.interp_nx, args.interp_ny)
        path  = args.fitting_params_file

        fv.load_yaml(args.fitting_params_file)
        count = 0


        # =======================================
        if args.cam_id == 1:
            pub = rospy.Publisher('Fv/wrench_l', WrenchStamped, queue_size=10)
            pub_img = rospy.Publisher('Fv/img_l', Image, queue_size=1)

        elif args.cam_id == 2:
            pub = rospy.Publisher('Fv/wrench_r', WrenchStamped, queue_size=10)
            pub_img = rospy.Publisher('Fv/img_r', Image, queue_size=1)


        bridge = CvBridge()

        rate = rospy.Rate(15)

        wrench_msg = WrenchStamped()

        while True:
            fv.track(time_verbose=1)
            if fv.count > 1:
                wrench = fv.wrench_estimate()

                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.wrench.force.x = wrench[0]
                wrench_msg.wrench.force.y = wrench[1]
                wrench_msg.wrench.force.z = wrench[2]
                wrench_msg.wrench.torque.z = wrench[3]

                pub.publish(wrench_msg)

                # fetch disp field image (after interpolation) and publish
                img_dx = fv.vfield[:, :, 0]
                img_dy = fv.vfield[:, :, 1]

                pub_img.publish(bridge.cv2_to_imgmsg(image_cvt2msg(img_dx, img_dy), 'bgr8'))

                rate.sleep()


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            signal.signal(signal.SIGINT, signal_handler)

    except rospy.ROSInterruptException:
        pass




    fv.cap.release()
    cv2.destroyAllWindows()