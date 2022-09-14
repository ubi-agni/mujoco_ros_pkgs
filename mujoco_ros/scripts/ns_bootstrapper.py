#*********************************************************************
#  Software License Agreement (BSD License)
#
#   Copyright (c) 2022, Bielefeld University
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of Bielefeld University nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# *********************************************************************/

# Authors: David P. Leins

from logging import shutdown
from queue import Queue
import roslaunch
import roslaunch.rlutil
import roslaunch.parent
import rospy
import time

from mujoco_ros_msgs.srv import ShutdownNS, BootstrapNS

launch_queue = Queue(1)
launch_response_queue = Queue(1)

shutdown_queue = Queue(1)
shutdown_response_queue = Queue(1)

launchers = {}

def queue_launch(req):
    launch_queue.put(req)
    return launch_response_queue.get()

def queue_shutdown(req):
    shutdown_queue.put(req)
    return shutdown_response_queue.get()

def bootstrap_ns(req):
    rospy.logdebug(f'bootstrapping ns "{req.ros_namespace}"')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    args = req.args
    parsed_args = []
    ns_included = False

    rospy.loginfo(f'Got args: {args}')

    for arg in args:
        rospy.logdebug(f"arg: {arg}")
        if req.ros_namespace in arg:
            ns_included = True
        if ':=' not in arg:
            rospy.logwarn(f"arg '{arg}' should be set with ':=', otherwise it will be ignored")
        if '%(arg ns)' in arg:
            parsed_args.append(arg.replace('%(arg ns)', req.ros_namespace))
            rospy.loginfo(f'Replaced namespace in argument {arg} to {arg.replace("%(arg ns)", req.ros_namespace)}')
        else:
            parsed_args.append(arg)

    if not ns_included:
        rospy.logdebug(f"Namespace is not supplied as argument to bootstrap launchfile, setting default arg 'ns' to namespace ({req.namespace})")
        parsed_args.append(f'ns:={req.ros_namespace}')

    try:
        parent = roslaunch.parent.ROSLaunchParent(uuid, [(req.launchfile, parsed_args)])
        parent.start()
    except Exception as e:
        rospy.logerr(f'Error while bootstrapping namespace: {e}')
        return False

    launchers[req.ros_namespace] = parent
    return True


def shutdown_ns(req):
    rospy.logdebug(f"shutting down namespace '{req.namespace}'")

    if (req.ros_namespace not in launchers.keys()):
        rospy.logwarn(f"No active namespace '{req.ros_namespace}' found!")
        return True

    try:
        parent = launchers.pop(req.ros_namespace)
        parent.shutdown()
    except Exception as e:
        rospy.logerr(f'Error while shutting down namespace: {e}')
        return False
    return True


if __name__ == '__main__':
    rospy.init_node("ns_bootstrapper", log_level=rospy.DEBUG);

    # Must be handled over queues, because roslaunch won't let us launch
    # from non-main thread
    create_ns = rospy.Service('bootstrap_ns', BootstrapNS, queue_launch)
    delete_ns = rospy.Service('shutdown_ns', ShutdownNS, queue_shutdown)

    rospy.loginfo("Ready to bootstrap namespaces")

    while not rospy.is_shutdown():
        if launch_queue.qsize() > 0:
            launch_response_queue.put(bootstrap_ns(launch_queue.get()))

        if shutdown_queue.qsize() > 0:
            shutdown_response_queue.put(shutdown_ns(shutdown_queue.get()))
        time.sleep(0.01)
