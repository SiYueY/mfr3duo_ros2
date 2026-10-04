"""Actual public TaskSequence across navigation and physical Pick/Place."""
import math
import os
import re
import signal
import subprocess
import tempfile
import time
def verify_scene_alignment():
    import rclpy
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene

    def rotate(q, v):
        # CollisionObject.pose may have an unset identity quaternion in Humble.
        if q.x == q.y == q.z == q.w == 0:
            return v
        u = (q.x, q.y, q.z)
        cross = lambda a, b: (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
        uv = cross(u, v)
        uuv = cross(u, uv)
        return tuple(v[i] + 2*(q.w*uv[i]+uuv[i]) for i in range(3))

    def center(obj):
        primitive = obj.primitive_poses[0].position
        offset = rotate(obj.pose.orientation, (primitive.x, primitive.y, primitive.z))
        origin = obj.pose.position
        return tuple(a+b for a, b in zip((origin.x, origin.y, origin.z), offset))

    rclpy.init()
    node = rclpy.create_node('robot_demo_scene_verification')
    try:
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('PlanningScene verification service unavailable')
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        if not future.done() or future.result() is None:
            raise RuntimeError('PlanningScene verification deadline')
        scene = future.result().scene
        objects = {obj.id: obj for obj in scene.world.collision_objects}
        table, box = objects['grasp_table'], objects['box']
        if table.header.frame_id != box.header.frame_id:
            raise RuntimeError('Support and released object use different scene frames')
        if any(obj.object.id == 'box' for obj in scene.robot_state.attached_collision_objects):
            raise RuntimeError('Released object still attached')
        expected = (.24, .24, .9998)
        if any(abs(x-y) > 1e-9 for x, y in zip(table.primitives[0].dimensions, expected)):
            raise RuntimeError('Fixed planning support contact skin changed')
        delta = tuple(b-t for b, t in zip(center(box), center(table)))
        # Table primitive is axis aligned in simulation_world; include both scene rotations.
        from geometry_msgs.msg import Quaternion
        a, b = table.pose.orientation, table.primitive_poses[0].orientation
        if a.x == a.y == a.z == a.w == 0:
            a = Quaternion(w=1.0)
        q = Quaternion(x=-(a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y),
                       y=-(a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x),
                       z=-(a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w),
                       w=a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z)
        local = rotate(q, delta)
        error = math.sqrt(sum((x-y)**2 for x, y in zip(local, (-.08, 0, .525))))
        if error > .01:
            raise RuntimeError(f'Post-navigation support/object scene mismatch: {local}, error={error}')
        print(f'ROBOT_SCENE_ALIGNMENT_PASS error={error:.6f} m', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-phase6-demo-', suffix='.log') as log:
    print('Runtime log:', log.name, flush=True)
    process = subprocess.Popen(['ros2', 'launch', 'mfr3duo_robot', 'robot.launch.py',
                                'viewer_enabled:=' + os.environ.get('MFR3DUO_TEST_VIEWER', 'false'),
                                'run_demo:=true'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 300
        while time.monotonic() < deadline:
            log.seek(0)
            output = log.read()
            if 'ROBOT_DEMO_FAIL' in output or 'process has died' in output or process.poll() is not None:
                raise RuntimeError(output)
            if 'ROBOT_DEMO_PASS' in output and re.search(r'\[task_demo-\d+\]: process has finished cleanly', output):
                print(output)
                verify_scene_alignment()
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Whole TaskSequence deadline\n' + output)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
