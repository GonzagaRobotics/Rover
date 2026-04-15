from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from auto_msgs.msg import Aruco, Location, Plan, State as StateMsg, Target
from auto_msgs.srv import GetState, Instruct, SetTarget


class State(Enum):
    DISABLED = 0
    READY = 1
    PLANNING = 2
    WAITING = 3
    TRAVELING = 4
    TERMINAL_SEARCHING = 5
    TERMINAL_MOVING = 6
    PAUSED = 7
    SUCCESS = 8
    FAILURE = 9


class Auto(Node):
    _state = State.DISABLED
    # When paused, we need to remember the previous state to know what to resume to
    _paused_state = State.DISABLED
    _location = np.array([0.0, 0.0])
    _target: Target | None = None
    _plan: Plan | None = None

    def __init__(self):
        super().__init__('auto_man')

        # Outgoing to C2
        self._state_pub = self.create_publisher(StateMsg, 'auto/state', 10)
        self._plan_pub = self.create_publisher(Plan, 'auto/plan', 10)
        self._target_pub = self.create_publisher(Target, 'auto/target', 10)

        # Incoming from C2
        self.create_subscription(Bool, 'auto/enable', self._enable_cb, 10)
        self.create_service(SetTarget, 'auto/target', self._set_target_cb)
        self.create_service(Instruct, 'auto/instruct', self._instruction_cb)
        self.create_service(GetState, 'auto/state', self._get_state_cb)

        # Data from sensors
        self.create_subscription(NavSatFix, 'fix', self._fix_cb, 10)
        self.create_subscription(Aruco, 'aruco/detect', self._aruco_cb, 10)

        # Pathfinder
        self._pathfinder_pub = self.create_publisher(Target, 'pathfinder/target', 10)
        self.create_subscription(Plan, 'pathfinder/plan', self._plan_cb, 10)

    def _set_target_cb(self, req: SetTarget.Request, res: SetTarget.Response):
        if self._state not in [State.READY]:
            res.ok = False
            res.reason = 'Cannot set target in current state'
            return res

        self._set_target(req.target)
        res.ok = True

        self._set_state(State.PLANNING)
        self._pathfinder_pub.publish(self._target)

        return res

    def _instruction_cb(self, req: Instruct.Request, res: Instruct.Response):
        res.ok = False

        if req.instruction == 0:  # Pause
            if not self._is_moving():
                res.reason = 'Cannot pause when not moving'
                return res

            self._paused_state = self._state
            self._set_state(State.PAUSED)
        elif req.instruction == 1:  # Resume
            if self._state != State.PAUSED:
                res.reason = 'Cannot resume when not paused'
                return res

            self._set_state(self._paused_state)
        elif req.instruction == 2:  # Execute
            if self._state != State.WAITING:
                res.reason = 'Cannot execute when not waiting'
                return res

            self._set_state(State.TRAVELING)
        elif req.instruction == 3:  # Terminate
            if self._state in [State.PLANNING, State.READY]:
                res.reason = 'Can only terminate while doing something'
                return res

            self._set_plan(None)
            self._set_target(None)
            self._set_state(State.READY)
        else:
            res.reason = 'Invalid instruction'
            return res

        res.ok = True
        return res

    def _get_state_cb(self, _, response: GetState.Response):
        response.state = self._state.value

        response.has_target = self._target is not None
        response.target = self._target if self._target is not None else Target()

        response.has_plan = self._plan is not None
        response.plan = self._plan if self._plan is not None else Plan()

        return response

    def _enable_cb(self, msg: Bool):
        if msg.data:
            if self._state == State.DISABLED:
                self._set_state(State.READY)
        else:
            self._set_state(State.DISABLED)
            self._set_plan(None)
            self._set_target(None)

    def _plan_cb(self, msg: Plan):
        if self._state != State.PLANNING:
            self.get_logger().warn('Received plan while not planning, ignoring')
            return

        self._set_plan(msg)
        self._set_state(State.WAITING)

    def _fix_cb(self, msg: NavSatFix):
        self._location = np.array([msg.latitude, msg.longitude])

    def _aruco_cb(self, msg: Aruco):
        pass

    def _set_state(self, state: State):
        self._state = state

        msg = StateMsg()
        msg.state = state.value
        self._state_pub.publish(msg)

    def _set_plan(self, plan: Plan):
        self._plan = plan

        # Since we can't literally publish None, we publish an empty plan instead
        self._plan_pub.publish(self._plan if self._plan is not None else Plan())

    def _set_target(self, target: Target):
        self._target = target

        # If target is None, publish a target with invalid coordinates to indicate no target
        if self._target is None:
            out = Target()
            out.location.latitude = -1000
            out.location.longitude = -1000
            self._target_pub.publish(out)
        else:
            self._target_pub.publish(self._target)

    def _is_moving(self):
        return self._state in [State.TRAVELING, State.TERMINAL_MOVING, State.TERMINAL_SEARCHING]


def main(args=None):
    rclpy.init(args=args)

    auto = Auto()

    try:
        rclpy.spin(auto)
    except KeyboardInterrupt:
        pass
