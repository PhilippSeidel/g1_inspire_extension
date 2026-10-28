import multiprocessing as mp
import traceback

from incar_networking.robot_interface import IncarRobotInterface
from inspire_extension.robot_interface.inspire_interface import openSerial, readRegister, regdict, write6

HAND_ID = 1

OPEN_POSE = [1000, 1000, 1000, 1000, 1000, 1000]
CLOSED_POSE = [0, 0, 0, 0, 400, 1000]  # thumb not fully bent, so it stays clear of the closed fingers

CLOSE_THRESHOLD = 0.75
OPEN_THRESHOLD = 0.25


class InspireGripper:

    def __init__(self, port: str = "/dev/ttyUSB0", module_name: str = "right_hand"):
        self.module_name = module_name
        ctx = mp.get_context("fork")
        self._enabled = ctx.Event()
        self._closed = ctx.Value("b", False)
        self._has_state = ctx.Value("b", False)
        self._angles = ctx.Array("i", 6)
        self._forces = ctx.Array("i", 6)
        self._process = ctx.Process(
            target=_hand_process,
            args=(port, self._enabled, self._closed, self._has_state, self._angles, self._forces),
            daemon=True,
        )
        self._process.start()

    def start(self) -> None:
        self._enabled.set()

    def on_gripper(self, openclose: list[float]) -> None:
        if openclose[0] >= CLOSE_THRESHOLD:
            self._closed.value = True
        elif openclose[0] <= OPEN_THRESHOLD:
            self._closed.value = False

    def on_loop(self, interface: IncarRobotInterface) -> None:
        """Loop callback; doesn't publish, so register it before a callback that does."""
        if not self._has_state.value:
            return
        interface.set_robot_state(self.module_name, joint_pos=list(self._angles), joint_effort=list(self._forces))


def _hand_process(port, enabled, closed, has_state, angles, forces):
    try:
        enabled.wait()
        ser = openSerial(port, 115200)
        write6(ser, HAND_ID, "speedSet", [1000] * 6)
        write6(ser, HAND_ID, "forceSet", [1000] * 6)
        write6(ser, HAND_ID, "angleSet", OPEN_POSE)
        is_closed = False
        print("Inspire hand initialised, fingers opening")
        while True:
            if bool(closed.value) != is_closed:
                is_closed = bool(closed.value)
                write6(ser, HAND_ID, "angleSet", CLOSED_POSE if is_closed else OPEN_POSE)
            current_angles = _read6(ser, "angleAct")
            current_forces = _read6(ser, "forceAct")
            if current_angles is not None and current_forces is not None:
                angles[:] = current_angles
                forces[:] = current_forces
                has_state.value = True
    except KeyboardInterrupt:
        pass
    except Exception:
        print("Inspire hand process stopped:")
        traceback.print_exc()


def _read6(ser, key):
    """read6 without its console print on every read; None if the reply is missing or incomplete."""
    try:
        raw = readRegister(ser, HAND_ID, regdict[key], 12, mute=True)
    except IndexError:  
        return None
    if len(raw) < 12:
        return None
    values = [raw[2 * i] | (raw[2 * i + 1] << 8) for i in range(6)]
    return [v - 65536 if v > 32767 else v for v in values]
