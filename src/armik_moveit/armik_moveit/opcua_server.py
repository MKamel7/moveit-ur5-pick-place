"""OPC UA server for the colour-sorting cell (industrial fieldbus interface).

Exposes the cell to a PLC / SCADA / OPC UA client (e.g. UaExpert) the way a real
production cell does. A client:
  - WRITES CellController/TargetColour ("red"/"green"/"blue") to command a sort,
  - READS live process values: State, PartsSorted, per-colour counts, cycle time,
    throughput, and an alarm flag.

The server bridges OPC UA to ROS 2: a write to TargetColour is published on
/target_color (commanding the arm); telemetry from /cell/telemetry is mirrored
into the OPC UA variables.

    ros2 run armik_moveit opcua_server        # endpoint opc.tcp://0.0.0.0:4840/cell/
"""
import asyncio
import json
import threading

import rclpy
from asyncua import Server
from rclpy.node import Node
from std_msgs.msg import Bool, String

ENDPOINT = "opc.tcp://0.0.0.0:4840/cell/"
NS_URI = "http://mkamel.robotcell"


class Bridge(Node):
    def __init__(self):
        super().__init__("opcua_bridge")
        self.latest = {}
        self.safety = {}
        self._pub = self.create_publisher(String, "/target_color", 10)
        self.create_subscription(String, "/cell/telemetry", self._on_tele, 10)
        self.create_subscription(String, "/safety/state", self._on_safety, 10)
        self._safety_pubs = {
            "estop": self.create_publisher(Bool, "/safety/estop", 10),
            "guard_closed": self.create_publisher(Bool, "/safety/guard_closed", 10),
            "human_present": self.create_publisher(Bool, "/safety/human_present", 10),
            "reset": self.create_publisher(Bool, "/safety/reset", 10),
        }

    def _on_tele(self, msg):
        try:
            self.latest = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def command(self, color):
        self._pub.publish(String(data=str(color)))

    def set_safety(self, name, value):
        self._safety_pubs[name].publish(Bool(data=bool(value)))


async def run_server(bridge):
    server = Server()
    await server.init()
    server.set_endpoint(ENDPOINT)
    server.set_server_name("Colour Sorting Cell")
    idx = await server.register_namespace(NS_URI)

    cell = await server.nodes.objects.add_object(idx, "CellController")
    v_cmd = await cell.add_variable(idx, "TargetColour", "")
    await v_cmd.set_writable()
    v_state = await cell.add_variable(idx, "State", "starting")
    v_parts = await cell.add_variable(idx, "PartsSorted", 0)
    v_red = await cell.add_variable(idx, "RedCount", 0)
    v_green = await cell.add_variable(idx, "GreenCount", 0)
    v_blue = await cell.add_variable(idx, "BlueCount", 0)
    v_cycle = await cell.add_variable(idx, "LastCycleTime_s", 0.0)
    v_tput = await cell.add_variable(idx, "Throughput_ppm", 0.0)
    v_alarm = await cell.add_variable(idx, "Alarm", False)
    v_amsg = await cell.add_variable(idx, "AlarmMessage", "")

    # Safety I/O: a safety PLC writes these and reads the resulting safe state.
    safety = await server.nodes.objects.add_object(idx, "Safety")
    v_estop = await safety.add_variable(idx, "EStop", False)
    v_guard = await safety.add_variable(idx, "GuardClosed", True)
    v_human = await safety.add_variable(idx, "HumanPresent", False)
    v_reset = await safety.add_variable(idx, "Reset", False)
    for v in (v_estop, v_guard, v_human, v_reset):
        await v.set_writable()
    v_sstate = await safety.add_variable(idx, "SafetyState", "INIT")
    v_clear = await safety.add_variable(idx, "ClearToRun", False)
    v_speed = await safety.add_variable(idx, "SpeedScale", 0.0)

    async with server:
        print(f"OPC UA server up at {ENDPOINT}")
        print("  write CellController/TargetColour = red|green|blue to command a sort")
        while rclpy.ok():
            t = bridge.latest
            counts = t.get("counts", {})
            await v_state.write_value(str(t.get("state", "?")))
            await v_parts.write_value(int(t.get("parts_sorted", 0)))
            await v_red.write_value(int(counts.get("red", 0)))
            await v_green.write_value(int(counts.get("green", 0)))
            await v_blue.write_value(int(counts.get("blue", 0)))
            await v_cycle.write_value(float(t.get("last_cycle_s", 0.0)))
            await v_tput.write_value(float(t.get("throughput_ppm", 0.0)))
            await v_alarm.write_value(bool(t.get("alarm", False)))
            await v_amsg.write_value(str(t.get("alarm_msg", "")))
            # consume a commanded colour (client write -> ROS)
            cmd = await v_cmd.get_value()
            if cmd:
                bridge.command(cmd)
                await v_cmd.write_value("")

            # safety: mirror the writable inputs onto the /safety topics ...
            bridge.set_safety("estop", await v_estop.get_value())
            bridge.set_safety("guard_closed", await v_guard.get_value())
            bridge.set_safety("human_present", await v_human.get_value())
            if await v_reset.get_value():
                bridge.set_safety("reset", True)
                await v_reset.write_value(False)
            # ... and reflect the resulting safe state back
            s = bridge.safety
            await v_sstate.write_value(str(s.get("state", "INIT")))
            await v_clear.write_value(bool(s.get("clear_to_run", False)))
            await v_speed.write_value(float(s.get("speed_scale", 0.0)))
            await asyncio.sleep(0.3)


def main():
    rclpy.init()
    bridge = Bridge()
    threading.Thread(target=lambda: rclpy.spin(bridge), daemon=True).start()
    try:
        asyncio.run(run_server(bridge))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
