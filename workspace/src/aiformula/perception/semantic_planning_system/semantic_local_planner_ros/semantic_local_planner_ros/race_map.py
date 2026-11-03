"""Vehicle race map assist with first-lap red waiting and later topo detours."""

import rclpy

from .map_assist import MapAssist, startup_args


class RaceMap(MapAssist):
    """Use the shared map/vision pipeline with the race traffic policy."""

    def __init__(self) -> None:
        super().__init__(node_name="race_map")


def main(args=None) -> None:
    rclpy.init(args=startup_args(args, "race_map.yaml"))
    node = RaceMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()