"""Check the single-node launch without importing or starting ROS."""

import ast
from pathlib import Path
import unittest


def call_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


def constant_keyword(call, name):
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return keyword.value.value
    return None


class MapAssistLaunchTests(unittest.TestCase):
    def setUp(self):
        package = Path(__file__).resolve().parents[1]
        self.path = package / "launch" / "map_assist.launch.py"
        self.assertTrue(self.path.is_file(), f"Missing launch entry: {self.path}")
        self.tree = ast.parse(self.path.read_text(encoding="utf-8"), filename=str(self.path))

    def test_launch_starts_only_map_assist(self):
        nodes = [node for node in ast.walk(self.tree)
                 if isinstance(node, ast.Call) and call_name(node.func) == "Node"]
        self.assertEqual(len(nodes), 1, "Only the map_assist node belongs in this launch")
        self.assertEqual(constant_keyword(nodes[0], "package"), "semantic_local_planner_ros")
        self.assertEqual(constant_keyword(nodes[0], "executable"), "map_assist")

    def test_launch_has_no_included_stack_or_subprocess(self):
        calls = {call_name(node.func) for node in ast.walk(self.tree) if isinstance(node, ast.Call)}
        imported = {alias.name for node in ast.walk(self.tree)
                    if isinstance(node, (ast.Import, ast.ImportFrom)) for alias in node.names}
        for forbidden in ("IncludeLaunchDescription", "ExecuteProcess"):
            self.assertNotIn(forbidden, calls)
            self.assertNotIn(forbidden, imported)


if __name__ == "__main__":
    unittest.main()
