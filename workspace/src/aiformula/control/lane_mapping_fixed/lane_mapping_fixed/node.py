"""Force fixed-only semantics even if a reference-mode YAML is supplied."""
from lane_mapping_lya_reference.follower_node import main_fixed


def main(args=None):
    return main_fixed(args=args)
