"""First-lap map, then fixed route with an independent live LYA reference."""
from lane_mapping_lya_reference.launch_support import generate_learning_launch


def generate_launch_description():
    return generate_learning_launch("lya_reference")
