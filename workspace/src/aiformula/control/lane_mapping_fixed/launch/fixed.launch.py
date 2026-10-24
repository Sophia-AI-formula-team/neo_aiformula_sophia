"""First-lap map, then fixed route after the managed LYA has stopped."""
from lane_mapping_lya_reference.launch_support import generate_learning_launch


def generate_launch_description():
    return generate_learning_launch("fixed_only")
