from setuptools import setup

package_name = "correction_controller_trainer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="debug",
    maintainer_email="debug@example.com",
    description="Joystick-command logger and online RPM correction trainer.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joy_trainer = correction_controller_trainer.scripted_trainer:main",
            "scripted_trainer = correction_controller_trainer.scripted_trainer:main",
            "evaluate_rpm_behavior = correction_controller_trainer.evaluate_rpm_behavior:main",
            "train_rpm_startpoint = correction_controller_trainer.train_rpm_startpoint:main",
            "visualize_online_run = correction_controller_trainer.visualize_online_run:main",
        ],
    },
)
