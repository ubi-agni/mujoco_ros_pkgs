from pathlib import Path

from mujoco_ros import MujocoEnv


def package_share(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except ImportError:
        import rospkg

        return Path(rospkg.RosPack().get_path(package_name))


model_path = package_share("mujoco_ros") / "assets" / "pendulum_world.xml"

plugin_config = [
    {
        "name": "python_sensors",
        "type": "mujoco_ros_sensors/MujocoRosSensorsPlugin",
    }
]

with MujocoEnv(plugin_config=plugin_config) as env:
    env.load_model_from_string(model_path)
    env.pause()

    env.settings.rt_factor = 0.5
    env.settings.gravity = [0.0, 0.0, -3.71]
    env.step(100)

    info = env.sim_info
    print(f"model: {info.model_path}")
    print(f"valid: {info.model_valid}, loads: {info.load_count}")
    print(f"rt setting: {info.rt_setting}")
    print(f"gravity: {env.settings.gravity}")

    for plugin in env.plugins:
        print(f"plugin: {plugin.name} ({plugin.type})")

    for stat in env.plugin_stats:
        print(f"{stat.name}: load={stat.load_time:.6f}s reset={stat.reset_time:.6f}s")
