import yaml
import sys


def update_config_file(new_value):
    with open("fleet_info.yaml", "r") as file:
        config = yaml.safe_load(file)

    # Update the configuration with the new value
    config["num_robots"] = new_value

    # Write back to the file
    with open("fleet_info.yaml", "w") as file:
        yaml.dump(config, file)


if __name__ == "__main__":
    print(f"----------------------{str(sys.argv)[34]}")
    update_config_file(sys.argv)
