# lslidar-docker


## How to use

### Configuration

1. Connect to the Panther's WiFi network, and open an SSH session on the NUC.
  
    ```bash
    ssh husarion@10.15.20.3
    ```

1. Clone this repository on the Panther robot.

    ```bash
    git clone https://github.com/husarion/lslidar-docker.git
    ```

1. [OPTIONAL] Configure sensor placement in the [config.env](./config/config.env) file.
1. [OPTIONAL] Provide custom network configuration in the [net.env](./config/net.env) file.

### Running

1. Run the driver.

    ```bash
    cd lslidar-docker
    docker compose up -d
    ```

