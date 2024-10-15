# ROS Bag WebSocket Server

This project sets up a WebSocket server using rosbridge, which streams data recorded in a ROS bag file over port 9090. Follow the instructions below to get started.

## Getting Started

### Clone and Build the Project

First, clone this project from GitHub:

```bash
git clone <repository_url>
```

Next, build the Docker image:

```bash
cd <project_directory>
docker compose build
```

### for Windows User
If you are using Windows, you can load docker-compose.windows.yml with the following command:

```bash
docker compose -f docker-compose.windows.yml build
```

For subsequent docker compose commands, make sure to specify this file using the -f option.

### Add ROS Bag File

Download your desired ROS bag file and place it in the `./bag` directory of the project. This step is essential to stream the data properly.

### Start the Server

To start the WebSocket server and stream data from the ROS bag file, use the following command:

```bash
docker compose up
```

This will launch the rosbridge WebSocket server on port 9090.

## Modify the Target File

If you need to change the file that is being played, modify the following script:

`workspace/src/websocket_server/scripts/bagfile_player.py`

Update the following lines as needed:

```python
#################### Configuration ####################
bag_dir = '/root/bag'

# when target_bag is empty, all bag files in bag_dir will be played
target_bag = ''

# if is_loop is True, all bag files will be played in loop
is_loop = True
#################### Configuration ####################
```

## Connect with ROSLIB

To access the data, connect to the WebSocket server from your application using [ROSLIB.js](http://wiki.ros.org/roslibjs). Here is a simple example of how to connect to the server:

```javascript
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to WebSocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to WebSocket server:', error);
});

ros.on('close', function() {
  console.log('Connection to WebSocket server closed.');
});
```

## License

This project is licensed under the MIT License.
