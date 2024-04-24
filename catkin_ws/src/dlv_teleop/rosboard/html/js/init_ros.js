import "./roslib.js";

// Connection
export const ros = new ROSLIB.Ros({
  // Change to robot ip address 
  // 9090 is the default port of rosbridge server
  url: 'ws://' + window.local_ip + ':9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});