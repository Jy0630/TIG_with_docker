import { ros } from './init_ros.js';

var camera_listener = new ROSLIB.Topic({
  ros: ros,
  name: "/golfbot/camera_web",
  messageType: 'std_msgs/String'
});

camera_listener.subscribe(function(msg) {
  var canvas = document.getElementById('rgb-canvas');
  var ctx = canvas.getContext('2d');
  var image = new Image();
  image.onload = function() {
    ctx.drawImage(image, 0, 0);
  };
  image.src = `data:image/png;base64,${msg.data}`;
});