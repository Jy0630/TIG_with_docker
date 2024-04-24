import { ros } from './init_ros.js';
import './nipplejs.js';

// Joystick control
var cmd_vel_listener = new ROSLIB.Topic({
  ros: ros,
  name: "/dlv/cmd_vel",
  messageType: 'geometry_msgs/Twist'
});

var linear_speed = 0;
var angular_speed = 0;

function move(linear, angular) {
  var twist = new ROSLIB.Message({
    linear: {
      x: linear,
      y: 0,
      z: 0
    },
    angular: {
      x: 0,
      y: 0,
      z: angular
    }
  });
  cmd_vel_listener.publish(twist);
}

function createJoystick() {
  var options = {
    zone: document.getElementById('zone_joystick'),
    threshold: 0.1,
    position: { left: 10 + '%', top: 80 + '%'},
    mode: 'static',
    size: 120,
    color: '#3498db',
  };
  var manager = nipplejs.create(options);

  var timer = null;
  manager.on('start', function (event, nipple) {
    timer = setInterval(function () {
      move(linear_speed, angular_speed);
    }, 25);
  });

  manager.on('move', function (event, nipple) {
    var max_linear = 1.7; // m/s
    var max_angular = 1.3; // rad/s
    var max_distance = 75.0; // pixels;
    linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance / max_distance;
    angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance / max_distance;
  });

  manager.on('end', function () {
    if (timer) {
      clearInterval(timer);
    }
    move(0, 0);
  });
}

function init() {
  createJoystick();
}
window.addEventListener('load', init);