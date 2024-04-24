import { ros } from './init_ros.js';

export function createBtns() {
  createLockBtn();
  createSettingsBtn();
  createLaunchBtn();
  createFullscreenBtn();
  createSwitchBtn();
  createDpad();
  createSelectBar();
  createServoSlider();
  createServoMethod();
  createWheelMethod();
}

function createLockBtn() {
  // Create Input
  var input = document.createElement('input');
  input.type = 'checkbox';
  input.id = 'lock_btn';
 
  // Create label
  const label = document.createElement('label');
  label.setAttribute('for', 'lock_btn');
  label.classList.add('lock-label');

  // Create lock-wrapper span
  const lockWrapper = document.createElement('span');
  lockWrapper.classList.add('lock-wrapper');

  // Create shackle span
  const shackle = document.createElement('span');
  shackle.classList.add('shackle');

  // Create SVG
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.classList.add('lock-body');
  svg.setAttribute('viewBox', '0 0 28 28');
  svg.setAttribute('fill', 'none');

  // Create path
  const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
  path.setAttribute('fill-rule', 'evenodd');
  path.setAttribute('clip-rule', 'evenodd');
  path.setAttribute('d', 'M0 5C0 2.23858 2.23858 0 5 0H23C25.7614 0 28 2.23858 28 5V23C28 25.7614 25.7614 28 23 28H5C2.23858 28 0 25.7614 0 23V5ZM16 13.2361C16.6137 12.6868 17 11.8885 17 11C17 9.34315 15.6569 8 14 8C12.3431 8 11 9.34315 11 11C11 11.8885 11.3863 12.6868 12 13.2361V18C12 19.1046 12.8954 20 14 20C15.1046 20 16 19.1046 16 18V13.2361Z');
  path.setAttribute('fill', 'white');

  // Append elements
  svg.appendChild(path);
  lockWrapper.appendChild(shackle);
  lockWrapper.appendChild(svg);
  label.appendChild(lockWrapper);

  // Append
  document.getElementById('lock_btn_zone').appendChild(input);
  document.getElementById('lock_btn_zone').appendChild(label);
}

function createSettingsBtn() {
  // Create Input
  var input = document.createElement('input');
  input.type = 'checkbox';
  input.id = 'setting_btn';
 
  // Create label
  const label = document.createElement('label');
  label.setAttribute('for', 'setting_btn');
  label.classList.add('setting-label');

  // Create lock-wrapper span
  const lockWrapper = document.createElement('span');
  lockWrapper.classList.add('setting-wrapper');

  // Create shackle span
  const shackle = document.createElement('span');
  shackle.classList.add('shackle');

  // Create Font Awesome icon
  const icon = document.createElement('i');
  icon.classList.add('fa-solid', 'fa-gear');

  // Append icon to label
  label.appendChild(icon);

  // Append
  document.getElementById('setting_btn_zone').appendChild(input);
  document.getElementById('setting_btn_zone').appendChild(label);

  input.addEventListener('change', function() {
    if (this.checked) {
      icon.className = '';
      icon.classList.add('fa-solid', 'fa-xmark');
    } else {
      icon.className = '';
      icon.classList.add('fa-solid', 'fa-gear');
    }
  });
}

function createLaunchBtn() {
  var launch_btn = document.createElement('button');
  launch_btn.className = 'launch_btn';
  launch_btn.id = 'launch_btn';
  launch_btn.innerHTML = 'launch';

  var launch_publisher = new ROSLIB.Topic({
    ros: ros,
    name: "/golfbot/launch",
    messageType: 'std_msgs/String'
  });

  let pressTimer;
  launch_btn.addEventListener('touchstart', function() {
    this.classList.add('pressed');
    pressTimer = setTimeout(() => {
      console.log('Button activated');
      launch_publisher.publish({ data: 'launch' });
    }, 1000);
  });

  launch_btn.addEventListener('touchend', function() {
    clearTimeout(pressTimer);
    this.classList.remove('pressed');
  });

  document.getElementById('launch_btn_zone').appendChild(launch_btn);
}

function createFullscreenBtn() {
  var fullscreen_btn = document.createElement('button');
  fullscreen_btn.className = 'full-screen';
  fullscreen_btn.title="Enter full screen mode";

  var svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("height", "14px");
  svg.setAttribute("version", "1.1");
  svg.setAttribute("viewBox", "0 0 14 14");
  svg.setAttribute("width", "14px");

  var path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", "M2,9 L0,9 L0,14 L5,14 L5,12 L2,12 L2,9 L2,9 Z M0,5 L2,5 L2,2 L5,2 L5,0 L0,0 L0,5 L0,5 Z M12,12 L9,12 L9,14 L14,14 L14,9 L12,9 L12,12 L12,12 Z M9,0 L9,2 L12,2 L12,5 L14,5 L14,0 L9,0 L9,0 Z");
  svg.appendChild(path);

  fullscreen_btn.appendChild(svg);

  document.getElementById('full_screen_btn_zone').appendChild(fullscreen_btn);

  const fullscreenButton = document.querySelector(".full-screen");
  fullscreenButton.addEventListener("click", function () {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen();
    }
    fullscreenButton.style.display = 'none'; // Hide the button
    fullscreenButton.disabled = true; // Disable the button
  });

  document.addEventListener('fullscreenchange', function () {
    console.log('Toggle fullscreen mode');

    if (!document.fullscreenElement) {
      fullscreenButton.style.display = 'block'; // Show the button
      fullscreenButton.disabled = false; // Enable the button
    }
  });
}

function createSwitchBtn() {
  var input = document.createElement('input');
  input.type = 'checkbox';
  input.id = 'switch_btn';

  const label = document.createElement('label');
  label.setAttribute('for', 'switch_btn');
  label.classList.add('switch_btn_label');

  const slider = document.createElement('span');
  slider.classList.add('slider', 'round');

  const onSpan = document.createElement('span');
  onSpan.classList.add('on');
  const onIcon = document.createElement('i');
  onIcon.classList.add('fa-solid', 'fa-person-biking');
  onSpan.appendChild(onIcon);

  const offSpan = document.createElement('span');
  offSpan.classList.add('off');
  const offIcon = document.createElement('i');
  offIcon.classList.add('fa-solid', 'fa-fire');
  offSpan.appendChild(offIcon);

  slider.appendChild(onSpan);
  slider.appendChild(offSpan);
  label.appendChild(slider);

  // Append the input and label to the body or another container
  document.getElementById('switch_btn_zone').appendChild(input);
  document.getElementById('switch_btn_zone').appendChild(label);
}

function createDpad() {
  var buttonConfigs = [
    { id: 'up', innerHTML: 'U' },
    { id: 'down', innerHTML: 'D' },
  ];

  var dpad_publisher = new ROSLIB.Topic({
    ros: ros,
    name: "/golfbot/dpad",
    messageType: 'std_msgs/String'
  });

  buttonConfigs.forEach(function(config) {
    var btn = document.createElement('button');
    btn.id = config.id;
    btn.innerHTML = config.innerHTML;

    var pressTimer;

    btn.addEventListener('touchstart', function() {
      this.classList.add('pressed');
      dpad_publisher.publish({ data: config.id });
  
      // Start a timer when the button is pressed
      pressTimer = window.setInterval(function() {
        dpad_publisher.publish({ data: config.id });
      }, 500); // Publish the message every 500 milliseconds
    });
  
    btn.addEventListener('touchend', function() {
      this.classList.remove('pressed');
      window.clearInterval(pressTimer);
    });

    document.getElementById('dpad_zone').appendChild(btn);
  });
}

function createSelectBar() {
  var select = document.createElement('select');
  select.size = '5';
  select.classList.add('form-control');

  const options = ['One', 'Two', 'Three', 'Four', 'Five', 'Six', 'Seven', 'Eight', 'Nine', 'Ten'];

  options.forEach(optionText => {
    var option = document.createElement('option');
    option.value = '';
    option.text = optionText;
    select.appendChild(option);
  });

  var current_hole_publisher = new ROSLIB.Topic({
    ros: ros,
    name: "/golfbot/current_hole",
    messageType: 'std_msgs/String'
  });

  select.addEventListener('change', function() {
    current_hole_publisher.publish({ data: select.selectedIndex });
  });

  // Append the select to the zone
  document.getElementById("select_bar_zone").appendChild(select);
}

function createServoSlider() {
  var sliderConfigs = [
    { id: 'fl-lift', type: 'range', min: '0', max: '180', value: '0', leg: 'FL', motion: 'LIFT' },
    { id: 'rl-lift', type: 'range', min: '0', max: '180', value: '0', leg: 'RL', motion: 'LIFT' },
    { id: 'fr-lift', type: 'range', min: '0', max: '180', value: '0', leg: 'FR', motion: 'LIFT' },
    // { id: 'rr-lift', type: 'range', min: '0', max: '180', value: '0', leg: 'RR', motion: 'LIFT' },
    // { id: 'fl-rotate', type: 'range', min: '0', max: '180', value: '0', leg: 'FL', motion: 'ROTATE' },
    // { id: 'rl-rotate', type: 'range', min: '0', max: '180', value: '0', leg: 'RL', motion: 'ROTATE' },
    // { id: 'fr-rotate', type: 'range', min: '0', max: '180', value: '0', leg: 'FR', motion: 'ROTATE' },
    // { id: 'rr-rotate', type: 'range', min: '0', max: '180', value: '0', leg: 'RR', motion: 'ROTATE' },
  ];

  sliderConfigs.forEach(function(config) {
    var slider = document.createElement('input');
    slider.id = config.id;
    slider.type = config.type;
    slider.min = config.min;
    slider.max = config.max;
    slider.value = config.value;

    // Detect double touch
    const initialValue = config.value;
    let lastTouchEnd = 0;
    slider.addEventListener('touchend', function (event) {
      const now = (new Date()).getTime();
      if (now - lastTouchEnd <= 300) {
        // Double touch detected
        slider.value = initialValue;
      }
      lastTouchEnd = now;
    }, false);

    var slider_publisher = new ROSLIB.Topic({
      ros: ros,
      name: "/golfbot/motors",
      messageType: 'golfbot_msgs/motor'
    });
  
    slider.addEventListener('touchmove', function() {
      var motor_cmd = new ROSLIB.Message({
        leg: config.leg,
        motion: config.motion,
        value: parseFloat(slider.value)
      });
      slider_publisher.publish(motor_cmd);
      // console.log(motor_cmd);
    });

    document.getElementById("slider_zone").appendChild(slider);
  });
}

function createServoMethod(){
  var select = document.createElement('select');
  select.size = '5';
  select.classList.add('form-control');

  const options = ['Seperated control', 'Combined control'];

  options.forEach(optionText => {
    var option = document.createElement('option');
    option.value = '';
    option.text = optionText;
    select.appendChild(option);

    if (optionText === 'Combined control') {
      option.selected = true;
    }
  });

  // Append the select to the zone
  document.getElementById("servo_method_zone").appendChild(select);
}

function createWheelMethod(){
  var select = document.createElement('select');
  select.size = '5';
  select.classList.add('form-control');

  const options = ['Omnidirectional', 'Omnidirectional - Fixed Heading', 'Differential'];

  options.forEach(optionText => {
    var option = document.createElement('option');
    option.value = '';
    option.text = optionText;
    select.appendChild(option);
    
    if (optionText === 'Omnidirectional') {
      option.selected = true;
    }
  });

  // Append the select to the zone
  document.getElementById("wheel_method_zone").appendChild(select);
}