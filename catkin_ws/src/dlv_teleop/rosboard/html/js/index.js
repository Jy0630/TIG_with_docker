import { createBtns } from './init_btns.js';

window.onload = function(){
  // Init elements in the control panel
  createBtns();

  // Init the pannels
  init_status_pnnl();
  init_ctrl_pnnl();
}

function init_ctrl_pnnl(){
  console.log('init_ctrl_pnnl');

  var plate_ctrl = document.getElementById('plate_ctrl');
  var turret_ctrl = document.getElementById('turret_ctrl');

  plate_ctrl.style.display = 'block';
  turret_ctrl.style.display = 'none';
}

function init_status_pnnl(){
  console.log('init_status_pnnl');

  document.getElementById('lock_btn').addEventListener('click', checkMode);
  document.getElementById('switch_btn').addEventListener('click', checkMode);
  document.getElementById('setting_btn').addEventListener('click', checkMode);
  checkMode();
}

function checkMode(){
  var setting_btn = document.getElementById('setting_btn');
  // var setting_btn_zone = document.getElementById('setting_btn_zone');

  var lock_btn = document.getElementById('lock_btn');
  var lock_btn_zone = document.getElementById('lock_btn_zone');

  var switch_btn = document.getElementById('switch_btn');
  var switch_btn_zone = document.getElementById('switch_btn_zone');

  var ctrl_pnnl_children = document.getElementById('ctrl_pnnl').children;
  var setting_zone = document.getElementById('setting_zone');
  var plate_ctrl = document.getElementById('plate_ctrl');
  var turret_ctrl = document.getElementById('turret_ctrl');

  var mode = 'Test Mode';
  
  if(setting_btn.checked){
    mode = 'Setting Mode';
  } else if(lock_btn.checked) {
    mode = 'Test Mode';
  } else if (switch_btn.checked) {
    mode = 'Game, Turret Mode';
  } else {
    mode = 'Game, Plate Mode';
  }

  switch (mode) {
  case 'Setting Mode':
    // console.log('Setting Mode');
    setting_zone.style.display = 'block';
    lock_btn_zone.style.display = 'none';
    switch_btn_zone.style.display = 'none';

    for (var i = 0; i < ctrl_pnnl_children.length; i++) {
      ctrl_pnnl_children[i].style.display = 'none';
    }

    setting_zone.style.display = 'block';
    break;

  case 'Test Mode' :
    // console.log('Test Mode');
    setting_zone.style.display = 'none';
    lock_btn_zone.style.display = 'block';
    switch_btn_zone.style.display = 'none';

    for (var i = 0; i < ctrl_pnnl_children.length; i++) {
      ctrl_pnnl_children[i].style.display = 'block';
    }

    setting_zone.style.display = 'none';
    break;

  case 'Game, Turret Mode':
    // console.log('Game, Turret Mode');
    setting_zone.style.display = 'none';
    lock_btn_zone.style.display = 'block';
    switch_btn_zone.style.display = 'block';

    for (var i = 0; i < ctrl_pnnl_children.length; i++) {
      ctrl_pnnl_children[i].style.display = 'block';
    }

    plate_ctrl.style.display = 'none';
    setting_zone.style.display = 'none';
    break;

  case 'Game, Plate Mode':
    // console.log('Game, Plate Mode');
    setting_zone.style.display = 'none';
    lock_btn_zone.style.display = 'block';
    switch_btn_zone.style.display = 'block';

    for (var i = 0; i < ctrl_pnnl_children.length; i++) {
      ctrl_pnnl_children[i].style.display = 'block';
    }
    
    turret_ctrl.style.display = 'none';
    setting_zone.style.display = 'none';
    break;
  }
}