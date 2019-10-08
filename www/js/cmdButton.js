/*
*
*   cmdButton handling script
*
*/

// Update the current slider value (each time you drag the slider handle)

var $element = $('input[type="range"]');

$element
  .rangeslider({
    polyfill: false,
    onInit: function() {
      var $handle = $('.rangeslider__handle', this.$range);
      updateHandle($handle[0], this.value);
    }
  })
  .on('input', function(e) {
    var $handle = $('.rangeslider__handle', e.target.nextSibling);
    updateHandle($handle[0], this.value);
    CTRL_CMD_VALUE.DOOR = this.value;
    JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
    websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
  });

function updateHandle(el, val) {
  el.textContent = val;
}

var ANI_btnSolderingSequence = document.getElementById("btnSolderingSequence");
var ANI_btnHeatingElements = document.getElementById("btnHeatingElements");
var ANI_btnConvectionFan = document.getElementById("btnConvectionFan");
//var ANI_btnDoor = document.getElementById("btnDoor");

ANI_btnSolderingSequence.addEventListener("animationend", AnimationEndListener, false);
ANI_btnHeatingElements.addEventListener("animationend", AnimationEndListener, false);
ANI_btnConvectionFan.addEventListener("animationend", AnimationEndListener, false);
//ANI_btnDoor.addEventListener("animationend", AnimationEndListener, false);

//btnDoor.oninput = function() {
//  CTRL_CMD_VALUE.DOOR = ANI_btnDoor.value;
//  JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
//  websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
//}

function AnimationEndListener(){
    if (this.id == "btnSolderingSequence"){
      CTRL_CMD_VALUE.SEQ_RUN = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.run_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (this.id == "btnHeatingElements"){
      CTRL_CMD_VALUE.HEATER = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.heater_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (this.id == "btnConvectionFan"){
      CTRL_CMD_VALUE.FAN = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.fan_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (this.id == "btnDoor"){
      CTRL_CMD_VALUE.DOOR = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
    }

    this.classList.remove('startup');
    this.classList.remove('sroGreen');
    this.classList.remove('sroYellow');
    this.classList.remove('sroBtnGray');
    this.classList.add('error');
    this.classList.add('sroRed');
    this.textContent = "ERROR";
}

$(document).ready(function() {
  cmdButtonsDisable(true);
});

/* animation stuff */
function cmdBtn_ClearColorClasses(_animation){
  _animation.classList.remove('sroRed');
  _animation.classList.remove('sroYellow');
  _animation.classList.remove('sroGreen');
  _animation.classList.remove('sroBtnGray');
}

function cmdBtnAni(animation){

  cmdBtn_ClearColorClasses(animation);
  let error = animation.classList.contains('error');
  let startup = animation.classList.contains('startup');
  let running = animation.classList.contains('running');

  if (!startup && !running && !error) {
    animation.classList.add('startup');
    animation.textContent = "sending command...";
    if (animation.id == "btnSolderingSequence"){
      CTRL_CMD_VALUE.SEQ_RUN = 1;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.run_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (animation.id == "btnHeatingElements"){
      CTRL_CMD_VALUE.HEATER = 1;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.heater_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (animation.id == "btnConvectionFan"){
      CTRL_CMD_VALUE.FAN = 1;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.fan_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
  } else {
      animation.classList.remove('startup');
      cmdBtn_ClearColorClasses(animation);
      animation.classList.add('sroBtnGray');
      animation.textContent = animation.dataset.txtoff;
    }

  if (error) {
    animation.classList.remove('error');
    cmdBtn_ClearColorClasses(animation);
    animation.classList.add('sroBtnGray');
    animation.textContent = animation.dataset.txtoff;
  } 

  if (running) {
    animation.classList.remove('running');
    cmdBtn_ClearColorClasses(animation);
    animation.classList.add('sroBtnGray');
    animation.textContent = animation.dataset.txtoff;
    if (animation.id == "btnSolderingSequence"){
      CTRL_CMD_VALUE.SEQ_RUN = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.run_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (animation.id == "btnHeatingElements"){
      CTRL_CMD_VALUE.HEATER = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.heater_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (animation.id == "btnConvectionFan"){
      CTRL_CMD_VALUE.FAN = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.fan_out, JSON_CTRL_CMD_VALUE, 0, false);
    }

  } 
}


/* handling actual commands */

function CMD_BtnClick(_par) {

  if (_par.id == "PAR_BtnLoad"){
    JSON_SolderingParameterSet = JSON.stringify(SolderingParameter);
    websocketclient.publish(MQTT_PublishTag.pubSolParLoad, JSON_SolderingParameterSet, 0, false);
    DataStream();
  }

  if (_par.id == "PAR_BtnSave"){
    JSON_SolderingParameterSet = JSON.stringify(SolderingParameter);
    websocketclient.publish(MQTT_PublishTag.pubSolParSave, JSON_SolderingParameterSet, 0, false);
    clearInterval(DataStreamInterval);
  }

  if (_par.id == "PID_BtnLoad"){
    JSON_PID_ParameterSet = JSON.stringify(ParPID);
    websocketclient.publish(MQTT_PublishTag.pubPID_ParLoad, JSON_PID_ParameterSet, 0, false);
//    DataStream();
  }

  if (_par.id == "PID_BtnSave"){
    JSON_PID_ParameterSet = JSON.stringify(ParPID);
    console.log(JSON_PID_ParameterSet);
    websocketclient.publish(MQTT_PublishTag.pubPID_ParSave, JSON_PID_ParameterSet, 0, false);
  }

}





