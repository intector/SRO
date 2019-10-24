/*
*
*   cmdButton handling script
*
*/

var ANI_btnSolderingSequence = document.getElementById("btnSolderingSequence");
var ANI_btnHeatingElements = document.getElementById("btnHeatingElements");
var ANI_btnConvectionFan = document.getElementById("btnConvectionFan");
var ANI_btnSetup11 = document.getElementById("btnSetup11");
var ANI_btnSetup21 = document.getElementById("btnSetup21");

ANI_btnSolderingSequence.addEventListener("animationend", AnimationEndListener, false);
ANI_btnHeatingElements.addEventListener("animationend", AnimationEndListener, false);
ANI_btnConvectionFan.addEventListener("animationend", AnimationEndListener, false);
ANI_btnSetup11.addEventListener("animationend", AnimationEndListener, false);
ANI_btnSetup21.addEventListener("animationend", AnimationEndListener, false);

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
    if (this.id == "btnSetup11"){
      CTRL_CMD_VALUE.RAMP = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.ramp_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (this.id == "btnSetup21"){
      CTRL_CMD_VALUE.PID = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.pid_out, JSON_CTRL_CMD_VALUE, 0, false);
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
//  cmdButtonsDisable(true);
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
    if (animation.id == "btnSetup11"){
      CTRL_CMD_VALUE.RAMP = 1;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.ramp_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (animation.id == "btnSetup21"){
      CTRL_CMD_VALUE.PID = 1;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.pid_out, JSON_CTRL_CMD_VALUE, 0, false);
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
    if (animation.id == "btnSetup11"){
      CTRL_CMD_VALUE.RAMP = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.ramp_out, JSON_CTRL_CMD_VALUE, 0, false);
    }
    if (animation.id == "btnSetup21"){
      CTRL_CMD_VALUE.PID = 0;
      JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
      websocketclient.publish(MQTT_PublishTag.pid_out, JSON_CTRL_CMD_VALUE, 0, false);
    }

  } 
}


/* handling actual commands */

function CMD_BtnClick(_par) {

  if (_par.id == "PAR_BtnLoad"){
    JSON_SolderingParameterSet = JSON.stringify(SolderingParameter);
    websocketclient.publish(MQTT_PublishTag.pubSolParLoad, JSON_SolderingParameterSet, 0, false);
    SRO_Para_MemToInput();
    RSP_Chart.destroy();
//    DataStream();
  }

  if (_par.id == "PAR_BtnSave"){

    SRO_Para_InputToMem();
    JSON_SolderingParameterSet = JSON.stringify(SolderingParameter);
    websocketclient.publish(MQTT_PublishTag.pubSolParSave, JSON_SolderingParameterSet, 0, false);

    RSP_Chart = new Chart( A01_ctx, {
      type : "line",
      data : RSP_Data,
      options : RSP_Options
    });
    
//    clearInterval(DataStreamInterval);
    RSP_SetValChart_Update();
  }
}

// move door motor with buttons

const DoorMin = 0;
const DoorMax = 100;

function onDoorBtnClickEvent(_par) {
//  close door complete
  if (_par.id == "btnDoorA"){
    CTRL_CMD_VALUE.SET_DOOR_POS = DoorMin;
    JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
    websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
    document.getElementById("InputDoorPos").value = CTRL_CMD_VALUE.SET_DOOR_POS;
  }
//  close door one step of 10%
  if (_par.id == "btnDoorB"){
    if (CTRL_CMD_VALUE.SET_DOOR_POS >= DoorMin + 10){
      CTRL_CMD_VALUE.SET_DOOR_POS -= 10;
    } else {
      CTRL_CMD_VALUE.SET_DOOR_POS = DoorMin;
    }
    JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
    websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
    document.getElementById("InputDoorPos").value = CTRL_CMD_VALUE.SET_DOOR_POS;
  }
//  open door one step of 10%
  if (_par.id == "btnDoorC"){
    if (CTRL_CMD_VALUE.SET_DOOR_POS <= DoorMax - 10){
      CTRL_CMD_VALUE.SET_DOOR_POS += 10;
    } else {
      CTRL_CMD_VALUE.SET_DOOR_POS = DoorMax;
    }
    JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
    websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
    document.getElementById("InputDoorPos").value = CTRL_CMD_VALUE.SET_DOOR_POS;
  }
//  open door complete
  if (_par.id == "btnDoorD"){
    CTRL_CMD_VALUE.SET_DOOR_POS = DoorMax;
    JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
    websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);
    document.getElementById("InputDoorPos").value = CTRL_CMD_VALUE.SET_DOOR_POS;
  }
//  console.log("1 CTRL_CMD_VALUE.DOOR = ", CTRL_CMD_VALUE.DOOR);
}

function onClickSetupBtn(_par) {
// load PID parameter
  if (_par.id == "btnSetup12"){
    JSON_PID_ParameterSet = JSON.stringify(ParPID);
    websocketclient.publish(MQTT_PublishTag.pubPID_ParLoad, JSON_PID_ParameterSet, 0, false);
  }
// save PID parameter
  if (_par.id == "btnSetup13"){
    ParPID.PID_KP = parseFloat(document.getElementById("SI_PID01").value);
    ParPID.PID_KI = parseFloat(document.getElementById("SI_PID02").value);
    ParPID.PID_KD = parseFloat(document.getElementById("SI_PID03").value);
    ParPID.HU_Ramp = parseInt(document.getElementById("SI_HS01").value);
    ParPID.CD_Ramp = parseInt(document.getElementById("SI_HS02").value);
    ParPID.TestTemp = parseInt(document.getElementById("SI_HS03").value);
    ParPID.HoldTime = parseInt(document.getElementById("SI_HS04").value);
    ParPID.OvenMaxTemp = parseInt(document.getElementById("SI_HS05").value);
    RedrawSetupChart();
    JSON_PID_ParameterSet = JSON.stringify(ParPID);
    websocketclient.publish(MQTT_PublishTag.pubPID_ParSave, JSON_PID_ParameterSet, 0, false);
  }

}







