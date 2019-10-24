/*
*
*   data handling script
*
*/

const datTimeMin = 0;
const datTimeMax = 600;
const datTempMin = 25;
const datTempMax = 400;
const datTempUnit = " °C";

var Actual_OPM = "OPM - Automatic";

var ColorPalette = {'SRO_RED': "rgb(255,69,84)",
                    'SRO_BLUE': "rgb(0,195,227)",
                    'SRO_YELLOW': "rgb(255,206,38)",
                    'SRO_GREEN': "rgb(44,180,87)",
                    'SRO_ORANGE': "rgb(255,103,20)",
                    'SRO_OFF_WHITE': "rgb(229,229,229)",
                    'SRO_OFF_GRAY': "rgb(128,128,128)",
                    'SRO_OFF_BLACK': "rgb(38,38,38)"
                   };

var MQTT_SubscriptionTag = {'temperature': "/a2s/temperature",
                            'subSolParLoad': "/a2s/parameter_set_load_response",
                            'subPID_ParLoad': "/a2s/pid_parameter_load_response",
                            'subDOOR_ParLoad': "/a2s/door_parameter_load_response",
                            'run_in': "/a2s/run_from_esp32",
                            'heater_in': "/a2s/heater_from_esp32",
                            'fan_in': "/a2s/fan_from_esp32",
                            'ramp_out': "/a2s/measure_ramp_from_esp32",
                            'pid_out': "/a2s/pid_try_autotune_from_esp32",
                            'door_in': "/a2s/door_from_esp32",
                            'lifebit_in': "/a2s/lifebit_from_esp32",
                            'timestamp': "/a2s/timestamp"
                            };

var MQTT_PublishTag = {'pubSolParSave': "/a2s/parameter_set_save",
                       'pubSolParLoad': "/a2s/parameter_set_load",
                       'pubPID_ParSave': "/a2s/pid_parameter_save",
                       'pubPID_ParLoad': "/a2s/pid_parameter_load",
                       'pubDOOR_ParSave': "/a2s/door_parameter_save",
                       'pubDOOR_ParLoad': "/a2s/door_parameter_load",
                       'run_out': "/a2s/run_to_esp32",
                       'heater_out': "/a2s/heater_to_esp32",
                       'fan_out': "/a2s/fan_to_esp32",
                       'ramp_out': "/a2s/measure_ramp_to_esp32",
                       'pid_out': "/a2s/pid_try_autotune_to_esp32",
                       'door_out': "/a2s/door_to_esp32",
                       'data_update_out': "/a2s/data_update_to_esp32",
                       'lifebit_out': "/a2s/lifebit_to_esp32"
                      };

var JSON_SolderingParameterSet = {};

var SolderingParameter = {'idSRO_ParSetSel': 1,
                          'InputA_min': 0,
                          'InputA_set': 0,
                          'InputA_max': 0,
                          'InputB_min': 0,
                          'InputB_set': 0,
                          'InputB_max': 0,
                          'InputC_min': 0,
                          'InputC_set': 0,
                          'InputC_max': 0,
                          'InputD_min': 0,
                          'InputD_set': 0,
                          'InputD_max': 0,
                          'InputE_min': 0,
                          'InputE_set': 0,
                          'InputE_max': 0,
                          'InputF_min': 0,
                          'InputF_set': 0,
                          'InputF_max': 0,
                          'InputG_min': 0,
                          'InputG_set': 0,
                          'InputG_max': 0,
                          'InputH_min': 0,
                          'InputH_set': 0,
                          'InputH_max': 0,
                          'InputI_min': 0,
                          'InputI_set': 0,
                          'InputI_max': 0};

var ActualTemperature = {'sTemp_C': "0",
                         'sTemp_F': "0",
                         'iTemp_C': 0,
                         'iTemp_F': 0,
                         'bSensor_OK': false}

var JSON_PID_ParameterSet = {};

var ParPID = {'PID_KP': 0.0,
              'PID_KI': 0.0,
              'PID_KD': 0.0,
              'HU_Ramp': 0.0,
              'CD_Ramp': 0.0,
              'TestTemp': 0.0,
              'OvenMaxTemp': datTempMax,
              'HoldTime': 0.0};

var JSON_CTRL_CMD_VALUE = {};

var CTRL_CMD_VALUE = {"SEQ_RUN": 0,
                      "HEATER": 0,
                      "FAN": 0,
                      "RAMP": 0,
                      "PID": 0,
                      "ACT_DOOR_POS": 0,
                      "SET_DOOR_POS": 0};

function onDoorPosInputChange(_par) {
  var tmpDoorInputVal = 0;

  tmpDoorInputVal = parseInt(document.getElementById("InputDoorPos").value);
  if (tmpDoorInputVal < DoorMin) {
    document.getElementById("InputDoorPos").value = DoorMin;
    tmpDoorInputVal = DoorMin;
  }
  if (tmpDoorInputVal > DoorMax) {
    document.getElementById("InputDoorPos").value = DoorMax;
    tmpDoorInputVal = DoorMax;
  }
  if (tmpDoorInputVal >= DoorMin && tmpDoorInputVal <= DoorMax) {
    CTRL_CMD_VALUE.SET_DOOR_POS = tmpDoorInputVal;
    JSON_CTRL_CMD_VALUE = JSON.stringify(CTRL_CMD_VALUE);
    websocketclient.publish(MQTT_PublishTag.door_out, JSON_CTRL_CMD_VALUE, 0, false);

  }
}

// parameter input handling
function PID_ParInput(_par) {
  if (_par.id == "maint_PID_KP")
    ParPID.PID_KP = parseFloat(document.getElementById(_par.id).value);
  if (_par.id == "maint_PID_KI")
    ParPID.PID_KI = parseFloat(document.getElementById(_par.id).value);
  if (_par.id == "maint_PID_KD")
    ParPID.PID_KD = parseFloat(document.getElementById(_par.id).value);
}

function RedrawSetupChart(){
  var tmp_HU_Time;
  var tmp_CD_Time;

  tmp_HU_Time = ParPID.TestTemp / ParPID.HU_Ramp;
  tmp_CD_Time = ParPID.TestTemp / ParPID.CD_Ramp;
  
  RSP_Chart.data.datasets[0].data[0].x = 0;
  RSP_Chart.data.datasets[0].data[1].x = tmp_HU_Time;
  RSP_Chart.data.datasets[0].data[2].x = tmp_HU_Time + ParPID.HoldTime;
  RSP_Chart.data.datasets[0].data[3].x = tmp_HU_Time + ParPID.HoldTime + tmp_CD_Time;
  
  RSP_Chart.data.datasets[0].data[0].y = 25;
  RSP_Chart.data.datasets[0].data[1].y = ParPID.TestTemp;
  RSP_Chart.data.datasets[0].data[2].y = ParPID.TestTemp;
  RSP_Chart.data.datasets[0].data[3].y = 25;
  
// adjusting X-Axis 
  var tmp_Ticks = 0;
  var tmp_ChartTime = 0;
  
  tmp_ChartTime = tmp_HU_Time + ParPID.HoldTime + tmp_CD_Time;

  tmp_Ticks = tmp_ChartTime / 10;
  tmp_Ticks = (tmp_Ticks - Math.floor(tmp_Ticks)) * 10;
  tmp_Ticks = tmp_Ticks + (10 - tmp_Ticks);
  tmp_Ticks = ((Math.floor(tmp_ChartTime / 10)) * 10) + tmp_Ticks;

  if (tmp_Ticks <= 0 || isNaN(tmp_Ticks)){
    tmp_Ticks = datTimeMax;
  }
  
  RSP_Chart.options.scales.xAxes[0].ticks.max = tmp_Ticks;
  RSP_Chart.options.scales.xAxes[1].ticks.max = tmp_Ticks;
  RSP_Chart.options.annotation.annotations[0].value = ParPID.TestTemp;
  RSP_Chart.options.annotation.annotations[0].label.content = "Test Temp: " + ParPID.TestTemp + " °C";
  
  if (ParPID.OvenMaxTemp > 0){
    tmp_Ticks = ParPID.OvenMaxTemp;
  } else {
    tmp_Ticks = datTempMax;
  }
  
  RSP_Chart.options.scales.yAxes[0].ticks.max = tmp_Ticks;
  
  RSP_Chart.update();

}

function SetupOnChange(_par){
// might needed for something later
}

function SRO_ParSetSel(_par) {
  SolderingParameter[_par.id] = document.getElementById(_par.id).selectedIndex + 1;
}

function cmdButtonsDisable(_disable){
  document.getElementById("btnSetup11").disabled = _disable;                // SETUP-TAB PID 'Measure Ramps' button
  document.getElementById("btnSetup12").disabled = _disable;                // SETUP-TAB PID 'Load Data' button
  document.getElementById("btnSetup13").disabled = _disable;                // SETUP-TAB PID 'Save Data' button
  document.getElementById("btnSetup21").disabled = _disable;                // SETUP-TAB PID 'Try Auto Tune' button

  document.getElementById("btnSolderingSequence").disabled = _disable;      // AUTOMATIC-TAB Soldering Sequence 'RUN' button
  document.getElementById("btnHeatingElements").disabled = _disable;        // MANUAL-TAB Heating Element 'ON' button
  document.getElementById("btnConvectionFan").disabled = _disable;          // MANUAL-TAB Convection Fan 'ON' button
  document.getElementById("btnDoorA").disabled = _disable;                  // MANUAL-TAB DOOR 'Close' button
  document.getElementById("btnDoorB").disabled = _disable;                  // MANUAL-TAB DOOR '<' button
  document.getElementById("btnDoorC").disabled = _disable;                  // MANUAL-TAB DOOR '>' button
  document.getElementById("btnDoorD").disabled = _disable;                  // MANUAL-TAB DOOR 'Open' button
}

// soldering parameter input handling
//function SRO_ParInput(_par) {
//  SolderingParameter[_par.id] = parseFloat(document.getElementById(_par.id).value);
//  console.log(SolderingParameter);
//}

// copy soldering profile parameter from input form to memory
function SRO_Para_InputToMem() {
  SolderingParameter.InputA_min = parseFloat(document.getElementById("InputA_min").value);
  SolderingParameter.InputA_set = parseFloat(document.getElementById("InputA_set").value);
  SolderingParameter.InputA_max = parseFloat(document.getElementById("InputA_max").value);
  SolderingParameter.InputB_min = parseFloat(document.getElementById("InputB_min").value);
  SolderingParameter.InputB_set = parseFloat(document.getElementById("InputB_set").value);
  SolderingParameter.InputB_max = parseFloat(document.getElementById("InputB_max").value);
  SolderingParameter.InputC_min = parseFloat(document.getElementById("InputC_min").value);
  SolderingParameter.InputC_set = parseFloat(document.getElementById("InputC_set").value);
  SolderingParameter.InputC_max = parseFloat(document.getElementById("InputC_max").value);
  SolderingParameter.InputD_min = parseFloat(document.getElementById("InputD_min").value);
  SolderingParameter.InputD_set = parseFloat(document.getElementById("InputD_set").value);
  SolderingParameter.InputD_max = parseFloat(document.getElementById("InputD_max").value);
  SolderingParameter.InputE_min = parseFloat(document.getElementById("InputE_min").value);
  SolderingParameter.InputE_set = parseFloat(document.getElementById("InputE_set").value);
  SolderingParameter.InputE_max = parseFloat(document.getElementById("InputE_max").value);
  SolderingParameter.InputF_min = parseFloat(document.getElementById("InputF_min").value);
  SolderingParameter.InputF_set = parseFloat(document.getElementById("InputF_set").value);
  SolderingParameter.InputF_max = parseFloat(document.getElementById("InputF_max").value);
  SolderingParameter.InputG_min = parseFloat(document.getElementById("InputG_min").value);
  SolderingParameter.InputG_set = parseFloat(document.getElementById("InputG_set").value);
  SolderingParameter.InputG_max = parseFloat(document.getElementById("InputG_max").value);
  SolderingParameter.InputH_min = parseFloat(document.getElementById("InputH_min").value);
  SolderingParameter.InputH_set = parseFloat(document.getElementById("InputH_set").value);
  SolderingParameter.InputH_max = parseFloat(document.getElementById("InputH_max").value);
  SolderingParameter.InputI_set = parseFloat(document.getElementById("InputI_set").value);
}

// copy soldering profile parameter from memory to input form
function SRO_Para_MemToInput() {
  document.getElementById("InputA_min").value = SolderingParameter.InputA_min;
  document.getElementById("InputA_set").value = SolderingParameter.InputA_set;
  document.getElementById("InputA_max").value = SolderingParameter.InputA_max;
  document.getElementById("InputB_min").value = SolderingParameter.InputB_min;
  document.getElementById("InputB_set").value = SolderingParameter.InputB_set;
  document.getElementById("InputB_max").value = SolderingParameter.InputB_max;
  document.getElementById("InputC_min").value = SolderingParameter.InputC_min;
  document.getElementById("InputC_set").value = SolderingParameter.InputC_set;
  document.getElementById("InputC_max").value = SolderingParameter.InputC_max;
  document.getElementById("InputD_min").value = SolderingParameter.InputD_min;
  document.getElementById("InputD_set").value = SolderingParameter.InputD_set;
  document.getElementById("InputD_max").value = SolderingParameter.InputD_max;
  document.getElementById("InputE_min").value = SolderingParameter.InputE_min;
  document.getElementById("InputE_set").value = SolderingParameter.InputE_set;
  document.getElementById("InputE_max").value = SolderingParameter.InputE_max;
  document.getElementById("InputF_min").value = SolderingParameter.InputF_min;
  document.getElementById("InputF_set").value = SolderingParameter.InputF_set;
  document.getElementById("InputF_max").value = SolderingParameter.InputF_max;
  document.getElementById("InputG_min").value = SolderingParameter.InputG_min;
  document.getElementById("InputG_set").value = SolderingParameter.InputG_set;
  document.getElementById("InputG_max").value = SolderingParameter.InputG_max;
  document.getElementById("InputH_min").value = SolderingParameter.InputH_min;
  document.getElementById("InputH_set").value = SolderingParameter.InputH_set;
  document.getElementById("InputH_max").value = SolderingParameter.InputH_max;
  document.getElementById("InputI_set").value = SolderingParameter.InputI_set;
}

$(document).ready(function() {
//    do something on document ready
    SRO_Para_MemToInput();
});

