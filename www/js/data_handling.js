/*
*
*   data handling script
*
*/

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
                            'run_in': "/a2s/run_from_esp32",
                            'heater_in': "/a2s/heater_from_esp32",
                            'fan_in': "/a2s/fan_from_esp32",
                            'door_in': "/a2s/door_from_esp32",
                            'lifebit_in': "/a2s/lifebit_from_esp32",
                            'timestamp': "/a2s/timestamp"
                            };

var MQTT_PublishTag = {'pubSolParSave': "/a2s/parameter_set_save",
                       'pubSolParLoad': "/a2s/parameter_set_load",
                       'pubPID_ParSave': "/a2s/pid_parameter_save",
                       'pubPID_ParLoad': "/a2s/pid_parameter_load",
                       'run_out': "/a2s/run_to_esp32",
                       'heater_out': "/a2s/heater_to_esp32",
                       'fan_out': "/a2s/fan_to_esp32",
                       'door_out': "/a2s/door_to_esp32",
                       'data_update_out': "/a2s/data_update_to_esp32",
                       'lifebit_out': "/a2s/lifebit_to_esp32"
                      };

var JSON_SolderingParameterSet = {};

var SolderingParameter = {'idSRO_ParSetSel': 1,
                          'InputTime01': 0,
                          'InputTime02': 0,
                          'InputTime03': 0,
                          'InputTime04': 0,
                          'InputTime05': 0,
                          'InputTemp01': 0,
                          'InputTemp02': 0,
                          'InputTemp03': 0,
                          'InputTemp04': 0,
                          'InputTemp05': 0};

var ActualTemperature = {'sTemp_C': "0",
                         'sTemp_F': "0",
                         'iTemp_C': 0,
                         'iTemp_F': 0,
                         'bSensor_OK': false}

var JSON_PID_ParameterSet = {};

var ParPID = {"PID_KP": 0.0,
              "PID_KI": 0.0,
              "PID_KD": 0.0};

var JSON_CTRL_CMD_VALUE = {};

var CTRL_CMD_VALUE = {"SEQ_RUN": 0,
                      "HEATER": 0,
                      "FAN": 0,
                      "DOOR": 0};

const datTimeMin = 0;
const datTimeMax = 1200;
const datTempMin = 25;
const datTempMax = 400;
const datTempUnit = " °C";

// parameter input handling
function PID_ParInput(_par) {
  if (_par.id == "maint_PID_KP")
    ParPID.PID_KP = parseFloat(document.getElementById(_par.id).value);
  if (_par.id == "maint_PID_KI")
    ParPID.PID_KI = parseFloat(document.getElementById(_par.id).value);
  if (_par.id == "maint_PID_KD")
    ParPID.PID_KD = parseFloat(document.getElementById(_par.id).value);
}

function SRO_ParSetSel(_par) {
  SolderingParameter[_par.id] = document.getElementById(_par.id).selectedIndex + 1;
}

function cmdButtonsDisable(_disable){
  document.getElementById("PAR_BtnLoad").disabled = _disable;
  document.getElementById("PAR_BtnSave").disabled = _disable;
  document.getElementById("PID_BtnLoad").disabled = _disable;
  document.getElementById("PID_BtnSave").disabled = _disable;

  document.getElementById("btnSolderingSequence").disabled = _disable;
  document.getElementById("btnHeatingElements").disabled = _disable;
  document.getElementById("btnConvectionFan").disabled = _disable;
  document.getElementById("btnDoor").disabled = _disable;
}

//  $(document).ready(function() {
//    MQTTconnect();
//  });

