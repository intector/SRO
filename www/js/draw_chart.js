/*
*
*   draw chart script
*
*/

$(document).ready(function() {
//  do something on document ready
//  _ScanLineTimer = setInterval(function(){

//    if (ScanLinePos < RSP_Chart.options.scales.xAxes[0].ticks.max) {
//      ScanLinePos ++;
//    } else {
//      ScanLinePos = 0;
//    }
//    RSP_Chart.options.annotation.annotations[3].value = ScanLinePos;
//    RSP_Chart.update();

//  }, 1000);

});

const RSP_Chart_Colors = {
  Min_Values: {
    Fill: 'rgba(255, 174, 149, 0.2)',
    Stroke: 'rgba(255, 174, 149, 0.5)',
  },
  Max_Values: {
    Fill: 'rgba(128, 128, 128, 0.1)',
    Stroke: 'rgba(255, 157, 168, 0.5)',
  },
  Set_Values: {
    Fill: 'rgba(0, 168, 219, 0.2)',
    Stroke: 'rgba(0, 168, 219, 0.5)',
  },
  AutoAct_Values: {
    Fill: 'rgba(64, 64, 64, 0.2)',
    Stroke: 'rgba(64, 64, 64, 0.8)',
  },
  ManAct_Values: {
    Fill: 'rgba(64, 64, 64, 0.2)',
    Stroke: 'rgba(64, 64, 64, 0.8)',
  },
};

var RSP_Chart = 0;

  //get canvas
var A01_ctx = $("#ReflowChart");
var M01_ctx = $("#ManualChart");
var S01_ctx = $("#SetupChart");

/*
 *
// chart for Reflow Soldering Profile
 *
 */

var RSP_MinValues = [{ x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                    ];
var RSP_MaxValues = [{ x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                    ];
var RSP_SetValues = [{ x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                     { x: 0, y: 25 },
                    ];
var RSP_ActValues = [{ x: 0, y: 25 }];

var Setup_Temperature = [{ x: 0, y: 0 },
                         { x: 0, y: 0 },
                         { x: 0, y: 0 },
                         { x: 0, y: 0 }
                        ];

var RSP_HL_B = 0.0;
var RSP_HL_C = 0.0;
var RSP_HL_F = 0.0;
var RSP_HL_I = 0.0;
var RSP_VL_A = 0.0;

var RSP_Data = {
  datasets : [
    {
      label : "min tolerance",
      data: RSP_MinValues,
      backgroundColor : RSP_Chart_Colors.Min_Values.Fill,
      borderColor : RSP_Chart_Colors.Min_Values.Stroke,
      pointBackgroundColor: RSP_Chart_Colors.Min_Values.Stroke,
      pointHighlightStroke: RSP_Chart_Colors.Min_Values.Stroke,
      fill : false,
      lineTension : 0,
      pointRadius : 5,
      xAxisID: 'x-axis-Parameter',
    }, {
      label : "max tolerance",
      data: RSP_MaxValues,
      backgroundColor : RSP_Chart_Colors.Max_Values.Fill,
      borderColor : RSP_Chart_Colors.Max_Values.Stroke,
      pointBackgroundColor: RSP_Chart_Colors.Max_Values.Stroke,
      pointHighlightStroke: RSP_Chart_Colors.Max_Values.Stroke,
      fill : 0,
      lineTension : 0,
      pointRadius : 5,
      xAxisID: 'x-axis-Parameter',
    }, {
      label : "set value",
      data: RSP_SetValues,
      backgroundColor : RSP_Chart_Colors.Set_Values.Fill,
      borderColor : RSP_Chart_Colors.Set_Values.Stroke,
      pointBackgroundColor: RSP_Chart_Colors.Set_Values.Stroke,
      pointHighlightStroke: RSP_Chart_Colors.Set_Values.Stroke,
      fill : false,
      lineTension : 0,
      pointRadius : 5,
      xAxisID: 'x-axis-Parameter',
    }, {
      label : "act values",
      data: RSP_ActValues,
      backgroundColor : RSP_Chart_Colors.AutoAct_Values.Fill,
      borderColor : RSP_Chart_Colors.AutoAct_Values.Stroke,
      pointBackgroundColor: RSP_Chart_Colors.AutoAct_Values.Stroke,
      pointHighlightStroke: RSP_Chart_Colors.AutoAct_Values.Stroke,
      fill : false,
      lineTension : 0,
      pointRadius : 2,
      xAxisID: 'x-axis-ActualValues',
    }
  ]
};

var RSP_Options = {
  scales: {
    yAxes: [{
      id: 'y-axis-1',
      type: 'linear',
      position: 'left',
      display: true,
      scaleLabel: {
        display: true,
        labelString: 'Temperature in °C'
      },
      ticks: {
        min: 25,
        max: ParPID.OvenMaxTemp,
      }
    }, {
      id: 'y-axis-2',
      type: 'category',
      labels: ['TP', 'Ts End', 'Ts Start', ''],
      display: false,
      position: 'right',
      scaleLabel: {
        display: true,
        labelString: 'Request State'
      },
      ticks: {
        reverse: true
      }
    }],
      
    xAxes: [{
      id: 'x-axis-Parameter',
      type: 'linear',
      position: 'bottom',
      display: true,
      scaleLabel: {
        display: true,
        labelString: 'Time in seconds'
      },
      ticks: {
        min: 0,
        max: datTimeMax,
        stepSize: 10
      },
    }, {
      id: 'x-axis-ActualValues',
      type: 'linear',
      position: 'bottom',
      display: false,
      scaleLabel: {
        display: true,
        labelString: 'Time in seconds'
      },
      ticks: {
        min: 0,
        max: datTimeMax,
        stepSize: 10
      }
    }],
  },
  annotation: {
//    events: ["click"],
    annotations: [
      {
        drawTime: "afterDatasetsDraw",
        type: "line",
        mode: "horizontal",
        scaleID: "y-axis-1",
        value: RSP_HL_B,
        borderColor: "black",
        borderWidth: 1,
        borderDash: [2, 2],
        label: {
          position: "left",
          backgroundColor: 'rgba(128,128,128,0.8)',
          fontColor: "#000",
          content: "B",
          enabled: true
        }
      }, {
        drawTime: "afterDatasetsDraw",
        type: "line",
        mode: "horizontal",
        scaleID: "y-axis-1",
        value: RSP_HL_C,
        borderColor: "black",
        borderWidth: 1,
        borderDash: [2, 2],
        label: {
          position: "left",
          backgroundColor: 'rgba(128,128,128,0.8)',
          fontColor: "#000",
          content: "C",
          enabled: true
        }
      }, {
        drawTime: "afterDatasetsDraw",
        type: "line",
        mode: "horizontal",
        scaleID: "y-axis-1",
        value: RSP_HL_F,
        borderColor: "black",
        borderWidth: 1,
        borderDash: [2, 2],
        label: {
          position: "left",
          backgroundColor: 'rgba(128,128,128,0.8)',
          fontColor: "#000",
          content: "F",
          enabled: true
        }
      }, {
        drawTime: "afterDatasetsDraw",
        type: "line",
        mode: "vertical",
        scaleID: "x-axis-ActualValues",
        value: RSP_VL_A,
        borderColor: "gray",
        borderWidth: 1,
        borderDash: [3, 5],
        label: {
          position: "top",
          backgroundColor: 'rgba(128,128,128,0.8)',
          fontColor: "#000",
          content: "actual position",
          enabled: false
        }
      },
    ]},
  title : {
    display : true,
    position : "top",
    text : "Soldering Reflow Profile",
    fontSize : 18,
    fontColor : "#111"
  },
  tooltips: {
    enabled : false
  },
  legend : {
    display : true,
    position : "bottom"
  }
};

var Setup_Data = {
  datasets : [
    {
      label : "set temperature",
      data: Setup_Temperature,
      backgroundColor : RSP_Chart_Colors.Min_Values.Fill,
      borderColor : RSP_Chart_Colors.Min_Values.Stroke,
      pointBackgroundColor: RSP_Chart_Colors.Min_Values.Stroke,
      pointHighlightStroke: RSP_Chart_Colors.Min_Values.Stroke,
      fill : false,
      lineTension : 0,
      pointRadius : 5,
      xAxisID: 'x-axis-Parameter',
    }, {
      label : "act values",
      data: RSP_ActValues,
      backgroundColor : RSP_Chart_Colors.AutoAct_Values.Fill,
      borderColor : RSP_Chart_Colors.AutoAct_Values.Stroke,
      pointBackgroundColor: RSP_Chart_Colors.AutoAct_Values.Stroke,
      pointHighlightStroke: RSP_Chart_Colors.AutoAct_Values.Stroke,
      fill : false,
      lineTension : 0,
      pointRadius : 2,
      xAxisID: 'x-axis-ActualValues',
    }
  ]
};

var Setup_Options = {
  scales: {
    yAxes: [{
      id: 'y-axis-1',
      type: 'linear',
      position: 'left',
      display: true,
      scaleLabel: {
        display: true,
        labelString: 'Temperature in °C'
      },
      ticks: {
        min: 25,
        max: ParPID.OvenMaxTemp,
      }
    }, {
      id: 'y-axis-2',
      type: 'category',
      labels: ['TP', 'Ts End', 'Ts Start', ''],
      display: false,
      position: 'right',
      scaleLabel: {
        display: true,
        labelString: 'Request State'
      },
      ticks: {
        reverse: true
      }
    }],
      
    xAxes: [{
      id: 'x-axis-Parameter',
      type: 'linear',
      position: 'bottom',
      display: true,
      scaleLabel: {
        display: true,
        labelString: 'Time in seconds'
      },
      ticks: {
        min: 0,
        max: datTimeMax,
        stepSize: 10
      },
    }, {
      id: 'x-axis-ActualValues',
      type: 'linear',
      position: 'bottom',
      display: false,
      scaleLabel: {
        display: true,
        labelString: 'Time in seconds'
      },
      ticks: {
        min: 0,
        max: datTimeMax,
        stepSize: 10
      }
    }],
  },
  annotation: {
//    events: ["click"],
    annotations: [
      {
        drawTime: "afterDatasetsDraw",
        type: "line",
        mode: "horizontal",
        scaleID: "y-axis-1",
        value: -10,
        borderColor: 'rgba(255, 0, 0, 0.8)',
        borderWidth: 1,
        borderDash: [4, 4],
        label: {
          position: "left",
          backgroundColor: 'rgba(255, 0, 0, 0.5)',
          fontColor: "#000",
          content: "Test Temp",
          enabled: true
        }
      },
    ]},
  title : {
    display : true,
    position : "top",
    text : "Setup Chart",
    fontSize : 18,
    fontColor : "#111"
  },
  tooltips: {
    enabled : false
  },
  legend : {
    display : true,
    position : "bottom"
  }
};

function RSP_SetValChart_Update(){

  var tmp_WarmupTime = 0;
  var tmp_PreHeatTime = 0;
  var tmp_MeltRampTime = 0;
  
  // minimum values graph
  tmp_WarmupTime = SolderingParameter.InputB_min / SolderingParameter.InputA_min;
  tmp_PreHeatTime = tmp_WarmupTime + (SolderingParameter.InputC_min - SolderingParameter.InputB_min) / SolderingParameter.InputD_min;
  tmp_MeltRampTime = tmp_PreHeatTime + (SolderingParameter.InputF_min - SolderingParameter.InputC_min) / SolderingParameter.InputE_min
  
  RSP_Chart.data.datasets[0].data[1].x = tmp_WarmupTime;
  RSP_Chart.data.datasets[0].data[2].x = tmp_PreHeatTime;
  RSP_Chart.data.datasets[0].data[3].x = tmp_MeltRampTime;
  RSP_Chart.data.datasets[0].data[4].x = tmp_MeltRampTime + SolderingParameter.InputG_min;
  RSP_Chart.data.datasets[0].data[5].x = tmp_MeltRampTime + SolderingParameter.InputG_min + SolderingParameter.InputH_min;

  RSP_Chart.data.datasets[0].data[1].y = SolderingParameter.InputB_min;
  RSP_Chart.data.datasets[0].data[2].y = SolderingParameter.InputC_min;
  RSP_Chart.data.datasets[0].data[3].y = SolderingParameter.InputF_min;
  RSP_Chart.data.datasets[0].data[4].y = SolderingParameter.InputF_min;
  RSP_Chart.data.datasets[0].data[5].y = RSP_MinValues[0].y;
  
  RSP_Chart.options.annotation.annotations[0].value = SolderingParameter.InputB_set;
  RSP_Chart.options.annotation.annotations[1].value = SolderingParameter.InputC_set;
  RSP_Chart.options.annotation.annotations[2].value = SolderingParameter.InputF_set;

  // maximum values graph
  tmp_WarmupTime = SolderingParameter.InputB_max / SolderingParameter.InputA_max;
  tmp_PreHeatTime = tmp_WarmupTime + (SolderingParameter.InputC_max - SolderingParameter.InputB_max) / SolderingParameter.InputD_max;
  tmp_MeltRampTime = tmp_PreHeatTime + (SolderingParameter.InputF_max - SolderingParameter.InputC_max) / SolderingParameter.InputE_max
  
  RSP_Chart.data.datasets[1].data[1].x = tmp_WarmupTime;
  RSP_Chart.data.datasets[1].data[2].x = tmp_PreHeatTime;
  RSP_Chart.data.datasets[1].data[3].x = tmp_MeltRampTime;
  RSP_Chart.data.datasets[1].data[4].x = tmp_MeltRampTime + SolderingParameter.InputG_max;
  RSP_Chart.data.datasets[1].data[5].x = tmp_MeltRampTime + SolderingParameter.InputG_max + SolderingParameter.InputH_max;

  RSP_Chart.data.datasets[1].data[1].y = SolderingParameter.InputB_max;
  RSP_Chart.data.datasets[1].data[2].y = SolderingParameter.InputC_max;
  RSP_Chart.data.datasets[1].data[3].y = SolderingParameter.InputF_max;
  RSP_Chart.data.datasets[1].data[4].y = SolderingParameter.InputF_max;
  RSP_Chart.data.datasets[1].data[5].y = RSP_MaxValues[0].y;
  
  // set values graph
  tmp_WarmupTime = SolderingParameter.InputB_set / SolderingParameter.InputA_set;
  tmp_PreHeatTime = tmp_WarmupTime + (SolderingParameter.InputC_set - SolderingParameter.InputB_set) / SolderingParameter.InputD_set;
  tmp_MeltRampTime = tmp_PreHeatTime + (SolderingParameter.InputF_set - SolderingParameter.InputC_set) / SolderingParameter.InputE_set
  
  RSP_Chart.data.datasets[2].data[1].x = tmp_WarmupTime;
  RSP_Chart.data.datasets[2].data[2].x = tmp_PreHeatTime;
  RSP_Chart.data.datasets[2].data[3].x = tmp_MeltRampTime;
  RSP_Chart.data.datasets[2].data[4].x = tmp_MeltRampTime + SolderingParameter.InputG_set;
  RSP_Chart.data.datasets[2].data[5].x = tmp_MeltRampTime + SolderingParameter.InputG_set + SolderingParameter.InputH_set;

  RSP_Chart.data.datasets[2].data[1].y = SolderingParameter.InputB_set;
  RSP_Chart.data.datasets[2].data[2].y = SolderingParameter.InputC_set;
  RSP_Chart.data.datasets[2].data[3].y = SolderingParameter.InputF_set;
  RSP_Chart.data.datasets[2].data[4].y = SolderingParameter.InputF_set;
  RSP_Chart.data.datasets[2].data[5].y = RSP_SetValues[0].y;

// adjusting X-Axis 
  var tmp_Ticks = 0;
  var tmp_Ticks0 = 0;
  var tmp_Ticks1 = 0;
  var tmp_Ticks2 = 0;
  
  tmp_Ticks0 = RSP_Chart.data.datasets[0].data[5].x / 10;
  tmp_Ticks0 = (tmp_Ticks0 - Math.floor(tmp_Ticks0)) * 10;
  tmp_Ticks0 = tmp_Ticks0 + (10 - tmp_Ticks0);
  tmp_Ticks0 = ((Math.floor(RSP_Chart.data.datasets[0].data[5].x / 10)) * 10) + tmp_Ticks0;

  tmp_Ticks1 = RSP_Chart.data.datasets[1].data[5].x / 10;
  tmp_Ticks1 = (tmp_Ticks1 - Math.floor(tmp_Ticks1)) * 10;
  tmp_Ticks1 = tmp_Ticks1 + (10 - tmp_Ticks1);
  tmp_Ticks1 = ((Math.floor(RSP_Chart.data.datasets[1].data[5].x / 10)) * 10) + tmp_Ticks1;

  tmp_Ticks2 = RSP_Chart.data.datasets[2].data[5].x / 10;
  tmp_Ticks2 = (tmp_Ticks2 - Math.floor(tmp_Ticks2)) * 10;
  tmp_Ticks2 = tmp_Ticks2 + (10 - tmp_Ticks2);
  tmp_Ticks2 = ((Math.floor(RSP_Chart.data.datasets[2].data[5].x / 10)) * 10) + tmp_Ticks2;
  
  tmp_Ticks = datTimeMax;

  if (tmp_Ticks0 > 0 && !isNaN(tmp_Ticks0)){
    tmp_Ticks = tmp_Ticks0;
  }

  if (tmp_Ticks2 > 0 && !isNaN(tmp_Ticks2)){
    tmp_Ticks = tmp_Ticks2;
  }

  if (tmp_Ticks1 > 0 && !isNaN(tmp_Ticks1)){
    tmp_Ticks = tmp_Ticks1;
  }

  RSP_Chart.options.scales.xAxes[0].ticks.max = tmp_Ticks;
  RSP_Chart.options.scales.xAxes[1].ticks.max = tmp_Ticks;

// drawing horizontal lines  
  RSP_Chart.options.annotation.annotations[0].value = SolderingParameter.InputB_set;
  RSP_Chart.options.annotation.annotations[1].value = SolderingParameter.InputC_set;
  RSP_Chart.options.annotation.annotations[2].value = SolderingParameter.InputF_set;
  
  if (ParPID.OvenMaxTemp > 0){
    tmp_Ticks = ParPID.OvenMaxTemp;
  } else {
    tmp_Ticks = datTempMax;
  }
  
  RSP_Chart.options.scales.yAxes[0].ticks.max = tmp_Ticks;

  RSP_Chart.update();
  
//  console.log(SolderingParameter);
}

function RSP_ActValChart_Update(){

//  RSP_Chart.data.datasets[1].data[1].x = SolderingParameter.InputTime01;
//  RSP_Chart.data.datasets[1].data[2].x = SolderingParameter.InputTime02;
//  RSP_Chart.data.datasets[1].data[3].x = SolderingParameter.InputTime03;
//  RSP_Chart.data.datasets[1].data[4].x = SolderingParameter.InputTime04;
//  RSP_Chart.data.datasets[1].data[5].x = SolderingParameter.InputTime05;
  
//  RSP_Chart.data.datasets[1].data[1].y = SolderingParameter.InputTemp01;
//  RSP_Chart.data.datasets[1].data[2].y = SolderingParameter.InputTemp02;
//  RSP_Chart.data.datasets[1].data[3].y = SolderingParameter.InputTemp03;
//  RSP_Chart.data.datasets[1].data[4].y = SolderingParameter.InputTemp04;
//  RSP_Chart.data.datasets[1].data[5].y = SolderingParameter.InputTemp05;

//  RSP_Chart.update();
}

function RSP_Chart_ClearData(_dataset){
  var tmp_ArrayLength = _dataset.data.length;
  for (var Idx = 0; Idx < tmp_ArrayLength; Idx++){
    _dataset.data.pop();
  }
  RSP_Chart.update();

}

var DataStreamInterval = null;
var X_Pos = 0;

function DataStream(){
//  if (ScanLinePos <= RSP_Chart.options.scales.xAxes[0].ticks.max) {
//    ScanLinePos ++;
//  } else {
//    ScanLinePos = 0;
//  }
  RSP_Chart.options.annotation.annotations[3].value = X_Pos;
  RSP_Chart.data.datasets[3].data[X_Pos] = { x: X_Pos, y: ActualTemperature.iTemp_C };

  if (X_Pos <= RSP_Chart.options.scales.xAxes[0].ticks.max){
    X_Pos++;
  } else {
    X_Pos = 0;
  }

  RSP_Chart.update();

}



function RSP_ActValChart_TestData(){
  var min=25;
  var max=275;

  RSP_Chart(RSP_Chart.data.datasets[1]);
  
  RSP_Chart.data.datasets[1].data[0] = { x: 0, y: datTempMin };
  for (var Idx = 1; Idx < 200; Idx++) {
    RSP_Chart.data.datasets[1].data[Idx] = { x: Idx, y: Math.floor(Math.random() * (+max - +min)) + +min };
  }
  RSP_Chart.data.datasets[1].data[200] = { x: 200, y: datTempMin };
  RSP_Chart.update();
}














