$(document).ready(function() {

});

// parameter input handling
function SRO_ParInput(_par) {
  SolderingParameter[_par.id] = parseInt(document.getElementById(_par.id).value, 10);
  SRO_SetValChart_Update();
//  console.log(SolderingParameter);
}


  //get canvas
var ctx = $("#ReflowChart");
/*
var SRO_SetValues = [{ x: 0, y: 25 },
                     { x: 10, y: 100 },
                     { x: 90, y: 150 },
                     { x: 125, y: 240 },
                     { x: 130, y: 240 },
                     { x: 170, y: 25  }
                    ];
*/
var SRO_SetValues = [{ x: 0, y: datTempMin },
                     { x: SolderingParameter.InputTime01, y: SolderingParameter.InputTemp01 },
                     { x: SolderingParameter.InputTime02, y: SolderingParameter.InputTemp02 },
                     { x: SolderingParameter.InputTime03, y: SolderingParameter.InputTemp03 },
                     { x: SolderingParameter.InputTime04, y: SolderingParameter.InputTemp04 },
                     { x: SolderingParameter.InputTime05, y: SolderingParameter.InputTemp05 }
                    ];

var SRO_ActValues = [{ x: 0, y: 0 }];

//	var ctx = document.getElementById("TestChart").getContext("2d");
var data = {
  datasets : [
    {
      label : "set values",
      data: SRO_SetValues,
      backgroundColor : "blue",
      borderColor : "lightblue",
      fill : false,
      lineTension : 0,
      pointRadius : 5,
      xAxisID: 'x-axis-1',
    }, {
      label : "act values",
      data: SRO_ActValues,
      backgroundColor : "green",
      borderColor : "lightgreen",
      fill : false,
      lineTension : 0,
      pointRadius : 2,
      xAxisID: 'x-axis-2',
    }
  ]
};

var options = {
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
        max: 300
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
      id: 'x-axis-1',
      type: 'linear',
      position: 'bottom',
      display: true,
      scaleLabel: {
        display: true,
        labelString: 'Time in seconds'
      },
      ticks: {
        min: 0,
        max: 200,
        stepSize: 10
      },
    }, {
      id: 'x-axis-2',
      type: 'linear',
      position: 'bottom',
      display: false,
      scaleLabel: {
        display: true,
        labelString: 'Time in seconds'
      },
      ticks: {
        min: 0,
        max: 200,
        stepSize: 10
      }
    }],
    },
  title : {
    display : true,
    position : "top",
    text : "Soldering Reflow Curve",
    fontSize : 18,
    fontColor : "#111"
  },
  legend : {
    display : true,
    position : "bottom"
  }
};

var SRO_Chart = new Chart( ctx, {
  type : "line",
  data : data,
  options : options
});

function SRO_SetValChart_Update(){

  SRO_Chart.data.datasets[0].data[1].x = SolderingParameter.InputTime01;
  SRO_Chart.data.datasets[0].data[2].x = SolderingParameter.InputTime02;
  SRO_Chart.data.datasets[0].data[3].x = SolderingParameter.InputTime03;
  SRO_Chart.data.datasets[0].data[4].x = SolderingParameter.InputTime04;
  SRO_Chart.data.datasets[0].data[5].x = SolderingParameter.InputTime05;
  
  SRO_Chart.data.datasets[0].data[1].y = SolderingParameter.InputTemp01;
  SRO_Chart.data.datasets[0].data[2].y = SolderingParameter.InputTemp02;
  SRO_Chart.data.datasets[0].data[3].y = SolderingParameter.InputTemp03;
  SRO_Chart.data.datasets[0].data[4].y = SolderingParameter.InputTemp04;
  SRO_Chart.data.datasets[0].data[5].y = SolderingParameter.InputTemp05;

  SRO_Chart.update();
}

function SRO_Chart_ClearData(_dataset){
  console.log(_dataset);
  for (var Idx = 0; Idx < _dataset.data.length; Idx++){
    _dataset.data.pop();
  }
  SRO_Chart.update();

}

var DataStreamInterval = null;
var X_Pos = 0;

function DataStream(){
    SRO_Chart.data.datasets[1].data[X_Pos] = { x: X_Pos, y: ActualTemperature.iTemp_C };
//    console.log(SRO_Chart.data.datasets[1].data[X_Pos]);
    if (X_Pos <= 200){
      X_Pos++;
    } else {
      X_Pos = 0;
    }
    SRO_Chart.update();
     
}




function SRO_ActValChart_TestData(){
  var min=25;
  var max=275;

  SRO_Chart_ClearData(SRO_Chart.data.datasets[1]);
  
  SRO_Chart.data.datasets[1].data[0] = { x: 0, y: datTempMin };
  for (var Idx = 1; Idx < 200; Idx++) {
    SRO_Chart.data.datasets[1].data[Idx] = { x: Idx, y: Math.floor(Math.random() * (+max - +min)) + +min };
  }
  SRO_Chart.data.datasets[1].data[200] = { x: 200, y: datTempMin };
  SRO_Chart.update();
}

function SRO_ActValChart_Update(){

  SRO_Chart.data.datasets[1].data[1].x = SolderingParameter.InputTime01;
  SRO_Chart.data.datasets[1].data[2].x = SolderingParameter.InputTime02;
  SRO_Chart.data.datasets[1].data[3].x = SolderingParameter.InputTime03;
  SRO_Chart.data.datasets[1].data[4].x = SolderingParameter.InputTime04;
  SRO_Chart.data.datasets[1].data[5].x = SolderingParameter.InputTime05;
  
  SRO_Chart.data.datasets[1].data[1].y = SolderingParameter.InputTemp01;
  SRO_Chart.data.datasets[1].data[2].y = SolderingParameter.InputTemp02;
  SRO_Chart.data.datasets[1].data[3].y = SolderingParameter.InputTemp03;
  SRO_Chart.data.datasets[1].data[4].y = SolderingParameter.InputTemp04;
  SRO_Chart.data.datasets[1].data[5].y = SolderingParameter.InputTemp05;

  SRO_Chart.update();
}













