/*
*
*   script for vorious tools
*
*/

var _ScanLineTimer = null;
var ScanLinePos = 0;

$('#operating-mode a').on('click', function (e) {
  e.preventDefault()
  $(this).tab('show')
})

$('#operating-mode a').on('shown.bs.tab', function (e) {
  document.getElementById("ActualOPM").innerHTML = e.target.id;
  Actual_OPM = e.target.id;
  
  X_Pos = 0;
//  ScanLinePos = 0;

  if (RSP_Chart.canvas.id == "SetupChart"){
    RSP_Chart_ClearData(RSP_Chart.data.datasets[1]);
    RedrawSetupChart();
  }
  if (RSP_Chart.canvas.id == "ReflowChart" || RSP_Chart.canvas.id == "ManualChart"){
    RSP_Chart_ClearData(RSP_Chart.data.datasets[3]);
    RSP_SetValChart_Update();
  }

  if (Actual_OPM == "OPM - Automatic"){
    RSP_Chart = new Chart( A01_ctx, {
      type : "line",
      data : RSP_Data,
      options : RSP_Options
    });
    RSP_SetValChart_Update();
  }

  if (Actual_OPM == "OPM - Manual"){
    RSP_Chart = new Chart( M01_ctx, {
      type : "line",
      data : RSP_Data,
      options : RSP_Options
    });
    RSP_SetValChart_Update();
  }

  if (Actual_OPM == "OPM - Setup"){
    RSP_Chart = new Chart( S01_ctx, {
      type : "line",
      data : Setup_Data,
      options : Setup_Options
    });
    RedrawSetupChart();
  }

})

$(document).ready(function() {
  RSP_Chart = new Chart( A01_ctx, {
    type : "line",
    data : RSP_Data,
    options : RSP_Options
  });
});

