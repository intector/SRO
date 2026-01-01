/**
 * SRO - Soldering Reflow Oven
 * Copyright (c) 2025-2030 Intector
 *
 * SRO class ChartsManager (Chart.js-4.5.0)
 *
 */

// -----------------------------------------------------------------------------
// class SRO_Chart
// -----------------------------------------------------------------------------

class SRO_Chart {
    isInitialized = false;
    ReflowChart = null;
    DataStreamInterval = null;
    X_Pos = 0;

    ChartColors = {
        Title:{
            TitleColor: "#FFF",
            SubtitleColor: "#CFD8DC",
        },
        X_Axis: {
            Fill: "#9E9E9EAA",
            Stroke: "#9E9E9EFF",
            FontColor: "#757575FF",
        },
        Y_Axis: {
            Fill: "#9E9E9EAA",
            Stroke: "#9E9E9EFF",
            FontColor: "#757575FF",
        },
        Grid: {
            Fill: "#424242AA",
            Stroke: "#424242FF",
        },
        SoakZone: {
            Fill: "#AA00FF3F",
            Stroke: "#AA00FFFF",
        },
        ReflowZone: {
            Fill: "#FFAB00AA",
            Stroke: "#FFAB00FF",
        },
        SetValues: {
            Fill: "#9E9E9EAA",
            Stroke: "#9E9E9EFF",
        },
        AutoActValues: {
            Fill: "#00C853AA",
            Stroke: "#00C853FF"
        },
        SolderMeltingLine: {
            Fill: "#1A237EFF",
            Stroke: "#304FFEFF",
            TextColor: "#82B1FFFF"
        },
        PeakTempLine: {
            Fill: "#BF360CAA",
            Stroke: "#FFAB00FF",
            TextColor: "#FFD600FF"
        },
        OverTempBox: {
            Fill: "#B71C1CAA",
            Stroke: "#FFD600FF",
            TextColor: "#FFD600FF"
        },
        SoakTempBox: {
            Fill: "#006064AA",
            Stroke: "#00B8D4FF",
            TextColor: "#00B8D4FF"
        },
        ReflowTempBox: {
            Fill: "#1B5E20AA",
            Stroke: "#00C853FF",
            TextColor: "#00C853FF"
        }
    };

    ChartTitleText = "title text";
    ChartSubtitleText = "subtitle text";

    ChartMinTemperature = 25;
    ChartMaxTemperature = 400;

    ChartMinTicks = 0;
    ChartMaxTicks = 600;
    ChartStepSize = 10;

    solderMeltingPoint = 250;
    setTemperature = 200;
    PeakTemperature = 350;

    SetValues = [];
    ActValues = [];

    ChartData = { datasets: [] };
    ChartOptions = {};

    constructor() {
        this.ChartOptions = {
            // chart options...
            responsive: true,
            maintainAspectRatio: false,
            font: {
                family: "'Ubuntu Condensed', sans-serif",
                weight: "normal"
            },
            scales: {
                y: {
                    type: 'linear',
                    alignToPixels: true,
                    position: 'left',
                    display: true,
                    title: {
                        display: true,
                        text: 'Temperature in °C',
                        color: this.ChartColors.Y_Axis.FontColor,
                        font: {
                            family: "'Ubuntu Condensed', sans-serif",
                            size: 14,
                            weight: "normal",
                        }
                    },
                    ticks: {
                        color: this.ChartColors.Y_Axis.Stroke,
                        font: {
                            family: "'Ubuntu Mono', monospace",
                            size: 12,
                            weight: "normal",
                        }
                    },
                    grid: {
                        color: this.ChartColors.Grid.Stroke
                    },
                    min: this.ChartMinTemperature,
                    max: this.ChartMaxTemperature
                },
                x: {
                    type: 'linear',
                    alignToPixels: true,
                    position: 'bottom',
                    display: true,
                    title: {
                        display: true,
                        text: 'Time in seconds',
                        color: this.ChartColors.X_Axis.FontColor,
                        font: {
                            family: "'Ubuntu Condensed', sans-serif",
                            size: 14,
                            weight: "normal"
                        }
                    },
                    ticks: {
                        stepSize: this.ChartStepSize,
                        color: this.ChartColors.X_Axis.Stroke,
                        font: {
                            family: "'Ubuntu Mono', monospace",
                            size: 12,
                            weight: "normal"
                        }
                    },
                    grid: {
                        color: this.ChartColors.Grid.Stroke
                    },
                    min: this.ChartMinTicks,
                    max: this.ChartMaxTicks
                }
            },
            plugins: {
                annotation: {
                    annotations: {
                        solderMeltingLine: {
                            display: false,
                            type: "line",
                            drawTime: "afterDraw",
                            yMin: this.SolderMeltingPoint,
                            yMax: this.SolderMeltingPoint,
                            borderColor: this.ChartColors.SolderMeltingLine.Stroke,
                            borderWidth: 2,
                            borderDash: [10, 5],
                            label: {
                                position: "start",
                                backgroundColor: this.ChartColors.SolderMeltingLine.Fill,
                                color: this.ChartColors.SolderMeltingLine.TextColor,
                                content: "solder melting point",
                                font: {
                                    family: "'Ubuntu Condensed', sans-serif",
                                    size: 12,
                                    weight: "normal"
                                },
                                borderRadius: {
                                    topLeft: 10,
                                    topRight: 10,
                                },
                                yAdjust: -14,
                                display: true,
                            },
                        },
                        setTemperatureLine: {
                            display: false,
                            type: "line",
                            drawingOrder: "afterDatasetsDraw",
                            yMin: this.setTemperature,
                            yMax: this.setTemperature,
                            borderColor: this.ChartColors.SolderMeltingLine.Stroke,
                            borderWidth: 2,
                            borderDash: [5, 10],
                            label: {
                                position: "start",
                                backgroundColor: this.ChartColors.SolderMeltingLine.Fill,
                                color: this.ChartColors.SolderMeltingLine.TextColor,
                                content: "Set-Temp",
                                font: {
                                    family: "'Ubuntu Condensed', sans-serif",
                                    size: 12,
                                    weight: "normal"
                                },
                                borderRadius: {
                                    topLeft: 10,
                                    topRight: 10,
                                },
                                yAdjust: -14,
                                display: true,
                            },
                        },
                        peakTemperatureLine: {
                            display: false,
                            type: "line",
                            drawTime: "afterDraw",
                            yMin: this.PeakTemperature,
                            yMax: this.PeakTemperature,
                            borderColor: this.ChartColors.PeakTempLine.Stroke,
                            borderWidth: 2,
                            label: {
                                position: "end",
                                backgroundColor: this.ChartColors.PeakTempLine.Fill,
                                color: this.ChartColors.PeakTempLine.TextColor,
                                content: "peak temperature",
                                font: {
                                    family: "'Ubuntu Condensed', sans-serif",
                                    size: 12,
                                    weight: "normal"
                                },
                                borderRadius: {
                                    bottomLeft: 10,
                                    bottomRight: 10,
                                },
                                yAdjust: 14,
                                display: true,
                            },
                        },
                        overTemperatureBox: {
                            display: false,
                            type: "box",
                            drawTime: "beforeDraw",
                            xMin: this.ChartMinTicks + 1,
                            yMin: this.PeakTemperature,
                            xMax: this.ChartMaxTicks,
                            yMax: this.ChartMaxTemperature - 1,
                            borderColor: this.ChartColors.OverTempBox.Stroke,
                            backgroundColor: this.ChartColors.OverTempBox.Fill,
                            borderWidth: 1,
                            borderDash: [2, 2],
                            label: {
                                position: "center",
                                backgroundColor: this.ChartColors.OverTempBox.Fill,
                                color: this.ChartColors.OverTempBox.TextColor,
                                content: "!!!   OVER TEMPERATURE ZONE   !!!",
                                font: {
                                    family: "'Ubuntu Condensed', sans-serif",
                                    size: 20,
                                    weight: "normal"
                                },
                                display: true
                            }
                        },
                        soakTemperatureBox: {
                            display: false,
                            type: "box",
                            drawTime: "beforeDraw",
                            xMin: 0,
                            xMax: 0,
                            yMin: 0,
                            yMax: 0,
                            borderColor: this.ChartColors.SoakTempBox.Stroke,
                            backgroundColor: this.ChartColors.SoakTempBox.Fill,
                            borderWidth: 1,
                            borderDash: [2, 2],
                            label: {
                                position: { x: 'start', y: 'start'},
                                // position: "center",
                                backgroundColor: this.ChartColors.SoakTempBox.Fill,
                                color: this.ChartColors.SoakTempBox.TextColor,
                                content: "soak zone",
                                font: {
                                    family: "'Ubuntu Condensed', sans-serif",
                                    size: 20,
                                    weight: "normal"
                                },
                                display: true
                            }
                        },
                        reflowTemperatureBox: {
                            display: false,
                            type: "box",
                            drawTime: "beforeDraw",
                            xMin: 0,
                            xMax: 0,
                            yMin: 0,
                            yMax: 0,
                            borderColor: this.ChartColors.ReflowTempBox.Stroke,
                            backgroundColor: this.ChartColors.ReflowTempBox.Fill,
                            borderWidth: 1,
                            borderDash: [2, 2],
                            label: {
                                // position: { x: 'start', y: 'start'},
                                position: "center",
                                backgroundColor: this.ChartColors.ReflowTempBox.Fill,
                                color: this.ChartColors.ReflowTempBox.TextColor,
                                content: ["reflow", " zone"],
                                font: {
                                    family: "'Ubuntu Condensed', sans-serif",
                                    size: 20,
                                    weight: "normal"
                                },
                                display: true
                            }
                        }
                    }
                },
                title: {
                    display: true,
                    position: "top",
                    padding: {
                        top: 5,
                        bottom: 0,
                    },
                    text: this.ChartTitleText,
                    color: this.ChartColors.Title.TitleColor,
                    font: {
                        family: "'Ubuntu Condensed', sans-serif",
                        size: 24,
                        weight: "normal"
                    }
                },
                subtitle: {
                    display: true,
                    position: "top",
                    padding: {
                        top: 0,
                        bottom: 5,
                    },
                    text: this.ChartSubtitleText,
                    color: this.ChartColors.Title.SubtitleColor,
                    font: {
                        family: "'Ubuntu Condensed', sans-serif",
                        size: 18,
                        weight: "normal"
                    }
                },
                legend: {
                    display: true,
                    position: "bottom",
                    align: "start",
                    labels: {
                        font: {
                            family: "'Ubuntu Condensed', sans-serif",
                            size: 14,
                            weight: "normal"
                        },
                        color: this.ChartColors.X_Axis.Stroke,
                    },
                }
            }
        };
    }

    // ------------------------------------------------------------------------
    // Chart Init
    // ------------------------------------------------------------------------
    Init(canvasElement = null) {
        if (this.isInitialized) {
            console.warn('Chart already initialized');
            return;
        }

        try {
            let canvas;

            // If canvas element is provided, use it; otherwise look for default ID
            if (canvasElement) {
                if (typeof canvasElement === 'string') {
                    canvas = document.getElementById(canvasElement);
                } else if (canvasElement instanceof HTMLCanvasElement) {
                    canvas = canvasElement;
                } else {
                    console.error('Invalid canvas element provided');
                    return false;
                }
            } else {
                canvas = document.getElementById('ReflowChart');
            }

            if (!canvas) {
                console.error('Canvas element not found');
                return false;
            }

            const ctx = canvas.getContext('2d');
            if (!ctx) {
                console.error('Could not get 2D context from canvas');
                return false;
            }

            // Create chart with Chart.js v4 syntax
            this.ReflowChart = new Chart(ctx, {
                type: 'line',
                data: this.ChartData,
                options: { ...this.ChartOptions }
            });

            this.isInitialized = true;
            console.log('Chart initialized successfully');
            return true;

        } catch (error) {
            console.error('Error initializing chart:', error);
            return false;
        }
    }

    // ------------------------------------------------------------------------
    // Chart Destroy
    // ------------------------------------------------------------------------
    Destroy() {
        if (this.ReflowChart) {
            this.ReflowChart.destroy();
            this.ReflowChart = null;
        }
        this.isInitialized = false;
        console.log('Chart destroyed');
    }

    // ------------------------------------------------------------------------
    // Clear Chart Data
    // ------------------------------------------------------------------------
    ClearData(_datasetIndex = 1) {
        if (!this.ReflowChart) {
            console.warn('Chart not initialized');
            return;
        }

        if (_datasetIndex >= 0 && _datasetIndex < this.ReflowChart.data.datasets.length) {
            this.ReflowChart.data.datasets[_datasetIndex].data = [];
            console.log('deleting dataset: ' + _datasetIndex);
            this.ReflowChart.update();
        }
    }

    // ------------------------------------------------------------------------
    // Clear All Data
    // ------------------------------------------------------------------------
    ClearAllData() {
        if (!this.ReflowChart) {
            console.warn('Chart not initialized');
            return;
        }

        this.ReflowChart.data.datasets.forEach(dataset => {
            dataset.data = [];
        });
        this.X_Pos = 0;
        this.ReflowChart.update();
    }

    // ------------------------------------------------------------------------
    // Update Profile Chart Data
    // ------------------------------------------------------------------------
    UpdateProfileChartData(profileData, startTemp = null) {
        const actualStartTemp = (startTemp !== null) ? Math.round(startTemp) : this.ChartMinTemperature;

        if (!profileData) {
            console.warn('No profile data provided');
            return;
        }

        let tmpX_Scale_Max = 0;
        let tmpY_Scale_Max = 0;

        // Clear existing dataset and rebuild dynamically
        if (this.ReflowChart) {
            this.ReflowChart.data.datasets[0].data = [];

            // Profile point 0: Start at room temperature
            this.ReflowChart.data.datasets[0].data.push({
                x: 0,
                y: actualStartTemp
            });

            // Profile point 1: End of preheat phase
            this.ReflowChart.data.datasets[0].data.push({
                x: profileData.preheat.time_sec,
                y: profileData.preheat.max_temp
            });

            // Profile point 2: End of soak phase
            this.ReflowChart.data.datasets[0].data.push({
                x: profileData.preheat.time_sec + profileData.soak.time_sec,
                y: profileData.soak.max_temp
            });

            // Profile point 3: Start of reflow phase (calculated ramp to reflow min temp)
            let tmpValue = Math.round((profileData.preheat.max_temp - actualStartTemp) / profileData.preheat.time_sec);
            let reflowStartTime = (profileData.preheat.time_sec + profileData.soak.time_sec) +
                Math.abs(profileData.reflow.min_temp - profileData.soak.max_temp) / tmpValue;
            let reflowMidTemp = Math.ceil(profileData.reflow.min_temp +
                ((profileData.reflow.max_temp - profileData.reflow.min_temp) / 2));

            this.ReflowChart.data.datasets[0].data.push({
                x: reflowStartTime,
                y: reflowMidTemp
            });

            // Profile point 4: End of reflow phase
            this.ReflowChart.data.datasets[0].data.push({
                x: reflowStartTime + profileData.reflow.time_sec,
                y: reflowMidTemp
            });

            // Profile point 5: End of cooling phase (back to room temperature)
            let coolingTime = profileData.reflow.max_temp / Math.abs(profileData.cooling.max_rate);
            this.ReflowChart.data.datasets[0].data.push({
                x: reflowStartTime + profileData.reflow.time_sec + coolingTime,
                y: actualStartTemp
            });

            // Calculate scale maximums
            let lastPoint = this.ReflowChart.data.datasets[0].data[this.ReflowChart.data.datasets[0].data.length - 1];
            tmpValue = lastPoint.x + (lastPoint.x * 0.1);
            tmpX_Scale_Max = Math.ceil(tmpValue / this.ChartStepSize) * this.ChartStepSize;

            tmpValue = profileData.absolute_peak_temp + (profileData.absolute_peak_temp * 0.1);
            tmpY_Scale_Max = Math.ceil(tmpValue / 10) * 10;

            // Update chart options
            this.ReflowChart.options.scales.x.max = tmpX_Scale_Max;
            this.ReflowChart.options.scales.y.min = Math.floor(actualStartTemp / 10) * 10;
            this.ReflowChart.options.scales.y.max = tmpY_Scale_Max;
            this.ReflowChart.options.plugins.subtitle.text = profileData.name + " - " + profileData.description;

            // Update annotations
            this.ReflowChart.options.plugins.annotation.annotations.solderMeltingLine.yMin = profileData.solder_melting_point;
            this.ReflowChart.options.plugins.annotation.annotations.solderMeltingLine.yMax = profileData.solder_melting_point;

            this.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.yMin = profileData.absolute_peak_temp;
            this.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.yMax = profileData.absolute_peak_temp;

            this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.xMin = this.ChartMinTicks + 1;
            this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.yMin = profileData.absolute_peak_temp;
            this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.xMax = tmpX_Scale_Max;
            this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.yMax = tmpY_Scale_Max - 1;

            this.ReflowChart.options.plugins.annotation.annotations.soakTemperatureBox.xMin = profileData.preheat.time_sec;
            this.ReflowChart.options.plugins.annotation.annotations.soakTemperatureBox.yMin = profileData.preheat.max_temp;
            this.ReflowChart.options.plugins.annotation.annotations.soakTemperatureBox.xMax = profileData.preheat.time_sec + profileData.soak.time_sec;
            this.ReflowChart.options.plugins.annotation.annotations.soakTemperatureBox.yMax = profileData.soak.max_temp;

            this.ReflowChart.options.plugins.annotation.annotations.reflowTemperatureBox.xMin = reflowStartTime;
            this.ReflowChart.options.plugins.annotation.annotations.reflowTemperatureBox.yMin = profileData.reflow.min_temp;
            this.ReflowChart.options.plugins.annotation.annotations.reflowTemperatureBox.xMax = reflowStartTime + profileData.reflow.time_sec;
            this.ReflowChart.options.plugins.annotation.annotations.reflowTemperatureBox.yMax = profileData.reflow.max_temp;

            // Update the chart
            this.ReflowChart.update();
        } else {
            // If chart not initialized yet, update the default values for later use
            this.ChartOptions.scales.x.max = tmpX_Scale_Max;
            this.ChartOptions.scales.y.max = tmpY_Scale_Max;
            this.ChartOptions.plugins.subtitle.text = profileData.name + " - " + profileData.description;
        }
    }

    // ------------------------------------------------------------------------
    // Update Manual Tuning Chart Data
    // ------------------------------------------------------------------------
    UpdateManualTuningChartData(tuningData) {
        if (!this.ReflowChart || !tuningData) {
            console.warn('Chart not initialized or no tuning data');
            return;
        }

        let tmpValue = 0;
        let tmpX_Value = 0;
        let tmpY_Value = 0;
        let tmpX_Scale_Max = 0;
        let tmpY_Scale_Max = 0;


        this.ReflowChart.data.datasets[0].data = [];

        // chart line point 0
        tmpX_Value = 0;
        tmpY_Value = this.ChartMinTemperature;
        this.ReflowChart.data.datasets[0].data.push({x: tmpX_Value, y: tmpY_Value});

        // chart line point 1
        tmpX_Value = tuningData.parameters.SET.preheat.time_sec;
        tmpY_Value = tuningData.parameters.SET.preheat.temp;
        this.ReflowChart.data.datasets[0].data.push({x: tmpX_Value, y: tmpY_Value});

        // chart line point 2
        tmpX_Value = tmpX_Value + tuningData.parameters.SET.testing.time_sec;
        tmpY_Value = tuningData.parameters.SET.preheat.temp;
        this.ReflowChart.data.datasets[0].data.push({x: tmpX_Value, y: tmpY_Value});

        // chart line point 3
        tmpX_Value = tmpX_Value + (tuningData.parameters.SET.preheat.temp / Math.abs(tuningData.parameters.SET.cooling.rate));
        tmpY_Value = this.ChartMinTemperature;
        this.ReflowChart.data.datasets[0].data.push({x: tmpX_Value, y: tmpY_Value});

        // let tmpValue = Math.round((tuningData.SET.preheat.temp - this.ChartMinTemperature) / tuningData.SET.preheat.time_sec);

        this.ChartOptions.plugins.subtitle.text = tuningData.name;

        // Calculate scale maximums
        let lastPoint = this.ReflowChart.data.datasets[0].data[this.ReflowChart.data.datasets[0].data.length - 1];
        tmpValue = lastPoint.x + (lastPoint.x * 0.1);
        tmpX_Scale_Max = Math.ceil(tmpValue / this.ChartStepSize) * this.ChartStepSize;

        tmpValue = tuningData.parameters.SET.max_temperature + (tuningData.parameters.SET.max_temperature * 0.1);
        tmpY_Scale_Max = Math.ceil(tmpValue / 10) * 10;

        this.ReflowChart.options.scales.x.max = tmpX_Scale_Max;
        this.ReflowChart.options.scales.y.max = tmpY_Scale_Max;
        this.ReflowChart.options.plugins.subtitle.text = this.ChartOptions.plugins.subtitle.text;

        this.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.yMin = tuningData.parameters.SET.max_temperature;
        this.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.yMax = tuningData.parameters.SET.max_temperature;

        this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.xMin = this.ChartMinTicks + 1;
        this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.yMin = tuningData.parameters.SET.max_temperature;
        this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.xMax = tmpX_Scale_Max;
        this.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.yMax = tmpY_Scale_Max - 1;

        if (this.ReflowChart) {
            this.ReflowChart.update();
        }
    }


}

