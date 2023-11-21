var characheristics = {}
var device = null
var charts = configureCharts()

function linspace(start, stop, num, endpoint = true) {
    const div = endpoint ? (num - 1) : num;
    const step = (stop - start) / div;
    return Array.from({length: num}, (_, i) => start + step * i);
}

function connect() {
    if (device) {
        device.disconnect()
        characheristics = {}
        device = null
        document.getElementById("connect").innerHTML = "Connect"
        return
    }

    navigator.bluetooth
        .requestDevice({
            filters: [{ name: "Hoverwheel" }],
            optionalServices: ["19b10000-e8f2-537e-4f6c-d104768a1215", "19a10000-e8f2-537e-4f6c-d104768a1215"]
        })
        .then((device) => {
            // Step 2: Connect to it
            console.log("device:", device)
            return device.gatt.connect()
        })
        .then(async (server) => {
            device = server

            document.getElementById("connect").innerHTML = "Disconnect"

            // Step 4: get the Characteristic
            var controlService = await server.getPrimaryService("19b10000-e8f2-537e-4f6c-d104768a1215")
            var statsService = await server.getPrimaryService("19a10000-e8f2-537e-4f6c-d104768a1215")

            characheristics.pChar = await controlService.getCharacteristic("19b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.rpChar = await controlService.getCharacteristic("39b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.fkpChar = await controlService.getCharacteristic("a9b1cc01-e8f2-537e-4f6c-d104768a1214")

            // stats
            characheristics.state = await statsService.getCharacteristic("49b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.angle = await statsService.getCharacteristic("59b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.pwm = await statsService.getCharacteristic("69b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.footpada = await statsService.getCharacteristic("79a10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.footpadb = await statsService.getCharacteristic("79b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.loopTime = await statsService.getCharacteristic("89b10001-e8f2-537e-4f6c-d104768a1214")
            characheristics.target = await statsService.getCharacteristic("99b10001-e8f2-537e-4f6c-d104768a1214")

            // subscribe to the stats
            characheristics.state.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("state").value = event.target.value.getInt32(0, true)
                })
            })

            characheristics.angle.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("angle").value = event.target.value.getFloat32(0, true)

                    // only keep the last 100 values
                    if (charts.motor.data.datasets[0].data.length > 100) {
                        charts.motor.data.datasets[0].data.shift()
                    }

                    charts.motor.data.datasets[0].data.push(event.target.value.getFloat32(0, true))
                    charts.motor.update()
                })
            })

            characheristics.pwm.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("pwm").value = event.target.value.getInt32(0, true)
                    // only keep the last 100 values
                    if (charts.motor.data.datasets[2].data.length > 100) {
                        charts.motor.data.datasets[2].data.shift()
                    }

                    charts.motor.data.datasets[2].data.push(event.target.value.getInt32(0, true))
                    charts.motor.update()
                })
            })

            characheristics.target.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("target").value = event.target.value.getFloat32(0, true)

                    // only keep the last 100 values
                    if (charts.motor.data.datasets[1].data.length > 100) {
                        charts.motor.data.datasets[1].data.shift()
                    }

                    charts.motor.data.datasets[1].data.push(event.target.value.getFloat32(0, true))
                    charts.motor.update()
                })
            })

            characheristics.footpada.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("footpada").value = event.target.value.getInt32(0, true)
                })
            })
            characheristics.footpadb.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("footpadb").value = event.target.value.getInt32(0, true)
                })
            })

            characheristics.loopTime.startNotifications().then((char) => {
                char.addEventListener('characteristicvaluechanged', (event) => {
                    document.getElementById("loopTime").value = event.target.value.getInt32(0, true)


                    // only keep the last 100 values
                    if (charts.timing.data.datasets[0].data.length > 100) {
                        charts.timing.data.datasets[0].data.shift()
                    }

                    charts.timing.data.datasets[0].data.push(event.target.value.getInt32(0, true))
                    charts.timing.update()
                })
            })

            // update the values on the page from each characteristic
            characheristics.pChar.readValue().then((value) => {
                document.getElementById("p").value = value.getInt32(0, true)
            })
            characheristics.rpChar.readValue().then((value) => {
                document.getElementById("rp").value = value.getInt32(0, true)
            })
            characheristics.fkpChar.readValue().then((value) => {
                console.log(value.getFloat32(0, true))
                document.getElementById("fkp").value = value.getFloat32(0, true)
            })
        })

        .catch((error) => {
            console.log(error)
        })
}

async function updateValue() {
    var p = document.getElementById("p").value
    var rp = document.getElementById("rp").value
    var fkp = document.getElementById("fkp").value

    var data = new Uint32Array(1)

    // int32 array for p
    data[0] = p
    await characheristics.pChar.writeValue(data)

    // int32 array for rp
    data[0] = rp
    await characheristics.rpChar.writeValue(data)

    // int32 array for rp
    data[0] = fkp
    await characheristics.fkpChar.writeValue(data)
}

function configureCharts() {
    var motor = new Chart(document.getElementById("motorChart"), {
        type: 'line',
        data: {
            labels: linspace(-5000, 0, 100, false),
            datasets: [
                {
                    label: 'Board Angle (deg)',
                    data: Array(100).fill(null)
                },
                {
                    label: 'Target Angle (deg)',
                    data: Array(100).fill(null)
                },
                {
                    label: 'Motor Output',
                    data: Array(100).fill(null),
                    yAxisID: 'y1'

                }
            ]
        },
        options: {
            events: ["click"],
            animation: {
                duration: 0
            },
            scales: {
              y: { // defining min and max so hiding the dataset does not change scale range
                min: -45,
                max: 45
              },
              y1: {
                type: 'linear',
                display: true,
                position: 'right',
                min: -600,
                max: 600,
        
                grid: {
                  drawOnChartArea: false, // only want the grid lines for one axis to show up
                },
              },
              x: {
                ticks: {
                    maxTicksLimit: 10
                  }
              }
            },
            elements: {
                point:{
                    radius: 0
                }
            }
          }
    });

    var timing = new Chart(document.getElementById("timingChart"), {
        type: 'line',
        data: {
            labels: linspace(-5000, 0, 100, false),
            datasets: [{
                label: 'Loop Time (us)',
                data: Array(100).fill(null)
            }]
        },

        options: {
            events: ["click"],
            animation: {
                duration: 0
            },
            scales: {
              y: { // defining min and max so hiding the dataset does not change scale range
                min: 500,
                max: 1500
              },
              x: {
                ticks: {
                    maxTicksLimit: 10
                  }
              }
            },
            elements: {
                point:{
                    radius: 0
                }
            }
          }
    });

    return { motor: motor, timing: timing }
}