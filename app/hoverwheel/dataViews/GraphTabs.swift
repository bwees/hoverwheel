//
//  BatteryCircle.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/18/24.
//

import SwiftUI
import AudioToolbox

struct GraphTabs: View {
    @ObservedObject var battery: FloatAttribute
    @ObservedObject var motorOutput: IntAttribute
    @ObservedObject var wheelSpeed: IntAttribute
    @ObservedObject var boardState: IntAttribute
    @ObservedObject var maxMotorPWM: IntAttribute
    
    let timer = Timer.publish(every: 0.25, on: .main, in: .common).autoconnect()

    var body: some View {
        TabView {
            
            // Battery
            ZStack {
                Gauge(value: battery.value ?? 0, in: 34...42) {
                    
                }
                .tint((battery.value ?? 0) < 35 ? Color.red : ((battery.value ?? 0) < 37 ? Color.yellow : Color.green))
                .gaugeStyle(.accessoryCircular)
                .frame(width: 200, height: 200) // adjust size as needed
                .scaleEffect(3)
                VStack {
                    Text(String(round((battery.value ?? 0)*100)/100.0))
                        .font(.system(size: 38))
                        .bold()
                        .offset(y: 5)
                    Text("VOLTS")
                        .bold()
                        .offset(y: 30)
                        .font(.title3)
                }
            }
            
            // Motor Duty %
            ZStack {
                Gauge(value: abs(Double(motorOutput.value ?? 0)/Double((maxMotorPWM.value ?? 1))), in: 0...1) {
                    
                }
                .tint({
                    let motorCmd = Double(abs(motorOutput.value ?? 0))
                    let maxMotor = Double(maxMotorPWM.value ?? 0)
                    
                    if (motorCmd > maxMotor*0.75) {
                        return Color.yellow
                    } else if (motorCmd > maxMotor*0.85) {
                        return Color.red
                    } else {
                        return Color.green
                    }
                }())
                .gaugeStyle(.accessoryCircular)
                .frame(width: 200, height: 200) // adjust size as needed
                .scaleEffect(3)
                VStack {
                    Text(String(Int(round(abs(Double(motorOutput.value ?? 0)/Double((maxMotorPWM.value ?? 1)))*100))))
                        .font(.system(size: 38))
                        .bold()
                        .offset(y: 5)
                    Text("%")
                        .bold()
                        .offset(y: 30)
                        .font(.title3)
                }
            }
            
            // Speedometer
            ZStack {
                Gauge(value: abs(Double(-(wheelSpeed.value ?? 0))*0.03270833333), in: 0...10) {
                    
                }
                .tint(Color.blue)
                .gaugeStyle(.accessoryCircular)
                .frame(width: 200, height: 200) // adjust size as needed
                .scaleEffect(3)
                VStack {
                    Text(String(abs(round(Double(-(wheelSpeed.value ?? 0))*0.03270833333*10)/10.0)))
                        .font(.system(size: 38))
                        .bold()
                        .offset(y: 5)
                    
                    Text("MPH")
                        .bold()
                        .offset(y: 30)
                        .font(.title3)
                }
            }
        }
        .tabViewStyle(.page)
        .frame(maxHeight: 225)
        .onReceive(timer) { _ in
            if (boardState.value == 2   ) {
                AudioServicesPlayAlertSound(SystemSoundID(kSystemSoundID_Vibrate))
            }
        }
    }
}

