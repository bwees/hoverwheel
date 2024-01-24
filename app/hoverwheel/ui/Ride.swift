//
//  Ride.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/17/24.
//

import SwiftUI

struct Ride: View {
    
    @ObservedObject var btManager: BluetoothManager

    var body: some View {
        VStack {
            
            GraphTabs(
                battery: btManager.boardTelemetry[BATTERY_TELEM] as! FloatAttribute,
                motorOutput: btManager.boardTelemetry[PWM_TELEM] as! IntAttribute,
                wheelSpeed: btManager.boardTelemetry[WHEEL_SPEED_TELEM] as! IntAttribute,
                boardState: btManager.boardTelemetry[STATE_TELEM] as! IntAttribute,
                maxMotorPWM: btManager.boardSettings[MAX_MOTOR_PWM] as! IntAttribute
            )

            Divider()
            
            Form {
                Section(header: Text("Ride Controls")) {
                    LabeledBoolInput(setting: btManager.boardSettings[HEADLIGHT_SET] as! BoolAttribute, changeCallback: {
                        btManager.writeSetting(for: HEADLIGHT_SET)
                    })
                    CompensatedKP(compensatedKP: btManager.boardTelemetry[COMPENSATED_KP_TELEM] as! FloatAttribute)
                }
            }
            Spacer()
        }
        .toolbar {
            Button("Disconnect") {
                btManager.shouldAutoconnect = false
                btManager.disconnectPeripheral()
            }
        }
    }
}
