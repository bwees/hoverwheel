//
//  Settings.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/17/24.
//

import SwiftUI

struct Settings: View {
    
    @ObservedObject var btManager: BluetoothManager
    @State var showSavedAlert = false

    
    var body: some View {

        Form {
            Section(header: Text("PID Loop")) {
                LabeledFloatInput(setting: btManager.boardSettings[RIDE_ANGLE_SET] as! FloatAttribute)
                LabeledFloatInput(setting: btManager.boardSettings[IMU_OFFSET_SET] as! FloatAttribute)
                LabeledIntInput(setting: btManager.boardSettings[ANGLE_P_SET] as! IntAttribute)
                LabeledIntInput(setting: btManager.boardSettings[RATE_P_SET] as! IntAttribute)
                LabeledFloatInput(setting: btManager.boardSettings[FILTER_KP_SET] as! FloatAttribute)
                LabeledIntInput(setting: btManager.boardSettings[CKP_PRESCALE_SET] as! IntAttribute)
                LabeledIntInput(setting: btManager.boardSettings[CKP_POSTSCALE_SET] as! IntAttribute)
                LabeledIntInput(setting: btManager.boardSettings[MAX_MOTOR_PWM] as! IntAttribute)

            }
            Section(header: Text("Footpads")) {
                LabeledIntInput(setting: btManager.boardSettings[FOOTPAD_A_THRESH_SET] as! IntAttribute)
                LabeledIntInput(setting: btManager.boardSettings[FOOTPAD_B_THRESH_SET] as! IntAttribute)
            }
            Section(header: Text("Pushback")) {
                LabeledFloatInput(setting: btManager.boardSettings[PUSHBACK_THRESH_SET] as! FloatAttribute)
                LabeledIntInput(setting: btManager.boardSettings[PUSHBACK_TIME_SET] as! IntAttribute)
                LabeledFloatInput(setting: btManager.boardSettings[PUSHBACK_SPEED_SET] as! FloatAttribute)
                LabeledFloatInput(setting: btManager.boardSettings[PUSHBACK_AMOUNT_SET] as! FloatAttribute)

            }
            Section(header: Text("Step Up")) {
                LabeledFloatInput(setting: btManager.boardSettings[STEP_UP_ANGLE_SET] as! FloatAttribute)
                LabeledFloatInput(setting: btManager.boardSettings[STEP_UP_SPEED_SET] as! FloatAttribute)
            }
        }
            .scrollDismissesKeyboard(.interactively)
            .toolbar {
                Button("Save") {
                    showSavedAlert = true
            
                    btManager.writeSettings()
                    
                    // close keyboard
                    UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
                }
            }
            .alert(isPresented: $showSavedAlert) {
                Alert(
                    title: Text("Success"),
                    message: Text("Settings have been successfully saved to board")
                )
            }
        
    }
}
