//
//  ContentView.swift
//  hoverwheel-watch Watch App
//
//  Created by Brandon Wees on 1/18/24.
//

import SwiftUI

struct ContentView: View {
    
    @StateObject var extendedSession = ExtendedRuntimeManager()
    @ObservedObject var btManager = BluetoothManager()
    @State var connectionSheetShown: Bool = true
    @Environment(\.scenePhase) var scenePhase
    
    var body: some View {
        InterfaceTabs(
            btManager: btManager,
            battery: btManager.boardTelemetry[BATTERY_TELEM] as! FloatAttribute,
            motorOutput: btManager.boardTelemetry[PWM_TELEM] as! IntAttribute,
            maxMotorPWM: btManager.boardSettings[MAX_MOTOR_PWM] as! IntAttribute,
            wheelSpeed: btManager.boardTelemetry[WHEEL_SPEED_TELEM] as! IntAttribute,
            boardState: btManager.boardTelemetry[STATE_TELEM] as! IntAttribute,
            headlight: btManager.boardSettings[HEADLIGHT_SET] as! BoolAttribute
        )
            .sheet(isPresented: $connectionSheetShown) {
                ConnectBTView(btManager: btManager)
                    .toolbar(content: {
                       // Remove the (x) on watchOS 10
                       ToolbarItem(placement: .cancellationAction) {
                          Button("", action: {}).opacity(0.0).disabled(true)
                       }
                    })
            }
            .onChange(of: btManager.connectionState, {
                connectionSheetShown = btManager.connectionState != .connected
                
                if (btManager.connectionState != .connected) {
                    if (extendedSession.sessionState == .running) {
                        extendedSession.endSession()
                    }
                } else if (btManager.connectionState == .connected) {
                    extendedSession.startSession()
                }
            })
            .onChange(of: scenePhase , {
                if scenePhase != .active && extendedSession.sessionState != .running && btManager.connectionState == .connected {
                    btManager.disconnectPeripheral()
                }
            })
            
    }
}
