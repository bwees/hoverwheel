//
//  ContentView.swift
//  core-bluetooth
//
//  Created by Andrew on 2022-07-14.
//

import SwiftUI
import CoreBluetooth


struct ConnectBTView: View {
    @ObservedObject var btManager: BluetoothManager
    @Environment(\.dismiss) var dismiss

    var body: some View {
        NavigationView {
            List(btManager.peripherals, id: \.self) { peripheral in
                Button(action: {
                    btManager.connectToPeripheral(peripheral: peripheral)
                }) {
                    HStack {
                        Text(peripheral.name ?? "Unknown")
                        
                        Spacer()
                        
                        if peripheral == btManager.selectedDevice && btManager.connectionState == .connecting {
                            ProgressView()
                                .progressViewStyle(CircularProgressViewStyle())
                        }
                    }
                }
            }
            .navigationTitle("Connect")
            .onChange(of: btManager.connectionState, {
                if (btManager.connectionState == .connected) {
                    dismiss()
                }
            })
            .toolbar {
                Button {
                    btManager.scanForDevices()
                } label: {
                    Label("Refresh", systemImage: "arrow.clockwise")
                }
                .tint(Color.blue)
            }
            .interactiveDismissDisabled()
        }
    }
}
