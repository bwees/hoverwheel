//
//  Main.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/17/24.
//

import SwiftUI

enum Tabs: String {
    case settings, ride
}

struct MainTabs: View {
    
    @ObservedObject private var btManager = BluetoothManager()
    @State var connectionSheetShown: Bool = true
    @State var selectedTab: Tabs = .ride
    @Environment(\.scenePhase) var scenePhase
        
    var body: some View {
        TabView(selection: $selectedTab) {
            NavigationView {
                Ride(
                    btManager: btManager
                )
                    .navigationTitle("Ride")
            }.tabItem {
                Label("Ride", systemImage: "figure.skating")
            }
            .tag(Tabs.ride)
            
            NavigationView {
                Settings(btManager: btManager)
                    .navigationTitle("Settings")
            }.tabItem {
                Label("Settings", systemImage: "slider.horizontal.3")
            }
            .tag(Tabs.settings)
        }
            .sheet(isPresented: $connectionSheetShown) {
                ConnectBTView(btManager: btManager)
            }
            .onChange(of: btManager.connectionState, {
                connectionSheetShown = btManager.connectionState != .connected
            })
            .onChange(of: scenePhase, {
                if scenePhase != .active {
                    btManager.disconnectPeripheral()
                }
            })
    }
}

