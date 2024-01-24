//
//  HoverwheelBLE.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/17/24.
//

import Foundation
import CoreBluetooth
import SwiftUI
import Combine

enum BTConnectionState {
    case connected, disconnected, connecting
}

class BluetoothManager: NSObject, ObservableObject, Observable {
    private var centralManager: CBCentralManager?
    
    @Published var peripherals: [CBPeripheral] = []
    @Published var selectedDevice: CBPeripheral?
    @Published var connectionState: BTConnectionState = .disconnected
    @Published var shouldAutoconnect: Bool = true
    
    @Published var boardSettings: [CBUUID: any GATTAttribute] = [
        ANGLE_P_SET: IntAttribute(name: "Angle P"),
        RATE_P_SET: IntAttribute(name: "Rate P"),
        FILTER_KP_SET: FloatAttribute(name: "Filter KP"),
        FOOTPAD_A_THRESH_SET: IntAttribute(name: "Footpad A Threshold"),
        FOOTPAD_B_THRESH_SET: IntAttribute(name: "Footpad B Threshold"),
        PUSHBACK_THRESH_SET: FloatAttribute(name: "Pushback Threshold"),
        PUSHBACK_TIME_SET: IntAttribute(name: "Pushback Time"),
        PUSHBACK_SPEED_SET: FloatAttribute(name: "Pushback Speed"),
        PUSHBACK_AMOUNT_SET: FloatAttribute(name: "Pushback Amount"),
        RIDE_ANGLE_SET: FloatAttribute(name: "Ride Angle"),
        STEP_UP_ANGLE_SET: FloatAttribute(name: "Step Up Angle"),
        STEP_UP_SPEED_SET: FloatAttribute(name: "Step Up Speed"),
        IMU_OFFSET_SET: FloatAttribute(name: "IMU Offset"),
        HEADLIGHT_SET: BoolAttribute(name: "Headlight"),
        CKP_PRESCALE_SET: IntAttribute(name: "Compensation Prescale"),
        CKP_POSTSCALE_SET: IntAttribute(name: "Compensation Postscale"),
        MAX_MOTOR_PWM: IntAttribute(name: "Max Motor PWM")
    ]
    
    @Published var boardTelemetry: [CBUUID: any GATTAttribute] = [
        STATE_TELEM: IntAttribute(name: "Stat"),
        ANGLE_TELEM: FloatAttribute(name: "Board Angle"),
        PWM_TELEM: IntAttribute(name: "Motor PWM"),
        
        FOOTPAD_A_TELEM: IntAttribute(name: "Footpad A"),
        FOOTPAD_B_TELEM: IntAttribute(name: "Footpad B"),
        LOOP_TIME_TELEM: IntAttribute(name: "Loop Time"),
        TARGET_TELEM: FloatAttribute(name: "Target Angle"),
        BATTERY_TELEM: FloatAttribute(name:"Battery Voltage"),
        BOARD_TEMP_TELEM: FloatAttribute(name:"Board Temperature"),
        WHEEL_SPEED_TELEM: IntAttribute(name: "Speed"),
        COMPENSATED_KP_TELEM: FloatAttribute(name: "Compensated KP")
    ]

    override init() {
        super.init()
        self.centralManager = CBCentralManager(delegate: self, queue: .main)
    }
}

extension BluetoothManager: CBCentralManagerDelegate, CBPeripheralDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        scanForDevices()
    }
    
    func connectToPeripheral(peripheral: CBPeripheral) {
        self.centralManager?.connect(peripheral)
        self.selectedDevice = peripheral
        self.connectionState = .connecting
    }
    
    func disconnectPeripheral() {
        if self.selectedDevice == nil || self.connectionState != .connected {
            return
        }
        self.centralManager?.cancelPeripheralConnection(self.selectedDevice!)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        self.connectionState = .connected
        self.selectedDevice = peripheral
        
        peripheral.delegate = self
        peripheral.discoverServices([
            SETTINGS_SERVICE,
            TELEMETRY_SERVICE
        ])
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        self.connectionState = .disconnected
        self.selectedDevice = nil
        self.peripherals = []
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        self.connectionState = .disconnected
        self.selectedDevice = nil
        self.peripherals = []
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        if (!peripherals.contains(peripheral)) {
            self.peripherals.append(peripheral)
            if shouldAutoconnect {
                self.connectToPeripheral(peripheral: peripheral)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if error != nil {
            print("Error discovering services: \(String(describing: error?.localizedDescription))")
            return
        }

        if let services = peripheral.services {
            for service in services {
                peripheral.discoverCharacteristics(nil, for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if error != nil {
            print("Error discovering services: \(String(describing: error?.localizedDescription))")
            return
        }
        
        if service.uuid == SETTINGS_SERVICE {
            // Read all of the charachteristics in
            for char in service.characteristics! {
                service.peripheral?.readValue(for: char)
                
                if (boardSettings.keys.contains(char.uuid)) { // Settings
                    boardSettings[char.uuid]?.charachteristic = char
                }
            }
        }
        
        else if service.uuid == TELEMETRY_SERVICE {
            // Read all of the charachteristics in
            for char in service.characteristics! {
                service.peripheral?.readValue(for: char)
                peripheral.setNotifyValue(true, for: char)
                
                if (boardTelemetry.keys.contains(char.uuid)) { // Settings
                    boardTelemetry[char.uuid]?.charachteristic = char
                }
            }
        }

    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
            if (boardSettings.keys.contains(characteristic.uuid)) {
                boardSettings[characteristic.uuid]?.setValue(data: characteristic.value!)
            } else if (boardTelemetry.keys.contains(characteristic.uuid)) { // Telemetry
                boardTelemetry[characteristic.uuid]?.setValue(data: characteristic.value!)
            }
    }
    
    func writeSettings() {
        for (_, setting) in boardSettings {
            selectedDevice?.writeValue(setting.getData(), for: setting.charachteristic!, type: .withResponse)
        }
    }
    
    func writeSetting(for uuid: CBUUID) {
        let setting = boardSettings[uuid]!
        selectedDevice?.writeValue(setting.getData(), for: setting.charachteristic!, type: .withResponse)
    }
    
    func scanForDevices() {
        if centralManager?.state == .poweredOn {
            self.centralManager?.scanForPeripherals(withServices: [
                SETTINGS_SERVICE,
                TELEMETRY_SERVICE
            ])
        }
    }
}
