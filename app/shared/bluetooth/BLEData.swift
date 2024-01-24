//
//  BLEData.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/18/24.
//

import Foundation
import CoreBluetooth

protocol GATTAttribute: ObservableObject {
    var name: String { get set }
    var charachteristic: CBCharacteristic? { get set }
    func setValue(data: Data)
    func getData() -> Data
}

class IntAttribute: GATTAttribute {
    @Published var charachteristic: CBCharacteristic?
    @Published var name: String
    @Published var value: Int32?
    
    init(name: String) {
        self.name = name
    }
    
    func setValue(data: Data) {
        self.value = data.withUnsafeBytes { $0.load(as: Int32.self) }
    }
    
    func setValue(with newValue: Int32) {
        self.value = newValue
    }
    
    func getData() -> Data {
        return withUnsafeBytes(of: self.value) { Data($0) }
    }
}

class FloatAttribute: GATTAttribute {
    @Published var charachteristic: CBCharacteristic?
    @Published var name: String
    @Published var value: Float32?
    
    init(name: String) {
        self.name = name
    }
    
    func setValue(data: Data) {
        self.value = data.withUnsafeBytes { $0.load(as: Float32.self) }
    }
    
    func setValue(with newValue: Float32) {
        self.value = newValue
    }
    
    func getData() -> Data {
        return withUnsafeBytes(of: self.value) { Data($0) }
    }
}

class BoolAttribute: GATTAttribute {
    @Published var charachteristic: CBCharacteristic?
    @Published var name: String
    @Published var value: Bool?
    
    init(name: String) {
        self.name = name
    }
    
    func setValue(data: Data) {
        self.value = data.withUnsafeBytes { $0.load(as: Bool.self) }
    }
    
    func setValue(with newValue: Bool) {
        self.value = newValue
    }
    
    func getData() -> Data {
        return withUnsafeBytes(of: self.value) { Data($0) }
    }
}
