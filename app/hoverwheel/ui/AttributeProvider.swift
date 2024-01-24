//
//  AttributeProvider.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/18/24.
//

import SwiftUI

struct AttributeProvider<T : GATTAttribute>: View {
    
    @ObservedObject var attribute:T

    var body: some View {
        children
    }
}
