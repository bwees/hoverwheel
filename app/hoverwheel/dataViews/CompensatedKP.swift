//
//  SpeedView.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/18/24.
//

import SwiftUI

struct CompensatedKP: View {
    @ObservedObject var compensatedKP: FloatAttribute
    
    var body: some View {
        HStack {
            Text("Compensated KP")
            Spacer()
            Text(String(compensatedKP.value ?? 0))
        }
    
    }
}

