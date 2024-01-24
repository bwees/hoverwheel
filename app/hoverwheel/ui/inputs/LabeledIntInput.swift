//
//  LabeledTextView.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/17/24.
//

import SwiftUI

struct LabeledIntInput: View {
    @ObservedObject var setting: IntAttribute

    var body: some View {
        LabeledContent {
            if setting.value != nil {
                TextField(setting.name, value: Binding(
                    get: { setting.value ?? 0},
                    set: { setting.value = setting.value == nil ? 0 : $0 }
                    )
                  , format: .number)
                    .keyboardType(.numberPad)
                    .multilineTextAlignment(.trailing)
            } else {
                ProgressView()
                    .progressViewStyle(CircularProgressViewStyle())
            }
        } label: {
            Text(setting.name)
        }
    }
}
