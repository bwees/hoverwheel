//
//  LabeledBoolInput.swift
//  hoverwheel
//
//  Created by Brandon Wees on 1/18/24.
//

import SwiftUI

struct LabeledBoolInput: View {
    @ObservedObject var setting: BoolAttribute
    var changeCallback: (() -> ())?
    
    var body: some View {
        LabeledContent {
            if setting.value != nil {
                Toggle("", isOn: Binding(
                    get: { setting.value ?? false },
                    set: {
                        setting.value = setting.value == nil ? false : $0
                        if changeCallback != nil {
                            changeCallback!()
                        }
                    }
                ))
                
            } else {
                ProgressView()
                    .progressViewStyle(CircularProgressViewStyle())
            }
        } label: {
            Text(setting.name)
        }
        
    }
}
