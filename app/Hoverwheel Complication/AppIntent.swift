//
//  AppIntent.swift
//  Hoverwheel Complication
//
//  Created by Brandon Wees on 1/18/24.
//

import WidgetKit
import AppIntents

struct ConfigurationAppIntent: WidgetConfigurationIntent {
    static var title: LocalizedStringResource = "Configuration"
    static var description = IntentDescription("Open Hoverwheel")
}
