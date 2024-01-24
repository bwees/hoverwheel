//
//  Hoverwheel_Complication.swift
//  Hoverwheel Complication
//
//  Created by Brandon Wees on 1/18/24.
//

import WidgetKit
import SwiftUI

struct Provider: AppIntentTimelineProvider {
    func placeholder(in context: Context) -> SimpleEntry {
        SimpleEntry(date: Date(), configuration: ConfigurationAppIntent())
    }

    func snapshot(for configuration: ConfigurationAppIntent, in context: Context) async -> SimpleEntry {
        SimpleEntry(date: Date(), configuration: configuration)
    }
    
    func timeline(for configuration: ConfigurationAppIntent, in context: Context) async -> Timeline<SimpleEntry> {
        var entries: [SimpleEntry] = []

        // Generate a timeline consisting of five entries an hour apart, starting from the current date.
        let currentDate = Date()
        for hourOffset in 0 ..< 5 {
            let entryDate = Calendar.current.date(byAdding: .hour, value: hourOffset, to: currentDate)!
            let entry = SimpleEntry(date: entryDate, configuration: configuration)
            entries.append(entry)
        }

        return Timeline(entries: entries, policy: .atEnd)
    }

    func recommendations() -> [AppIntentRecommendation<ConfigurationAppIntent>] {
        // Create an array with all the preconfigured widgets to show.
        [AppIntentRecommendation(intent: ConfigurationAppIntent(), description: "Open Hoverwheel")]
    }
    
}

struct SimpleEntry: TimelineEntry {
    let date: Date
    let configuration: ConfigurationAppIntent
}

struct Hoverwheel_ComplicationEntryView : View {
    var entry: Provider.Entry

    var body: some View {
        ZStack {
            Circle()
                .fill(.quinary)
            Image("OneWheel")
                .scaleEffect(0.6)
                .widgetAccentable()
        }
    }
}

@main
struct Hoverwheel_Complication: Widget {
    let kind: String = "Hoverwheel_Complication"

    var body: some WidgetConfiguration {
        AppIntentConfiguration(kind: kind, intent: ConfigurationAppIntent.self, provider: Provider()) { entry in
            Hoverwheel_ComplicationEntryView(entry: entry)
                .containerBackground(.fill.tertiary, for: .widget)
        }
        .supportedFamilies([.accessoryCorner, .accessoryCircular, .accessoryRectangular])
    }
}

extension ConfigurationAppIntent {
    fileprivate static var defaultIntent: ConfigurationAppIntent {
        let intent = ConfigurationAppIntent()
        return intent
    }
}

#Preview(as: .accessoryRectangular) {
    Hoverwheel_Complication()
} timeline: {
    SimpleEntry(date: .now, configuration: .defaultIntent)
}
