//
//  ExtendedRuntimeManager.swift
//  hoverwheel-watch Watch App
//
//  Created by Brandon Wees on 1/18/24.
//

import Foundation
import WatchKit

class ExtendedRuntimeManager: NSObject, ObservableObject, Observable {
    private var session: WKExtendedRuntimeSession
    
    @Published var sessionState: WKExtendedRuntimeSessionState

    override init() {
        self.session = WKExtendedRuntimeSession()
        self.sessionState = self.session.state
    }
    
    func startSession() {
        
        if (self.session.state == .invalid) {
            self.session = WKExtendedRuntimeSession()
        }
        
        self.session.delegate = self
        self.session.start()
        self.sessionState = self.session.state

    }
    
    func endSession() {
        self.session.invalidate()
        self.sessionState = self.session.state
    }
}

extension ExtendedRuntimeManager: WKExtendedRuntimeSessionDelegate {
    func extendedRuntimeSession(_ extendedRuntimeSession: WKExtendedRuntimeSession, didInvalidateWith reason: WKExtendedRuntimeSessionInvalidationReason, error: Error?) {
        print("Extended Session Invalidated")
        self.sessionState = self.session.state
    }
    
    func extendedRuntimeSessionDidStart(_ extendedRuntimeSession: WKExtendedRuntimeSession) {
        print("Extended Session Started")
        self.sessionState = self.session.state
    }
    
    func extendedRuntimeSessionWillExpire(_ extendedRuntimeSession: WKExtendedRuntimeSession) {
        print("Extended Session Will Expire")
        self.sessionState = self.session.state
    }
}
