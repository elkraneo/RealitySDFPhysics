// Port of ContentView from Apple's sample "Simulating dice rolls as a component for your game":
// https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game
//
// Apple's bottom ornament, with the last roll's score and the "Max score toss" and "Toss all"
// toggles, becomes a bar over the view. Dice are picked with the simulation's raycast.

import RealityKit
import SwiftUI

struct DiceView: View {
    @State private var scene = DiceScene()
    @State private var isLoaded = false
    @State private var error: String?
    @State private var maxScoreToss = false
    @State private var tossAll = false
    @State private var isHolding = false

    var body: some View {
        GeometryReader { geometry in
            RealityView { content in
                content.add(scene.content)
            }
            .gesture(pickUpAndToss(in: geometry.size))
        }
        .overlay(alignment: .bottom) {
            HStack(spacing: 16) {
                if let error {
                    Text(error).foregroundStyle(.red)
                } else if !isLoaded {
                    ProgressView().controlSize(.small)
                    Text("Baking the dice…")
                } else if let rolled = scene.rolled {
                    Text(Self.describe(rolled)).monospacedDigit()
                    if let predicted = scene.predicted {
                        let matches = predicted.map(\.value) == rolled.map(\.value)
                        Text(matches ? "as predicted" : "predicted \(Self.total(predicted))")
                            .foregroundStyle(.secondary)
                    }
                } else if scene.isRolling {
                    if let predicted = scene.predicted {
                        Text("Predicted \(Self.describe(predicted))…").monospacedDigit()
                    } else {
                        Text("Rolling…")
                    }
                } else {
                    Text("Drag a die to pick it up, and release to toss it")
                }
                Divider().frame(height: 16)
                Toggle("Max score toss", isOn: $maxScoreToss)
                Toggle("Toss all", isOn: $tossAll)
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 10)
            .background(.regularMaterial, in: .capsule)
            .padding(12)
        }
        .toolbar {
            Button("Reset", systemImage: "arrow.counterclockwise") { scene.reset() }
                .disabled(!isLoaded)
        }
        .task {
            do {
                try await scene.load()
                isLoaded = true
            } catch {
                self.error = "Could not load the dice: \(error.localizedDescription)"
            }
        }
    }

    private static func total(_ rolls: [DiceScene.Roll]) -> Int {
        rolls.map(\.value).reduce(0, +)
    }

    /// Each die and its value, and the total of several
    private static func describe(_ rolls: [DiceScene.Roll]) -> String {
        let dice = rolls.map { "\($0.kind.label) \($0.value)" }.joined(separator: " · ")
        return rolls.count > 1 ? "\(dice) = \(total(rolls))" : dice
    }

    private func pickUpAndToss(in size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                guard isLoaded else { return }
                if !isHolding {
                    isHolding = scene.pickUp(at: value.startLocation, in: size, tossAll: tossAll)
                }
                if isHolding { scene.move(to: value.location, in: size) }
            }
            .onEnded { _ in
                if isHolding { scene.toss(maxScore: maxScoreToss) }
                isHolding = false
            }
    }
}
