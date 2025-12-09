```mermaid

sequenceDiagram
    autonumber

    participant Analyzer as DTMF Detecter
    participant Stabilizer as DigitStabilizer
    participant Output as List of Detected Digits

    Note over Analyzer, Output: Process starts
    Analyzer ->> Stabilizer: sym = "?"
    Stabilizer ->> Stabilizer: State = IDLE

    Note over Analyzer, Output: When valid tone appears
    Analyzer ->> Stabilizer: Symbol appeared
    Stabilizer ->> Stabilizer: CANDIDATE the symbol

    Note over Analyzer, Output: Check symbol stability
    alt Symbol is stable (YES)
        Analyzer ->> Stabilizer: Symbol
        Stabilizer ->> Stabilizer: Update t_last_seen
    else Symbol not stable
        Stabilizer ->> Stabilizer: Return to IDLE / restart candidate
    end

    Note over Analyzer, Output: Check if stable ≥ hold_ms
    alt Stable long enough (YES)
        Stabilizer ->> Stabilizer: Enter LOCKED
        Stabilizer -->> Output: "Symbol"
    else Not stable long enough
        Stabilizer ->> Stabilizer: Continue monitoring
    end

    Note over Analyzer, Output: GAP - 78 ms
    alt GAP timer elapsed (YES)
        Stabilizer ->> Stabilizer: Ready for next digit
    else GAP not finished
        Stabilizer ->> Stabilizer: Keep waiting
    end

    Note over Analyzer, Output: New digit begins
    Analyzer ->> Stabilizer: Symbol


    ```