```mermaid
flowchart TD
    A[Input command] --> B[Computer plays DTMF sounds]
    B --> |waits 10 seckonds |C[Computer waits for NACK or ACK command]
    B --> D[Robot listens for DTMF sounds and decodes]
    D --> E[Runs CRC to check if command is valid]
    E --> F{is valid?}
    F -->|Yes| G[Run RoutePlanner with command]
    F -->|No| H[Play Nack command]
    H --> I{Does computer get NACK command?}
    C --> I
    I --> |yes| K[Computer retransmits command]
    K --> |Retransmits command| B 
    I --> |No| J[Ready for new command]
    M --> A
    C --> L{Does computer get ACK command?}   
    L --> |Yes| J 
    L --> |No| K

    # Mangler at implementere sequence nummer

```