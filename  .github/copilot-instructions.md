# Copilot Instructions for Semesterproject_3_Mobile_Robot_Systems

## Project Overview
This project is organized by subsystem, each in its own folder:
- `CommunicationProtocol/`: Handles the protocol for communication between robot components. See `Protocol.py` for the protocol class and `main.py` for usage examples.
- `DriveSystem/`: Contains code for the differential drive system.
- `Mapping/`: Intended for mapping-related code.
- `SignalProc/`: For signal processing logic.
- `SupplyDispencer/`: For supply dispenser logic.

## Key Patterns & Conventions
- Each subsystem has its own folder and a corresponding `.md` file describing its purpose.
- The communication protocol is implemented as a Python class (`protocol`) with methods for setting addresses and retrieving command dictionaries.
- Example usage of the protocol is in `CommunicationProtocol/main.py`.
- Naming conventions are simple and descriptive; class and method names are in lowercase with underscores.
- No build system or test framework is present; run Python files directly for testing (e.g., `python main.py` in a subsystem folder).

## Developer Workflows
- To test or run a subsystem, execute its `main.py` (if present) directly.
- There are no explicit requirements or dependency files; the codebase assumes a standard Python 3 environment.
- No external libraries are used as of now.

## Integration & Data Flow
- Communication between subsystems is expected to occur via the protocol defined in `CommunicationProtocol/Protocol.py`.
- Each subsystem is currently isolated; integration points should be documented in their respective folders as they are developed.

## Project-Specific Advice
- When adding new subsystems, follow the existing folder structure and document the purpose in a `.md` file.
- Keep protocol changes backward compatible or document breaking changes in `CommunicationProtocol/Protocol.md`.
- Use clear, descriptive names for new classes and methods.

## Example: Using the Protocol
```python
from Protocol import protocol

my_protocol = protocol("MyProtocol")
my_protocol.set_room_address(100)
my_protocol.set_supply_address("Supply456")
command = my_protocol.get_command()
print(command)
```

Refer to `CommunicationProtocol/main.py` for a runnable example.
