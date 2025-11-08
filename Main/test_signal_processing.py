from Main.main import readCommand

def test_readCommand():
    command = readCommand()
    assert all(c in "01234567" for c in command), "Detected digits should be in range 0-7"
    
if __name__ == "__main__":
    test_readCommand()
    print("test_readCommand passed.")
    