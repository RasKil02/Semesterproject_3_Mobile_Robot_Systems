def crc_remainder(input_bitstring, polynomial_bitstring, initial_filler='0'):
    """Calculate the CRC remainder of a string of bits using a given polynomial."""
    # Append zeros to the input (length = degree of polynomial - 1)
    polynomial_length = len(polynomial_bitstring) # Length of poly = 4
    input_padded = input_bitstring + initial_filler * (polynomial_length - 1) # "110110111011" + 0 * (4-1) = 110110111011000
    
    input_padded = list(input_padded) # Convert to list for mutability () ['1', '1', '0', '1', '1', '0', '1', '1', '1', '0', '1', '1', '0', '0', '0']
    polynomial = list(polynomial_bitstring) # Convert to list for mutability ['1', '0', '1', '1']
    
    for i in range(len(input_bitstring)): # Loop through each bit in the original input
        if input_padded[i] == '1': # Only perform division if the bit is 1
            for j in range(polynomial_length): # Perform XOR with the polynomial 4 times
                input_padded[i + j] = str(int(input_padded[i + j] != polynomial[j]))  # XOR operation on input_padded[i+j] with all bits in the polynomial
                # input_padded[i +j] = []
    
    # The remainder is the last bits after division
    remainder = ''.join(input_padded[-(polynomial_length - 1):])
    return remainder

# Eksempel:
data = "110110111011"  # 12 bits data
poly = "1011"          # CRC-3 generator polynomium

crc = crc_remainder(data, poly)
print("CRC:", crc)
