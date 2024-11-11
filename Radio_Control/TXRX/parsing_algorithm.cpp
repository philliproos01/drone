float stringToFloat(const char* str, int start, int end) {
    float result = 0.0f;
    float fraction = 0.1f;
    bool decimal = false;
    bool negative = false;
    
    for (int i = start; i < end; i++) {
        if (str[i] == '-') {
            negative = true;
        } else if (str[i] == '.') {
            decimal = true;
        } else if (str[i] >= '0' && str[i] <= '9') {
            if (!decimal) {
                result = result * 10.0f + (str[i] - '0');
            } else {
                result += (str[i] - '0') * fraction;
                fraction *= 0.1f;
            }
        }
    }
    
    return negative ? -result : result;
}

// Function to parse the string and fill the float array
int parseData(const char* input, float* output, int maxSize) {
    int count = 0;
    int start = 0;
    bool dataStarted = false;
    
    for (int i = 0; input[i] != '\0' && count < maxSize; i++) {
        if (!dataStarted) {
            if (input[i] == ':') {
                dataStarted = true;
                start = i + 1;
            }
        } else {
            if (input[i] == ' ' || input[i + 1] == '\0') {
                int end = (input[i + 1] == '\0') ? i + 1 : i;
                output[count++] = stringToFloat(input, start, end);
                start = i + 1;
            }
        }
    }
    
    return count;
}

// Example usage
void setup() {
    Serial.begin(9600);
    
    const char* dataString = "Data: 0.00 1.11 2.22 3.33 4.44 5.55 6.66 7.77 8.88";
    float dataArray[10];  // Assuming a maximum of 10 float values
    
    int dataCount = parseData(dataString, dataArray, 10);
    
    Serial.print("Parsed ");
    Serial.print(dataCount);
    Serial.println(" values:");
    
    for (int i = 0; i < dataCount; i++) {
        Serial.println(dataArray[i], 2);  // Print with 2 decimal places
    }
}

void loop() {
    // for later
}
