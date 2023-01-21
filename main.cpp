#include "mbed.h"

// Selecting the input pins
Serial sensor(p9, p10);

// Selecting the USB
Serial pc(USBTX, USBRX);

// Selecting LED1
DigitalOut myled(LED1);

// Arrays containing the binary information provided by the sensor
char Direction_Bits[4];
char Binary_Direction[16];
char Binary_Value[16];
char Ones_Complement[16];
char Twos_Complement[16];

// Integers containing the decimal numbers
int8_t A3_A0, D15_D8, D7_B0;
int16_t Data_Bits;
int32_t Data_Value;

// Calibration of the sensorial information
int fsx = 1150;
int fsy = 1110;
int fsz = 1850;
int msx = 56;
int msy = 52;
int msz = 61;

// Function that transforms into binary
void toBinaryBits(int16_t value, char *output) {
  int32_t i;
  output[16] = '\0';
  
  for (i = 15; i >= 0; --i, value >>= 1) {
    output[i] = (value & 1) + '0';
  }
}

// Function that obtains the data bits
int16_t DATA(int8_t x, int8_t y) {
  int16_t pow = 10;

  while (y >= pow)
    pow *= 10;

  return x * pow + y;
}

// Function that obtains the direction bits
void toDirectionBits(char *Binary, char *Direction) {
  for (int i = 15, j = 3; i > 11; i--, j--) {
    Direction[j] = Binary[i];
  }

  Direction[4] = '\0';
}

// Function that transforms Two's complement to One's Complement
void toOnesComplement(char *Twos_Complement, char *negacion) {
  int carry = 1;

  for (int i = 15; i >= 0; i--) {
    if (carry == 1 && Twos_Complement[i] == '1') {
      negacion[i] = '0';
      carry = 0;
    } else if (carry == 1 && Twos_Complement[i] == '0') {
      negacion[i] = '1';
    } else {
      negacion[i] = Twos_Complement[i];
    }
  }

  negacion[16] = '\0';
}

// Function that undos One's complement
void toBinary(char *negacion, char *result) {
  for (int i = 0; i < 16; i++) {
    if (negacion[i] == '1') {
      result[i] = '0';
    } else if (negacion[i] == '0') {
      result[i] = '1';
    }
  }

  result[16] = '\0';
}

// Function that transforms binary to decimal
int32_t toDecimal(char *bin) {
  int32_t dec;
  double p = 0;
  dec = 0;

  for (int i = 15; i >= 0; i--) {
    if (bin[i] == '1') {
      dec += pow((double)2, p);
    } else {
      dec = dec;
    }

    p++;
  }

  return dec;
}

// Function that takes the measured values and calibrates them
double FinalValue(int16_t Calibration, int16_t decimal_val) {
  double Force = (Calibration * decimal_val / 16384);
  double FinalForce = Force / 100;
  return FinalForce;
}

// Main
int main() {
  // Fixing the baud rate and bits
  sensor.format(8, Serial::None, 1);
  sensor.baud(115200);
  //pc.format(8, Serial::None, 1);
  pc.baud(115200);

  while (1) {
    // Making sure that the sensor is readable
    if (sensor.readable()) {
      // This led will blink, indicating that the program functions correctly
      myled = 1;

      // Getting the sensorial information
      A3_A0 = sensor.getc();
      wait(0.0234);
      D15_D8 = sensor.getc();
      wait(0.0468);
      D7_B0 = sensor.getc();
      Data_Bits = DATA(D15_D8, D7_B0);
      toBinaryBits(Data_Bits, Twos_Complement);
      toOnesComplement(Twos_Complement, Ones_Complement);
      toBinary(Ones_Complement, Binary_Value);
      Data_Value = toDecimal(Binary_Value);
      toBinaryBits(A3_A0, Binary_Direction);
      toDirectionBits(Binary_Direction, Direction_Bits);

      // Classifying the data according to the direction bits
      if (strcmp(Direction_Bits, "0001") == 0) {
        double Fxfinal = FinalValue(fsx, Data_Value);
        /*Fx*/ pc.printf("Fx = %lf\n", Fxfinal);
      } else if (strcmp(Direction_Bits, "0010") == 0) {
        double Fyfinal = FinalValue(fsy, Data_Value);
        /*Fy*/ pc.printf("Fy = %lf\n", Fyfinal);
      } else if (strcmp(Direction_Bits, "0011") == 0) {
        double Fzfinal = FinalValue(fsz, Data_Value);
        /*Fz*/ pc.printf("Fz = %lf\n", Fzfinal);
      } else if (strcmp(Direction_Bits, "0100") == 0) {
        double Mxfinal = FinalValue(msx, Data_Value);
        /*Mx*/ pc.printf("Mx = %lf\n", Mxfinal);
      } else if (strcmp(Direction_Bits, "0101") == 0) {
        double Myfinal = FinalValue(msy, Data_Value);
        /*My*/ pc.printf("My = %lf\n", Myfinal);
      } else if (strcmp(Direction_Bits, "0110") == 0) {
        double Mzfinal = FinalValue(msz, Data_Value);
        /*Mz*/ pc.printf("Mz = %lf\n", Mzfinal);
      }
    }
  }
}
